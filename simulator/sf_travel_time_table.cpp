#include "sf_travel_time_table.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
const std::size_t MAX_ROWS = 50000000;
const std::size_t MAX_METADATA_BYTES = 1024 * 1024;
const std::size_t MAX_BODY_LINE_BYTES = 4096;
const double RAW_TOLERANCE = 1e-12;
const double ROUNDING_EPSILON = 1e-9;
const int STATIC_SCALE = 10000;

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error("SF table: " + message);
}

struct Json {
    enum Kind { Null, Boolean, Number, String, Array, Object } kind = Null;
    bool boolean = false;
    double number = 0;
    std::string string;
    std::vector<Json> array;
    std::map<std::string, Json> object;

    const Json& at(const std::string& key) const {
        if (kind != Object) fail("expected JSON object for " + key);
        const auto found = object.find(key);
        if (found == object.end()) fail("missing metadata key " + key);
        return found->second;
    }
};

// Small strict JSON parser, used for one bounded metadata line only. It rejects
// duplicate keys at every nesting level and does not scan for unscoped min/max.
class JsonParser {
public:
    explicit JsonParser(const std::string& text) : text_(text) {}
    Json parse() {
        Json result = value(0);
        whitespace();
        if (position_ != text_.size()) fail("trailing JSON data");
        return result;
    }
private:
    const std::string& text_;
    std::size_t position_ = 0;
    std::size_t nodes_ = 0;
    void whitespace() {
        while (position_ < text_.size() && (text_[position_] == ' ' ||
               text_[position_] == '\t' || text_[position_] == '\r' || text_[position_] == '\n')) ++position_;
    }
    bool take(char c) {
        whitespace();
        if (position_ < text_.size() && text_[position_] == c) { ++position_; return true; }
        return false;
    }
    void expect(char c) { if (!take(c)) fail(std::string("JSON expected '") + c + "'"); }
    unsigned hex4() {
        if (position_ + 4 > text_.size()) fail("truncated JSON unicode escape");
        unsigned result = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = text_[position_++];
            result *= 16;
            if (c >= '0' && c <= '9') result += c - '0';
            else if (c >= 'a' && c <= 'f') result += c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') result += c - 'A' + 10;
            else fail("invalid JSON unicode escape");
        }
        return result;
    }
    static void utf8(std::string& output, unsigned code) {
        if (code <= 0x7f) output += static_cast<char>(code);
        else if (code <= 0x7ff) {
            output += static_cast<char>(0xc0 | (code >> 6));
            output += static_cast<char>(0x80 | (code & 63));
        } else if (code <= 0xffff) {
            output += static_cast<char>(0xe0 | (code >> 12));
            output += static_cast<char>(0x80 | ((code >> 6) & 63));
            output += static_cast<char>(0x80 | (code & 63));
        } else {
            output += static_cast<char>(0xf0 | (code >> 18));
            output += static_cast<char>(0x80 | ((code >> 12) & 63));
            output += static_cast<char>(0x80 | ((code >> 6) & 63));
            output += static_cast<char>(0x80 | (code & 63));
        }
    }
    std::string string() {
        expect('"');
        std::string result;
        while (position_ < text_.size()) {
            const unsigned char c = static_cast<unsigned char>(text_[position_++]);
            if (c == '"') return result;
            if (c < 0x20) fail("unescaped control character in JSON string");
            if (c != '\\') { result += static_cast<char>(c); continue; }
            if (position_ == text_.size()) fail("truncated JSON escape");
            switch (text_[position_++]) {
                case '"': result += '"'; break;
                case '\\': result += '\\'; break;
                case '/': result += '/'; break;
                case 'b': result += '\b'; break;
                case 'f': result += '\f'; break;
                case 'n': result += '\n'; break;
                case 'r': result += '\r'; break;
                case 't': result += '\t'; break;
                case 'u': {
                    unsigned code = hex4();
                    if (code >= 0xd800 && code <= 0xdbff) {
                        if (position_ + 2 > text_.size() || text_.substr(position_, 2) != "\\u")
                            fail("unpaired high surrogate in JSON");
                        position_ += 2;
                        const unsigned low = hex4();
                        if (low < 0xdc00 || low > 0xdfff) fail("invalid low surrogate in JSON");
                        code = 0x10000 + ((code - 0xd800) << 10) + low - 0xdc00;
                    } else if (code >= 0xdc00 && code <= 0xdfff) fail("unpaired low surrogate in JSON");
                    utf8(result, code);
                    break;
                }
                default: fail("invalid JSON string escape");
            }
        }
        fail("unterminated JSON string");
    }
    bool literal(const char* word) {
        const std::string token(word);
        if (text_.compare(position_, token.size(), token) != 0) return false;
        position_ += token.size();
        return true;
    }
    Json value(unsigned depth) {
        if (depth > 32 || ++nodes_ > 100000) fail("JSON complexity limit exceeded");
        whitespace();
        if (position_ == text_.size()) fail("truncated JSON value");
        Json result;
        const char c = text_[position_];
        if (c == '{') {
            ++position_; result.kind = Json::Object;
            if (take('}')) return result;
            do {
                const std::string key = string();
                expect(':');
                if (!result.object.emplace(key, value(depth + 1)).second) fail("duplicate JSON key " + key);
            } while (take(','));
            expect('}');
        } else if (c == '[') {
            ++position_; result.kind = Json::Array;
            if (take(']')) return result;
            do { result.array.push_back(value(depth + 1)); } while (take(','));
            expect(']');
        } else if (c == '"') { result.kind = Json::String; result.string = string(); }
        else if (literal("true")) { result.kind = Json::Boolean; result.boolean = true; }
        else if (literal("false")) { result.kind = Json::Boolean; result.boolean = false; }
        else if (literal("null")) { result.kind = Json::Null; }
        else {
            const std::size_t start = position_;
            if (text_[position_] == '-') ++position_;
            if (position_ >= text_.size()) fail("invalid JSON number");
            if (text_[position_] == '0') ++position_;
            else {
                if (text_[position_] < '1' || text_[position_] > '9') fail("invalid JSON value");
                while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) ++position_;
            }
            if (position_ < text_.size() && text_[position_] == '.') {
                ++position_; const std::size_t first = position_;
                while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) ++position_;
                if (first == position_) fail("invalid JSON fraction");
            }
            if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E')) {
                ++position_;
                if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-')) ++position_;
                const std::size_t first = position_;
                while (position_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[position_]))) ++position_;
                if (first == position_) fail("invalid JSON exponent");
            }
            errno = 0;
            char* end = nullptr;
            const std::string token = text_.substr(start, position_ - start);
            result.number = std::strtod(token.c_str(), &end);
            if (errno == ERANGE || !std::isfinite(result.number) || *end != '\0') fail("invalid finite JSON number");
            result.kind = Json::Number;
        }
        return result;
    }
};

double number(const Json& value, const std::string& name) {
    if (value.kind != Json::Number || !std::isfinite(value.number)) fail("numeric metadata required for " + name);
    return value.number;
}
double number(const Json& object, const std::string& key, double expected) {
    const double actual = number(object.at(key), key);
    if (actual != expected) fail("unsupported metadata " + key);
    return actual;
}
int integer(const Json& object, const std::string& key, int minimum, int maximum) {
    const double value = number(object.at(key), key);
    if (value < minimum || value > maximum || value != std::floor(value)) fail("invalid integer metadata " + key);
    return static_cast<int>(value);
}
std::string string_value(const Json& value, const std::string& name) {
    if (value.kind != Json::String) fail("string metadata required for " + name);
    return value.string;
}
void require_string(const Json& object, const std::string& key, const std::string& expected) {
    if (string_value(object.at(key), key) != expected) fail("unsupported metadata " + key);
}
std::int64_t quantize(double value, int scale, const std::string& name) {
    if (!std::isfinite(value) || value <= 0 || value * scale > 9e15) fail("invalid " + name);
    return static_cast<std::int64_t>(std::floor(value * scale + 0.5 + ROUNDING_EPSILON));
}
bool line(std::istream& input, std::string& output, std::size_t max_bytes) {
    if (!std::getline(input, output)) return false;
    if (output.size() > max_bytes) fail("line size limit exceeded");
    if (!output.empty() && output.back() == '\r') output.pop_back();
    return true;
}
void parse_row(const std::string& text, double (&fields)[6]) {
    if (text.find('\0') != std::string::npos) fail("embedded NUL in body row");
    const char* cursor = text.c_str();
    for (int i = 0; i < 6; ++i) {
        while (*cursor && std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
        if (!*cursor) fail("body row has fewer than six values");
        errno = 0;
        char* end = nullptr;
        fields[i] = std::strtod(cursor, &end);
        if (end == cursor || errno == ERANGE || !std::isfinite(fields[i])) fail("invalid finite body value");
        if (*end && !std::isspace(static_cast<unsigned char>(*end))) fail("body values must be whitespace-delimited");
        cursor = end;
    }
    while (*cursor && std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if (*cursor) fail("body row has extra data");
}
std::pair<double, double> training_range(const Json& ranges, const std::string& name) {
    const Json& range = ranges.at(name);
    const double lo = number(range.at("min"), name + ".min");
    const double hi = number(range.at("max"), name + ".max");
    if (lo < 0 || hi < lo || std::floor(lo) != lo || std::floor(hi) != hi)
        fail("invalid training flow range " + name);
    return {lo, hi};
}
}

void SfTravelTimeTable::clear() {
    *this = SfTravelTimeTable();
}

void SfTravelTimeTable::load(std::istream& input) {
    clear();
    try { load_checked(input); }
    catch (...) { clear(); throw; }
}

std::size_t SfTravelTimeTable::flow_offset(int roadFlow, int laneFlow) const {
    const std::size_t road = static_cast<std::size_t>(roadFlow);
    const std::size_t lane_max = static_cast<std::size_t>(lane_flow_max_);
    const std::size_t prefix = road <= lane_max ? road * (road + 1) / 2 :
        (lane_max + 1) * (lane_max + 2) / 2 + (road - lane_max - 1) * (lane_max + 1);
    return prefix + static_cast<std::size_t>(laneFlow);
}

void SfTravelTimeTable::load_checked(std::istream& input) {
    std::string text;
    if (!line(input, text, MAX_METADATA_BYTES)) fail("missing metadata line");
    const Json plan = JsonParser(text).parse();
    require_string(plan, "format", "CAMS_SF_TABLE_V1");
    require_string(plan, "training_schema", "road-time-core3-sf-speed-limit-v1");
    const std::vector<std::string> columns = {"road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit", "travel_time_no_waiting"};
    const Json& raw_columns = plan.at("columns");
    if (raw_columns.kind != Json::Array || raw_columns.array.size() != columns.size()) fail("invalid columns schema");
    for (std::size_t i = 0; i < columns.size(); ++i)
        if (string_value(raw_columns.array[i], "columns") != columns[i]) fail("unsupported columns schema");
    label_ = string_value(plan.at("source_label"), "source_label");
    const std::vector<std::string> labels = {"travel_time_label_gt_0p1", "travel_time_label_gt_0p5", "travel_time_label_gt_1",
        "travel_time_label_gt_1p5", "travel_time_label_gt_2", "travel_time_label_gt_3", "travel_time_label"};
    if (std::find(labels.begin(), labels.end(), label_) == labels.end()) fail("unsupported source label");
    number(plan, "static_key_scale", STATIC_SCALE);
    require_string(plan, "static_key_rounding", "floor(value * scale + 0.5 + 1e-9)");
    number(plan, "flow_min", 0);
    number(plan, "flow_step", 1);
    require_string(plan, "flow_constraint", "lane_flow <= road_flow");
    number(plan, "prediction_floor_seconds", 1.0);
    require_string(plan, "lookup_policy", "exact static/flow keys and nearest SF node; no interpolation or clamping");
    require_string(plan, "scope", "external-road time only; no junction/wait addition and no division by SF");
    if (plan.at("legacy_cpp_compatible").kind != Json::Boolean || plan.at("legacy_cpp_compatible").boolean)
        fail("legacy_cpp_compatible must be false");
    road_flow_max_ = integer(plan, "road_flow_max", 0, 1000000);
    lane_flow_max_ = integer(plan, "lane_flow_max", 0, road_flow_max_);
    const Json& grid = plan.at("speed_factor_grid");
    sf_scale_ = integer(grid, "scale", 1, 1000000);
    sf_index_min_ = integer(grid, "index_min", 1, 1000000000);
    sf_index_max_ = integer(grid, "index_max", sf_index_min_, 1000000000);
    sf_min_ = number(grid, "min", static_cast<double>(sf_index_min_) / sf_scale_);
    sf_max_ = number(grid, "max", static_cast<double>(sf_index_max_) / sf_scale_);
    number(grid, "step", 1.0 / sf_scale_);
    number(grid, "scaled_boundary_tolerance", ROUNDING_EPSILON);
    number(grid, "raw_boundary_tolerance", RAW_TOLERANCE);
    require_string(grid, "rounding", "nearest, positive half-up: floor(SF * scale + 0.5 + 1e-9)");
    require_string(grid, "out_of_range", "error; no clamp and no implicit formula fallback");
    sf_count_ = static_cast<std::size_t>(sf_index_max_ - sf_index_min_) + 1;
    flow_count_ = flow_offset(road_flow_max_, std::min(road_flow_max_, lane_flow_max_)) + 1;
    const Json& pairs = plan.at("static_pairs");
    if (pairs.kind != Json::Array || pairs.array.empty() || pairs.array.size() > 10000) fail("invalid static pair count");
    std::vector<std::pair<double, double>> static_values;
    for (const Json& pair : pairs.array) {
        const double length = number(pair.at("road_length"), "road_length");
        const double speed = number(pair.at("speed_limit"), "speed_limit");
        const StaticKey key(quantize(length, STATIC_SCALE, "road_length"), quantize(speed, STATIC_SCALE, "speed_limit"));
        if (std::abs(length - static_cast<double>(key.first) / STATIC_SCALE) > 1e-8 ||
            std::abs(speed - static_cast<double>(key.second) / STATIC_SCALE) > 1e-8) fail("static metadata exceeds four decimal precision");
        if (!static_indices_.emplace(key, static_values.size()).second) fail("duplicate static pair key");
        static_values.emplace_back(length, speed);
    }
    const double declared = number(plan.at("planned_rows"), "planned_rows");
    if (declared < 1 || declared > MAX_ROWS || declared != std::floor(declared)) fail("planned_rows outside memory guard");
    if (flow_count_ > MAX_ROWS || sf_count_ > MAX_ROWS / flow_count_ ||
        static_values.size() > MAX_ROWS / (flow_count_ * sf_count_)) fail("derived table exceeds memory guard");
    const std::size_t count = static_values.size() * sf_count_ * flow_count_;
    if (declared != static_cast<double>(count)) fail("planned_rows differs from derived grid count");
    const Json& ranges = plan.at("training_ranges");
    const auto road_range = training_range(ranges, "road_flow");
    const auto lane_range = training_range(ranges, "lane_flow");
    training_road_min_ = road_range.first; training_road_max_ = road_range.second;
    training_lane_min_ = lane_range.first; training_lane_max_ = lane_range.second;
    if (!line(input, text, MAX_BODY_LINE_BYTES)) fail("missing column header");
    std::istringstream header(text);
    std::string token;
    for (const std::string& column : columns)
        if (!(header >> token) || token != column) fail("body header does not match columns");
    if (header >> token) fail("extra body header column");
    values_.reserve(count);
    for (const auto& pair : static_values) {
        for (int sf_index = sf_index_min_; sf_index <= sf_index_max_; ++sf_index) {
            for (int road = 0; road <= road_flow_max_; ++road) {
                for (int lane = 0; lane <= std::min(road, lane_flow_max_); ++lane) {
                    if (!line(input, text, MAX_BODY_LINE_BYTES)) fail("truncated body at row " + std::to_string(values_.size() + 1));
                    double row[6];
                    parse_row(text, row);
                    if (row[0] != pair.first || row[1] != road || row[2] != lane ||
                        row[3] != static_cast<double>(sf_index) / sf_scale_ || row[4] != pair.second)
                        fail("body coordinate/order mismatch at row " + std::to_string(values_.size() + 1));
                    if (row[5] < 1.0) fail("prediction below one-second floor at row " + std::to_string(values_.size() + 1));
                    values_.push_back(row[5]);
                }
            }
        }
    }
    // Exporter writes exactly the planned rows. Even blank extra lines are not
    // silently accepted, so concatenations and incomplete exports fail closed.
    if (line(input, text, MAX_BODY_LINE_BYTES)) fail("extra data after planned body");
    if (input.bad() || !input.eof()) fail("I/O error while reading table");
}

double SfTravelTimeTable::lookup(double roadLength, int roadFlow, int laneFlow,
                                double speedFactor, double speedLimit) const {
    if (values_.empty()) fail("lookup requires a successfully loaded table");
    if (roadFlow < 0 || roadFlow > road_flow_max_ || laneFlow < 0 || laneFlow > lane_flow_max_ || laneFlow > roadFlow)
        fail("flow outside table domain: road=" + std::to_string(roadFlow) + " lane=" + std::to_string(laneFlow));
    const StaticKey key(quantize(roadLength, STATIC_SCALE, "road_length"), quantize(speedLimit, STATIC_SCALE, "speed_limit"));
    const auto pair = static_indices_.find(key);
    if (pair == static_indices_.end()) fail("missing road_length/speed_limit static key");
    if (!std::isfinite(speedFactor) || speedFactor <= 0 || speedFactor < sf_min_ - RAW_TOLERANCE || speedFactor > sf_max_ + RAW_TOLERANCE)
        fail("speed_factor outside table raw domain: " + std::to_string(speedFactor));
    const std::int64_t sf_index = quantize(speedFactor, sf_scale_, "speed_factor");
    if (sf_index < sf_index_min_ || sf_index > sf_index_max_) fail("rounded speed_factor outside table domain");
    const std::size_t index = (pair->second * sf_count_ + static_cast<std::size_t>(sf_index - sf_index_min_)) * flow_count_ + flow_offset(roadFlow, laneFlow);
    return values_[index];
}
