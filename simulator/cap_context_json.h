#ifndef CAMS_CAP_CONTEXT_JSON_H
#define CAMS_CAP_CONTEXT_JSON_H
// Strict bounded parser copied from the existing SF reader; kept isolated so
// legacy parsing and its error contract remain unchanged.
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
namespace cap_context_json {
[[noreturn]] inline void fail(const std::string& message) {
    throw std::runtime_error("Cap/context table: " + message);
}

struct Json {
    enum Kind { Null, Boolean, Number, String, Array, Object } kind = Null;
    bool boolean = false;
    double number = 0;
    // Retain lexical JSON integer identity for versioned ABC contracts. Legacy
    // readers intentionally keep their existing numeric validation unchanged.
    bool integer_token = false;
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
            result.integer_token = token.find_first_of(".eE") == std::string::npos;
            result.number = std::strtod(token.c_str(), &end);
            if (errno == ERANGE || !std::isfinite(result.number) || *end != '\0') fail("invalid finite JSON number");
            result.kind = Json::Number;
        }
        return result;
    }
};

inline double number(const Json& value, const std::string& name) {
    if (value.kind != Json::Number || !std::isfinite(value.number)) fail("numeric metadata required for " + name);
    return value.number;
}
inline double number(const Json& object, const std::string& key, double expected) {
    const double actual = number(object.at(key), key);
    if (actual != expected) fail("unsupported metadata " + key);
    return actual;
}
inline int integer(const Json& object, const std::string& key, int minimum, int maximum) {
    const double value = number(object.at(key), key);
    if (value < minimum || value > maximum || value != std::floor(value)) fail("invalid integer metadata " + key);
    return static_cast<int>(value);
}
inline std::string string_value(const Json& value, const std::string& name) {
    if (value.kind != Json::String) fail("string metadata required for " + name);
    return value.string;
}
inline void require_string(const Json& object, const std::string& key, const std::string& expected) {
    if (string_value(object.at(key), key) != expected) fail("unsupported metadata " + key);
}
}
#endif
