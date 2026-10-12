#include "head.h"

namespace {
void require_open(const std::ifstream& stream, const std::string& path, const std::string& label) {
    if (!stream) {
        throw std::runtime_error(label + ": cannot open required file '" + path + "'");
    }
}
}


// Read Graph
void Graph::read_graph()
{

    // 使用当前主数据源 "BJ" 构建基础路网（节点、边、RoadID 映射）
    ifstream IF(BJ);    //ID1, ID2, Weight (Length in Meters)
    require_open(IF, BJ, "read_graph BJ graph");
    //Read Node Number and Edge Number
    IF>>nodenum>>edgenum; // 296710 774660

    // Variable Initialization
    nodeID2RoadID.clear();
    roadID2NodeID.clear();
    vector<pair<int,int>> vecp;
    vecp.clear();
    vector<pair<int, double>> vecp_float;
    vecp_float.clear();
    graphLength.assign(nodenum, vecp_float);         //ID1, ID2, Length
    graphRoadID.assign(nodenum, vecp);         //ID1, ID2, RoadID
    roadInfor.clear();                         //Road Info (ID1, ID2, RoadID, Length, Time)
    set<int> setp; setp.clear();
    adjNodes.assign(nodenum, setp);            //ID1, ID2
    int ID1, ID2, edgeID;
    double length;
    // Avoid Redundant Edge
    set<pair<int,int>> edgeRedun;              //ID1, ID2
    // Read Graph Length as Weight
    for(int i=0;i<edgenum;i++){
        IF >> ID1 >> ID2 >> edgeID >> length;
        //Store Road Information
        Road r;
        r.ID1 = ID1;
        r.ID2 = ID2;
        r.roadID = edgeID;
        r.length = length;
        r.travelTime = -1;
        if (edgeID >= static_cast<int>(roadInfor.size())) {
            roadInfor.resize(edgeID + 1);
        }
        roadInfor[edgeID] = r;
        //Construct Maps
        nodeID2RoadID.insert(make_pair(make_pair(ID1, ID2), edgeID));    //<ID1, ID2> -> RoadID
        roadID2NodeID.insert(make_pair(edgeID, make_pair(ID1, ID2)));    //RoadID -> <ID1, ID2>
        //Check Redundant
        if(edgeRedun.find(make_pair(ID1,ID2))==edgeRedun.end())
        {
            graphLength[ID1].push_back(make_pair(ID2, length));
            graphRoadID[ID1].push_back(make_pair(ID2, edgeID));
            adjNodes[ID1].insert(ID2);
        }
        edgeRedun.insert(make_pair(ID1,ID2));
    }
    // Open File "BJ_minTravleTime"
    ifstream IFTime(BJ_minTravleTime);
    require_open(IFTime, BJ_minTravleTime, "read_graph BJ min travel time");
    // Read Node Number
    int nodenum, edgenum;
    IFTime>>nodenum>>edgenum; // 296710
    //Variable Initialization
    vector<pair<int,double>> vecpTIme; vecpTIme.clear();
    graphTime.assign(nodenum, vecpTIme);
    int ID1Time, ID2Time;
    double minTime;
    // Avoid Redundant Information
    set<pair<int,int>> edgeRedunTime;
    // Read Graph Min Travel Time as Weight
    for(int i=0;i<edgenum;i++){
        IFTime>>ID1>>ID2>>minTime;
        // Construct Maps
        nodeID2minTime.insert(make_pair(make_pair(ID1, ID2), minTime));  //<ID1, ID2> -> Min Travel Time
        // Check Redundant
        if(edgeRedunTime.find(make_pair(ID1,ID2))==edgeRedunTime.end()){
            graphTime[ID1].push_back(make_pair(ID2, minTime));
        }
        edgeRedunTime.insert(make_pair(ID1,ID2));
    }
}

// Read Road Information from Path "beijingMoreRoadInfo"
// More Road Information Contains in Another File
void Graph::read_road_info()
{
    // Read File
    ifstream IFNodeRoadID(beijingMoreRoadInfo);
    require_open(IFNodeRoadID, beijingMoreRoadInfo, "read_road_info road info");
    // Variable Initialization
    int nodeID1;
    int nodeID2;
    int routeIDLei;
    double length;
    int direction;
    double speedLimit;
    int laneNum;
    int width;
    int kindNumber;
    string kind;
    int routeIDGen;
    roadInforMore.clear();
    roadInforMore.reserve(400000);
    // Read In Info
    while(IFNodeRoadID >> nodeID1){
        IFNodeRoadID >> nodeID2 >> routeIDLei >> length >> direction;
        IFNodeRoadID >> speedLimit >> laneNum >> width >> kindNumber >> kind;
        auto it = nodeID2RoadID.find(make_pair(nodeID1, nodeID2));
        if (it == nodeID2RoadID.end()) {
            continue;
        }
        routeIDGen = it->second;

        roadMore info;
        info.nodeID1 = nodeID1;
        info.nodeID2 = nodeID2;
        info.routeID = routeIDGen;
        info.length = length;
        info.direction = direction;
        info.speedLimit = speedLimit;
        info.laneNum = laneNum;
        info.width = width;
        info.kindNumber = kindNumber;
        info.kind = kind;
        roadInforMore.push_back(info);
    }
    // 将补充属性回填到 roadInfor[roadID]，供后续新算法结构化阶段直接读取
    for (const auto &info : roadInforMore)
    {
        if (info.routeID < 0 || info.routeID >= static_cast<int>(roadInfor.size())) {
            continue;
        }
        roadInfor[info.routeID].length = info.length;
        roadInfor[info.routeID].direction = info.direction;
        roadInfor[info.routeID].speedLimit = info.speedLimit;
        roadInfor[info.routeID].laneNum = info.laneNum;
        roadInfor[info.routeID].width = info.width;
        roadInfor[info.routeID].kindNumber = info.kindNumber;
        roadInfor[info.routeID].kind = info.kind;
    }
}

namespace {
struct XmlTagLite {
    string name;
    map<string, string> attrs;
    bool closing = false;
    bool selfClosing = false;
};

string trim_copy(const string &s) {
    size_t b = 0;
    while (b < s.size() && isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

vector<string> split_ws(const string &s) {
    vector<string> out;
    string token;
    istringstream iss(s);
    while (iss >> token) out.push_back(token);
    return out;
}

int to_int_attr(const map<string, string> &attrs, const string &key, int def = 0) {
    auto it = attrs.find(key);
    if (it == attrs.end() || it->second.empty()) return def;
    try { return stoi(it->second); } catch (...) { return def; }
}

double to_double_attr(const map<string, string> &attrs, const string &key, double def = 0.0) {
    auto it = attrs.find(key);
    if (it == attrs.end() || it->second.empty()) return def;
    try { return stod(it->second); } catch (...) { return def; }
}

// New R/G contracts must not inherit the legacy parser's silent defaults or
// prefix acceptance (e.g. offset="bad", duration="24junk", linkIndex="1x").
// Used for R/G and opt-in road-pair calendar validation; legacy grouping keeps
// its existing input behavior. Missing offset is a defined zero, not an error.
double strict_rg_xml_real(const map<string, string>& attrs, const string& key,
                          const string& context, bool positive, bool optional_zero = false) {
    const auto found = attrs.find(key);
    if (found == attrs.end() && optional_zero) return 0.0;
    const string raw = found == attrs.end() ? "" : found->second;
    const string value = trim_copy(raw);
    try {
        size_t consumed = 0;
        const double number = stod(value, &consumed);
        if (consumed == value.size() && std::isfinite(number) && (!positive || number > 0.0))
            return number;
    } catch (const std::exception&) {
        // The context-rich error below also handles range and empty input.
    }
    throw runtime_error("R/G " + context + " " + key + " must be a complete finite "
                        + (positive ? "positive number" : "number") + ", got '" + raw + "'");
}

int strict_rg_xml_index(const map<string, string>& attrs, const string& key,
                        const string& context, bool optional_missing = false) {
    const auto found = attrs.find(key);
    if (found == attrs.end() && optional_missing) return -1;
    const string raw = found == attrs.end() ? "" : found->second;
    const string value = trim_copy(raw);
    try {
        size_t consumed = 0;
        const int number = stoi(value, &consumed);
        if (consumed == value.size() && number >= 0) return number;
    } catch (const std::exception&) {
        // Fail rather than selecting a different signal link or lane.
    }
    throw runtime_error("R/G " + context + " " + key
                        + " must be a complete nonnegative integer, got '" + raw + "'");
}

double strict_positive_xml_number(const string& raw, const string& context) {
    const string value = trim_copy(raw);
    try {
        size_t parsed = 0;
        const double number = stod(value, &parsed);
        if (parsed == value.size() && std::isfinite(number) && number > 0.0) {
            return number;
        }
    } catch (const std::exception&) {
        // Use one actionable diagnostic for syntax, range, and finite checks.
    }
    throw runtime_error(context + " must be a finite positive number, got '" + raw + "'");
}

double sumo_type_number(const map<string, string>& attrs, const string& key,
                        double fallback, const string& context, bool allow_zero = false) {
    const auto found = attrs.find(key);
    if (found == attrs.end()) return fallback;
    const string value = trim_copy(found->second);
    try {
        size_t consumed = 0;
        const double number = stod(value, &consumed);
        if (consumed == value.size() && std::isfinite(number) &&
            (allow_zero ? number >= 0.0 : number > 0.0) && (key != "sigma" || number <= 1.0))
            return number;
    } catch (const std::exception&) {}
    throw runtime_error(context + " " + key + " has invalid finite numeric value '" + found->second + "'");
}

VehicleType sumo_vehicle_type_defaults(const string& id, const string& vehicleClass) {
    // SUMO v1_27_0 SUMOVTypeParameter::{VClassDefaultValues,getDefaultAccel,
    // getDefaultDecel,getDefaultImperfection}; lengths: Vehicle_Type_Parameter_Defaults.
    // Only the road classes verified here are supported. Do not silently turn
    // an unknown/non-road class into a passenger car. Legacy VehicleType stays unchanged.
    VehicleType type;
    type.id = id;
    type.vehicleClass = vehicleClass;
    type.maxSpeed = 200.0 / 3.6;
    if (vehicleClass == "passenger" || vehicleClass == "private" || vehicleClass == "vip" ||
        vehicleClass == "hov" || vehicleClass == "taxi" || vehicleClass == "evehicle" ||
        vehicleClass == "custom1" || vehicleClass == "custom2") {
        return type;
    } else if (vehicleClass == "delivery" || vehicleClass == "emergency") {
        type.length = 6.5;
    } else if (vehicleClass == "truck" || vehicleClass == "trailer") {
        type.maxSpeed = 130.0 / 3.6;
        type.length = vehicleClass == "truck" ? 7.1 : 16.5;
        type.accel = vehicleClass == "truck" ? 1.3 : 1.1;
        type.decel = 4.0;
    } else if (vehicleClass == "bus" || vehicleClass == "coach") {
        type.maxSpeed = 100.0 / 3.6;
        type.length = vehicleClass == "bus" ? 12.0 : 14.0;
        type.accel = vehicleClass == "bus" ? 1.2 : 2.0;
        type.decel = 4.0;
    } else if (vehicleClass == "motorcycle") {
        type.length = 2.2; type.accel = 6.0; type.decel = 10.0;
    } else if (vehicleClass == "moped") {
        type.maxSpeed = 60.0 / 3.6;
        type.length = 2.1; type.accel = 1.1; type.decel = 7.0;
    } else if (vehicleClass == "bicycle") {
        type.maxSpeed = 50.0 / 3.6; type.desiredMaxSpeed = 20.0 / 3.6;
        type.length = 1.6; type.minGap = 0.5; type.accel = 1.2; type.decel = 3.0;
    } else {
        throw runtime_error("SUMO vType=" + id + " unsupported vClass=" + vehicleClass +
                            "; no passenger default substitution is permitted");
    }
    return type;
}

string to_string_attr(const map<string, string> &attrs, const string &key, const string &def = "") {
    auto it = attrs.find(key);
    return (it == attrs.end()) ? def : it->second;
}

bool parse_xml_tag_lite(const string &raw, XmlTagLite &tag) {
    string s = trim_copy(raw);
    if (s.empty() || s[0] == '?' || s.rfind("!--", 0) == 0 || s.rfind("!", 0) == 0) return false;
    if (!s.empty() && s.back() == '/') {
        tag.selfClosing = true;
        s.pop_back();
        s = trim_copy(s);
    }
    if (!s.empty() && s[0] == '/') {
        tag.closing = true;
        s = trim_copy(s.substr(1));
    }
    size_t pos = 0;
    while (pos < s.size() && !isspace(static_cast<unsigned char>(s[pos]))) ++pos;
    tag.name = s.substr(0, pos);
    while (pos < s.size()) {
        while (pos < s.size() && isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        if (pos >= s.size()) break;
        size_t keyStart = pos;
        while (pos < s.size() && s[pos] != '=' && !isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        string key = s.substr(keyStart, pos - keyStart);
        while (pos < s.size() && isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        if (pos >= s.size() || s[pos] != '=') break;
        ++pos;
        while (pos < s.size() && isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        if (pos >= s.size()) break;
        char quote = s[pos];
        string value;
        if (quote == '\'' || quote == '"') {
            ++pos;
            size_t valueStart = pos;
            while (pos < s.size() && s[pos] != quote) ++pos;
            value = s.substr(valueStart, pos - valueStart);
            if (pos < s.size()) ++pos;
        } else {
            size_t valueStart = pos;
            while (pos < s.size() && !isspace(static_cast<unsigned char>(s[pos]))) ++pos;
            value = s.substr(valueStart, pos - valueStart);
        }
        tag.attrs[key] = value;
    }
    return !tag.name.empty();
}

vector<XmlTagLite> parse_xml_tags_lite(const string &xml) {
    vector<XmlTagLite> tags;
    size_t pos = 0;
    while (true) {
        size_t open = xml.find('<', pos);
        if (open == string::npos) break;
        if (xml.compare(open, 4, "<!--") == 0) {
            const size_t end = xml.find("-->", open + 4);
            if (end == string::npos) throw runtime_error("Unclosed XML comment");
            pos = end + 3;
            continue;
        }
        size_t close = xml.find('>', open + 1);
        if (close == string::npos) break;
        XmlTagLite tag;
        if (parse_xml_tag_lite(xml.substr(open + 1, close - open - 1), tag)) {
            tags.push_back(tag);
        }
        pos = close + 1;
    }
    return tags;
}

bool is_sumo_internal_edge(const string &edgeID, const string &function) {
    return (!edgeID.empty() && edgeID[0] == ':') || function == "internal";
}

char turn_dir_to_sumo_char(TurnDir d) {
    if (d == TurnDir::Left) return 'l';
    if (d == TurnDir::Straight) return 's';
    if (d == TurnDir::Right) return 'r';
    if (d == TurnDir::UTurn) return 't';
    return '?';
}

string signal_state_to_string(SignalState s) {
    if (s == SignalState::Green) return "Green";
    if (s == SignalState::Yellow) return "Yellow";
    if (s == SignalState::AlwaysOpen) return "AlwaysOpen";
    return "Red";
}
}


void Graph::read_sumo_route_xml(const string& routeXmlPath, int maxVehicles)
{
    ifstream in(routeXmlPath.c_str());
    require_open(in, routeXmlPath, "read_sumo_route_xml SUMO route XML");
    stringstream buffer;
    buffer << in.rdbuf();
    vector<XmlTagLite> tags = parse_xml_tags_lite(buffer.str());

    queryDataRaw.clear();
    routeRoadID.clear();
    routeMovementID.clear();
    sumoVehicleIDs.clear();
    vehicleTypeIDs.clear();
    vehicleSpeedFactors.clear();
    vehicleHasExplicitSpeedFactor.clear();
    vehicleDepartureTimes.clear();
    vehicleDeparturePositions.clear();
    vehicleDeparturePositionIssues.clear();
    sumoVehicleTypes.clear();
    sumoVehicleTypes["DEFAULT_VEHTYPE"] = sumo_vehicle_type_defaults("DEFAULT_VEHTYPE", "passenger");

    unordered_map<string, vector<string>> sumoRouteIDToEdges;
    set<string> declaredTypeIDs;
    bool insideVehicle = false;
    for (const auto &tag : tags) {
        if (tag.name == "vTypeDistribution" && !tag.closing)
            throw runtime_error("SUMO vTypeDistribution requires resolved per-vehicle types; random type selection is unsupported");
        if (tag.name == "vType" && !tag.closing) {
            const string id = to_string_attr(tag.attrs, "id");
            if (id.empty() || !declaredTypeIDs.insert(id).second)
                throw runtime_error("SUMO vType requires a nonempty unique id: " + id);
            const string vehicleClass = to_string_attr(tag.attrs, "vClass", "passenger");
            VehicleType vt = sumo_vehicle_type_defaults(id, vehicleClass);
            const string context = routeXmlPath + ": vType=" + id;
            vt.accel = sumo_type_number(tag.attrs, "accel", vt.accel, context);
            vt.decel = sumo_type_number(tag.attrs, "decel", vt.decel, context);
            vt.sigma = sumo_type_number(tag.attrs, "sigma", vt.sigma, context, true);
            vt.length = sumo_type_number(tag.attrs, "length", vt.length, context);
            vt.minGap = sumo_type_number(tag.attrs, "minGap", vt.minGap, context, true);
            vt.maxSpeed = sumo_type_number(tag.attrs, "maxSpeed", vt.maxSpeed, context);
            vt.desiredMaxSpeed = sumo_type_number(tag.attrs, "desiredMaxSpeed", vt.desiredMaxSpeed, context);
            sumoVehicleTypes[id] = vt;
        } else if (tag.name == "vehicle" && !tag.closing) {
            insideVehicle = true;
            if (tag.selfClosing) insideVehicle = false;
        } else if (tag.name == "vehicle" && tag.closing) {
            insideVehicle = false;
        } else if (tag.name == "route" && !tag.closing && !insideVehicle) {
            string routeID = to_string_attr(tag.attrs, "id");
            string edges = to_string_attr(tag.attrs, "edges");
            if (!routeID.empty() && !edges.empty()) {
                sumoRouteIDToEdges[routeID] = split_ws(edges);
            }
        }
    }

    auto edge_list_text = [](const vector<string> &edges) {
        ostringstream oss;
        for (size_t i = 0; i < edges.size(); ++i) {
            if (i) oss << ' ';
            oss << edges[i];
        }
        return oss.str();
    };
    auto road_list_text = [](const vector<int> &roadsInRoute) {
        ostringstream oss;
        for (size_t i = 0; i < roadsInRoute.size(); ++i) {
            if (i) oss << ' ';
            oss << roadsInRoute[i];
        }
        return oss.str();
    };

    int vehiclesScanned = 0;
    int validVehicles = 0;
    int skippedVehicles = 0;
    int explicitSpeedFactorVehicles = 0;
    int previewPrinted = 0;
    bool haveDepart = false;
    double firstDepart = 0.0;
    double lastDepart = 0.0;

    auto process_vehicle = [&](const string &vehicleID,
                               const string &routeRef,
                               const vector<string> &inlineEdges,
                               double depart,
                               const string &vehicleTypeID,
                               double speedFactor,
                               bool hasExplicitSpeedFactor,
                               const string &departPos,
                               const string &departEdge) -> bool {
        if (maxVehicles > 0 && validVehicles >= maxVehicles) return false;

        const string resolvedTypeID = vehicleTypeID;
        if (sumoVehicleTypes.find(resolvedTypeID) == sumoVehicleTypes.end())
            throw runtime_error("SUMO vehicle=" + vehicleID + " references unknown vType=" + resolvedTypeID);

        vector<string> edgeIDs;
        if (!inlineEdges.empty()) {
            edgeIDs = inlineEdges;
        } else if (!routeRef.empty()) {
            auto routeIt = sumoRouteIDToEdges.find(routeRef);
            if (routeIt == sumoRouteIDToEdges.end()) {
                cout << "[SUMO Route Warning] vehicle=" << vehicleID
                     << " missing route=" << routeRef << endl;
                ++skippedVehicles;
                return true;
            }
            edgeIDs = routeIt->second;
        } else {
            cout << "[SUMO Route Warning] vehicle=" << vehicleID
                 << " has no route reference or inline route" << endl;
            ++skippedVehicles;
            return true;
        }

        vector<int> roadIDSequence;
        bool missingEdge = false;
        for (const string &edgeID : edgeIDs) {
            if (!edgeID.empty() && edgeID[0] == ':') {
                continue;
            }
            auto roadIt = sumoEdgeStrToRoadID.find(edgeID);
            if (roadIt == sumoEdgeStrToRoadID.end()) {
                cout << "[SUMO Route Warning] vehicle=" << vehicleID
                     << " missing edge=" << edgeID << endl;
                missingEdge = true;
                break;
            }
            roadIDSequence.push_back(roadIt->second);
        }
        if (missingEdge) {
            ++skippedVehicles;
            return true;
        }
        if (roadIDSequence.empty()) {
            cout << "[SUMO Route Warning] vehicle=" << vehicleID
                 << " has fewer than 1 road after filtering" << endl;
            ++skippedVehicles;
            return true;
        }

        int firstRoad = roadIDSequence.front();
        int lastRoad = roadIDSequence.back();
        if (firstRoad < 0 || firstRoad >= static_cast<int>(roads.size()) ||
            lastRoad < 0 || lastRoad >= static_cast<int>(roads.size())) {
            cout << "[SUMO Route Warning] vehicle=" << vehicleID
                 << " has roadID outside roads vector" << endl;
            ++skippedVehicles;
            return true;
        }

        // queryDataRaw retains the integer outer-window index for legacy callers.
        // The exact SUMO departure is stored separately and drives all events.
        int departWindow = static_cast<int>(floor(depart));
        int startNode = roads[firstRoad].fromNode;
        int endNode = roads[lastRoad].toNode;
        queryDataRaw.push_back({startNode, endNode, departWindow});
        routeRoadID.push_back(roadIDSequence);
        sumoVehicleIDs.push_back(vehicleID);
        vehicleDepartureTimes.push_back(depart);
        vehicleTypeIDs.push_back(resolvedTypeID);
        // Current-data contract: default/base, first external edge. SUMO base
        // places the front at vehicle length + POSITION_EPS (0.1 m), not minGap.
        // Do not guess insertion-dependent positions (free/random/last/etc.).
        double departurePosition = std::numeric_limits<double>::quiet_NaN();
        string departureIssue;
        if (departEdge != "0") {
            departureIssue = "unsupported departEdge=" + departEdge;
        } else if (departPos != "base") {
            departureIssue = "unsupported departPos=" + departPos + " (only base/default supported)";
        } else {
            const double vehicleLength = sumoVehicleTypes[resolvedTypeID].length;
            if (!std::isfinite(vehicleLength) || vehicleLength <= 0.0) {
                departureIssue = "invalid vehicle length for departPos=base";
            } else {
                departurePosition = vehicleLength + 0.1;
                if (!std::isfinite(roads[firstRoad].length) ||
                    departurePosition >= roads[firstRoad].length) {
                    departureIssue = "departPos=base reaches/exceeds first-road length; short-road insertion unsupported";
                }
            }
        }
        vehicleDeparturePositions.push_back(departurePosition);
        vehicleDeparturePositionIssues.push_back(departureIssue);
        vehicleSpeedFactors.push_back(speedFactor);
        vehicleHasExplicitSpeedFactor.push_back(hasExplicitSpeedFactor);
        if (hasExplicitSpeedFactor) ++explicitSpeedFactorVehicles;
        ++validVehicles;
        if (!haveDepart) {
            firstDepart = depart;
            haveDepart = true;
        }
        lastDepart = depart;

        if (previewPrinted < 5) {
            cout << "[SUMO Route Preview] vehicle=" << vehicleID
                 << " depart=" << depart;
            if (!routeRef.empty()) cout << " route=" << routeRef;
            else cout << " route=<inline>";
            cout << endl;
            cout << "  edges: " << edge_list_text(edgeIDs) << endl;
            cout << "  roadIDs: " << road_list_text(roadIDSequence) << endl;
            ++previewPrinted;
        }
        return true;
    };

    bool inVehicle = false;
    string currentVehicleID;
    string currentRouteRef;
    string currentVehicleTypeID;
    double currentSpeedFactor = std::numeric_limits<double>::quiet_NaN();
    bool currentHasExplicitSpeedFactor = false;
    double currentDepart = 0.0;
    string currentDepartPos = "base";
    string currentDepartEdge = "0";
    vector<string> currentInlineEdges;

    for (const auto &tag : tags) {
        if (maxVehicles > 0 && validVehicles >= maxVehicles) break;

        if (tag.name == "vehicle" && !tag.closing) {
            ++vehiclesScanned;
            currentVehicleID = to_string_attr(tag.attrs, "id", "vehicle_" + to_string(vehiclesScanned - 1));
            currentRouteRef = to_string_attr(tag.attrs, "route");
            currentVehicleTypeID = to_string_attr(tag.attrs, "type", "DEFAULT_VEHTYPE");
            auto speedFactor = tag.attrs.find("speedFactor");
            currentHasExplicitSpeedFactor = speedFactor != tag.attrs.end();
            currentSpeedFactor = currentHasExplicitSpeedFactor
                ? strict_positive_xml_number(speedFactor->second,
                    routeXmlPath + ": vehicle=" + currentVehicleID + " speedFactor")
                : std::numeric_limits<double>::quiet_NaN();
            currentDepart = to_double_attr(tag.attrs, "depart", 0.0);
            currentDepartPos = to_string_attr(tag.attrs, "departPos", "base");
            currentDepartEdge = to_string_attr(tag.attrs, "departEdge", "0");
            if (usesCapContextTable()) {
                const string arrivalPosition = to_string_attr(tag.attrs, "arrivalPos", "max");
                if ((!arrivalPosition.empty() && arrivalPosition != "max") ||
                    tag.attrs.find("arrivalEdge") != tag.attrs.end()) {
                    throw runtime_error("F13 base-first-v1 requires default/max arrivalPos and no arrivalEdge: vehicle="
                                        + currentVehicleID);
                }
            }
            currentInlineEdges.clear();

            if (tag.selfClosing) {
                process_vehicle(currentVehicleID, currentRouteRef, currentInlineEdges, currentDepart,
                                currentVehicleTypeID, currentSpeedFactor, currentHasExplicitSpeedFactor,
                                currentDepartPos, currentDepartEdge);
                inVehicle = false;
            } else {
                inVehicle = true;
            }
        } else if (tag.name == "route" && !tag.closing && inVehicle) {
            string edges = to_string_attr(tag.attrs, "edges");
            if (!edges.empty()) currentInlineEdges = split_ws(edges);
        } else if (tag.name == "vehicle" && tag.closing && inVehicle) {
            process_vehicle(currentVehicleID, currentRouteRef, currentInlineEdges, currentDepart,
                            currentVehicleTypeID, currentSpeedFactor, currentHasExplicitSpeedFactor,
                            currentDepartPos, currentDepartEdge);
            inVehicle = false;
            currentVehicleID.clear();
            currentRouteRef.clear();
            currentVehicleTypeID.clear();
            currentInlineEdges.clear();
        }
    }

    cout << "[SUMO Route] vehicle types: " << sumoVehicleTypes.size() << endl;
    cout << "[SUMO Route] route definitions: " << sumoRouteIDToEdges.size() << endl;
    cout << "[SUMO Route] vehicles scanned: " << vehiclesScanned << endl;
    cout << "[SUMO Route] valid vehicles: " << validVehicles << endl;
    cout << "[SUMO Route] skipped vehicles: " << skippedVehicles << endl;
    const size_t unresolvedDeparturePositions = count_if(
            vehicleDeparturePositionIssues.begin(), vehicleDeparturePositionIssues.end(),
            [](const string& issue) { return !issue.empty(); });
    cout << "[SUMO Route] resolved base departure positions: "
         << validVehicles - unresolvedDeparturePositions << '/' << validVehicles << endl;
    // Fail before simulation for the selected formula mode. Table labels already
    // use the real SUMO boundary; a successful table lookup is never corrected.
    if (((speedNetDepartPosition && travelTimeMode == TravelTimeMode::SPEED_NET) ||
         (usesCapContextTable() && capContextUsesDeparturePositionCorrection())) &&
        unresolvedDeparturePositions > 0) {
        for (size_t i = 0; i < vehicleDeparturePositionIssues.size(); ++i) {
            if (!vehicleDeparturePositionIssues[i].empty()) {
                throw runtime_error("Departure position for vehicle=" + sumoVehicleIDs[i] + ": " +
                    vehicleDeparturePositionIssues[i] +
                    (usesCapContextTable() ? "; F13 requires resolved base-first-v1 geometry"
                                          : "; --speed-net-depart-position off restores legacy full-road behavior"));
            }
        }
    }
    cout << "[SUMO Route] explicit vehicle speed factors: " << explicitSpeedFactorVehicles
         << '/' << validVehicles
         << (speedNetUseSpeedFactor ? " (speed-net SF cap enabled)"
             : usesCapContextTable() ? " (raw SF derives cap for cap/context table)"
             : travelTimeTableFormat == TravelTimeTableFormat::CAMS_SF_V1
                 ? " (SF used directly by five-feature table)" : " (SF cap disabled; metadata only)") << endl;
    if (speedNetUseSpeedFactor && explicitSpeedFactorVehicles < validVehicles) {
        throw runtime_error("--speed-net-speed-factor on requires explicit speedFactor for every loaded vehicle; "
                            + to_string(validVehicles - explicitSpeedFactorVehicles)
                            + " missing in " + routeXmlPath);
    }
    if (explicitSpeedFactorVehicles < validVehicles) {
        if (travelTimeMode == TravelTimeMode::TABLE
            && (travelTimeTableFormat == TravelTimeTableFormat::CAMS_SF_V1 || usesCapContextTable())) {
            throw runtime_error(string(usesCapContextTable()
                ? (usesNativeRoadTimeTable() ? "CAMS_NATIVE_ABC_RG_DEMAND_V2"
                   : usesRGCapContextTable() ? "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1"
                   : usesABCCapContextTable() ? "CAMS_CAP_CONTEXT_ABC_RLE_V1" : "CAMS_CAP_CONTEXT_RLE_V1")
                : "CAMS_SF_TABLE_V1")
                                + " requires explicit speedFactor for every loaded vehicle; "
                                + to_string(validVehicles - explicitSpeedFactorVehicles)
                                + " missing in " + routeXmlPath);
        }
        cout << "[SUMO Route Warning] " << validVehicles - explicitSpeedFactorVehicles
             << " loaded vehicles lack explicit speedFactor; per-vehicle SF/free-speed cap"
             << " remain unknown. Legacy prediction behavior is unchanged;"
             << " vType distribution means are not substituted." << endl;
    }
    if (haveDepart) {
        cout << "[SUMO Route] first depart: " << firstDepart << endl;
        cout << "[SUMO Route] last depart: " << lastDepart << endl;
    } else {
        cout << "[SUMO Route] first depart: n/a" << endl;
        cout << "[SUMO Route] last depart: n/a" << endl;
    }
}


void Graph::read_sumo_tripinfo_xml(const string& tripinfoPath)
{
    ifstream in(tripinfoPath.c_str());
    require_open(in, tripinfoPath, "read_sumo_tripinfo_xml SUMO tripinfo XML");
    stringstream buffer;
    buffer << in.rdbuf();
    vector<XmlTagLite> tags = parse_xml_tags_lite(buffer.str());

    sumoTruthByVehicleID.clear();
    sumoTruthAligned.clear();

    int duplicateCount = 0;
    int missingIDCount = 0;
    int missingDurationCount = 0;
    string firstVehicleID;
    string lastVehicleID;

    for (const auto &tag : tags) {
        if (tag.name != "tripinfo" || tag.closing) continue;

        string id = to_string_attr(tag.attrs, "id");
        if (id.empty()) {
            ++missingIDCount;
            cout << "[SUMO Truth Warning] tripinfo record missing required id; skipping." << endl;
            continue;
        }
        if (tag.attrs.find("duration") == tag.attrs.end()) {
            ++missingDurationCount;
            cout << "[SUMO Truth Warning] vehicle=" << id
                 << " missing required duration; skipping." << endl;
            continue;
        }

        SumoTripInfoTruth truth;
        truth.vehicleID = id;
        truth.depart = to_double_attr(tag.attrs, "depart", 0.0);
        truth.arrival = to_double_attr(tag.attrs, "arrival", 0.0);
        truth.duration = to_double_attr(tag.attrs, "duration", 0.0);
        truth.routeLength = to_double_attr(tag.attrs, "routeLength", 0.0);
        truth.waitingTime = to_double_attr(tag.attrs, "waitingTime", 0.0);
        truth.timeLoss = to_double_attr(tag.attrs, "timeLoss", 0.0);
        truth.departDelay = to_double_attr(tag.attrs, "departDelay", 0.0);

        if (sumoTruthByVehicleID.find(id) != sumoTruthByVehicleID.end()) {
            ++duplicateCount;
            cout << "[SUMO Truth Warning] duplicate tripinfo id=" << id
                 << "; keeping latest record." << endl;
        }
        if (firstVehicleID.empty()) firstVehicleID = id;
        lastVehicleID = id;
        sumoTruthByVehicleID[id] = truth;

    }

    sumoTruthAligned.reserve(sumoVehicleIDs.size());
    for (const string &vehicleID : sumoVehicleIDs) {
        auto it = sumoTruthByVehicleID.find(vehicleID);
        if (it != sumoTruthByVehicleID.end()) {
            sumoTruthAligned.push_back(it->second);
        }
    }

    cout << "[SUMO Truth] tripinfo records: " << sumoTruthByVehicleID.size() << endl;
    if (!sumoTruthByVehicleID.empty()) {
        double minDuration = 0.0;
        double maxDuration = 0.0;
        double totalDuration = 0.0;
        bool haveDuration = false;
        for (const auto &kv : sumoTruthByVehicleID) {
            const double duration = kv.second.duration;
            if (!haveDuration) {
                minDuration = duration;
                maxDuration = duration;
                haveDuration = true;
            } else {
                minDuration = min(minDuration, duration);
                maxDuration = max(maxDuration, duration);
            }
            totalDuration += duration;
        }
        cout << "[SUMO Truth] first vehicle: " << firstVehicleID << endl;
        cout << "[SUMO Truth] last vehicle: " << lastVehicleID << endl;
        double avgDuration = totalDuration / static_cast<double>(sumoTruthByVehicleID.size());
        cout << "[SUMO Truth] duration min/avg/max: "
             << minDuration << " / " << avgDuration << " / " << maxDuration << endl;
    } else {
        cout << "[SUMO Truth] first vehicle: n/a" << endl;
        cout << "[SUMO Truth] last vehicle: n/a" << endl;
        cout << "[SUMO Truth] duration min/avg/max: n/a" << endl;
    }
    if (duplicateCount > 0) cout << "[SUMO Truth] duplicate ids: " << duplicateCount << endl;
    if (missingIDCount > 0) cout << "[SUMO Truth] missing id records skipped: " << missingIDCount << endl;
    if (missingDurationCount > 0) cout << "[SUMO Truth] missing duration records skipped: " << missingDurationCount << endl;
    cout << "[SUMO Truth] aligned simulated vehicles with truth: " << sumoTruthAligned.size() << endl;
}

// SUMO parser / static data preparation only.
// Reads .net.xml roads, lanes, connections, junctions, and signal programs into
// raw/intermediate graph fields consumed later by the runtime simulator.
void Graph::read_sumo_net_xml(const string& netXmlPath)
{
    sumoNetworkWithoutInternalLinks = false;
    ifstream in(netXmlPath.c_str());
    require_open(in, netXmlPath, "read_sumo_net_xml SUMO net XML");
    stringstream buffer;
    buffer << in.rdbuf();
    vector<XmlTagLite> tags = parse_xml_tags_lite(buffer.str());

    if (tags.empty() || tags.front().name != "net" || tags.front().closing ||
        tags.back().name != "net" || !tags.back().closing)
        throw runtime_error("SUMO network must have a complete net root: " + netXmlPath);

    sumoNodeStrToID.clear();
    nodeIDToSumoNodeStr.clear();
    sumoEdgeStrToRoadID.clear();
    roadIDToSumoEdgeStr.clear();
    tlIDToSignalProgramID.clear();
    sumoEdgesRaw.clear();
    sumoJunctionsRaw.clear();
    sumoConnectionsRaw.clear();
    sumoSignalPrograms.clear();
    outgoingSignalRGCache.clear();
    unsignalizedGreenHorizonCache = numeric_limits<double>::quiet_NaN();
    sumoHasSignalProgramSwitching = false;
    signalPrograms.clear();
    nodeID2RoadID.clear();
    roadID2NodeID.clear();
    nodeID2minTime.clear();
    roadInfor.clear();
    roads.clear();
    nodes.clear();
    graphLength.clear();
    graphRoadID.clear();
    adjNodes.clear();

    SumoEdgeRaw *currentEdge = nullptr;
    SumoSignalProgram *currentTL = nullptr;
    int totalEdges = 0;
    int skippedInternalEdges = 0;

    for (const auto &tag : tags) {
        if (tag.name == "edge" && !tag.closing) {
            ++totalEdges;
            SumoEdgeRaw edge;
            edge.id = to_string_attr(tag.attrs, "id");
            edge.from = to_string_attr(tag.attrs, "from");
            edge.to = to_string_attr(tag.attrs, "to");
            edge.function = to_string_attr(tag.attrs, "function");
            edge.priority = to_int_attr(tag.attrs, "priority", 0);
            edge.isInternal = is_sumo_internal_edge(edge.id, edge.function);
            sumoEdgesRaw.push_back(edge);
            currentEdge = &sumoEdgesRaw.back();
            if (edge.isInternal) ++skippedInternalEdges;
            if (tag.selfClosing) currentEdge = nullptr;
        } else if (tag.name == "edge" && tag.closing) {
            currentEdge = nullptr;
        } else if (tag.name == "lane" && !tag.closing && currentEdge != nullptr) {
            SumoLaneRaw lane;
            lane.id = to_string_attr(tag.attrs, "id");
            lane.index = to_int_attr(tag.attrs, "index", static_cast<int>(currentEdge->lanes.size()));
            lane.speed = to_double_attr(tag.attrs, "speed", 0.0);
            lane.length = to_double_attr(tag.attrs, "length", 0.0);
            lane.shape = to_string_attr(tag.attrs, "shape");
            lane.allow = to_string_attr(tag.attrs, "allow", "all");
            lane.disallow = to_string_attr(tag.attrs, "disallow");
            lane.changeLeft = to_string_attr(tag.attrs, "changeLeft", "all");
            lane.changeRight = to_string_attr(tag.attrs, "changeRight", "all");
            currentEdge->lanes.push_back(lane);
        } else if (tag.name == "junction" && !tag.closing) {
            SumoJunctionRaw j;
            j.id = to_string_attr(tag.attrs, "id");
            j.type = to_string_attr(tag.attrs, "type");
            j.x = to_double_attr(tag.attrs, "x", 0.0);
            j.y = to_double_attr(tag.attrs, "y", 0.0);
            j.incLanes = split_ws(to_string_attr(tag.attrs, "incLanes"));
            j.intLanes = split_ws(to_string_attr(tag.attrs, "intLanes"));
            sumoJunctionsRaw.push_back(j);
        } else if (tag.name == "connection" && !tag.closing) {
            SumoConnectionRaw c;
            c.fromEdge = to_string_attr(tag.attrs, "from");
            c.toEdge = to_string_attr(tag.attrs, "to");
            c.via = to_string_attr(tag.attrs, "via");
            c.tl = to_string_attr(tag.attrs, "tl");
            c.allow = to_string_attr(tag.attrs, "allow", "all");
            c.disallow = to_string_attr(tag.attrs, "disallow");
            if (usesRGCapContextTable() || mergeRoadPairMovements) {
                const string context = "connection " + c.fromEdge + " -> " + c.toEdge;
                c.fromLane = strict_rg_xml_index(tag.attrs, "fromLane", context);
                c.toLane = strict_rg_xml_index(tag.attrs, "toLane", context);
                c.linkIndex = strict_rg_xml_index(tag.attrs, "linkIndex", context, c.tl.empty());
            } else {
                c.fromLane = to_int_attr(tag.attrs, "fromLane", -1);
                c.toLane = to_int_attr(tag.attrs, "toLane", -1);
                c.linkIndex = to_int_attr(tag.attrs, "linkIndex", -1);
            }
            string dir = to_string_attr(tag.attrs, "dir");
            c.dir = dir.empty() ? TurnDir::Unknown : parseTurnDir(dir[0]);
            string state = to_string_attr(tag.attrs, "state", "O");
            c.state = state.empty() ? 'O' : state[0];
            sumoConnectionsRaw.push_back(c);
        } else if (tag.name == "tlLogic" && !tag.closing) {
            SumoSignalProgram p;
            p.tlID = to_string_attr(tag.attrs, "id");
            p.type = to_string_attr(tag.attrs, "type");
            p.programID = to_string_attr(tag.attrs, "programID");
            p.offset = (usesRGCapContextTable() || mergeRoadPairMovements)
                ? strict_rg_xml_real(tag.attrs, "offset", "TLS=" + p.tlID, false, true)
                : to_double_attr(tag.attrs, "offset", 0.0);
            sumoSignalPrograms.push_back(p);
            currentTL = &sumoSignalPrograms.back();
            if (tag.selfClosing) currentTL = nullptr;
        } else if (tag.name == "tlLogic" && tag.closing) {
            currentTL = nullptr;
        } else if (tag.name == "phase" && !tag.closing && currentTL != nullptr) {
            SumoSignalPhase phase;
            phase.duration = (usesRGCapContextTable() || mergeRoadPairMovements)
                ? strict_rg_xml_real(tag.attrs, "duration", "TLS=" + currentTL->tlID + " phase", true)
                : to_double_attr(tag.attrs, "duration", 0.0);
            phase.startTime = currentTL->cycleLength;
            phase.endTime = phase.startTime + phase.duration;
            phase.state = to_string_attr(tag.attrs, "state");
            phase.nextPhaseIndices = to_string_attr(tag.attrs, "next");
            currentTL->cycleLength = phase.endTime;
            currentTL->phases.push_back(phase);
        } else if (!tag.closing && (tag.name == "WAUT" || tag.name == "wautJunction" || tag.name == "wautSwitch")) {
            sumoHasSignalProgramSwitching = true;
        }
    }

    if (usesRGCapContextTable() && sumoHasSignalProgramSwitching)
        throw runtime_error("R/G v1 requires a fixed static signal program; WAUT unsupported");

    // Network clock policy is structural, never inferred from a netconvert
    // comment or junction type. A dangling via/intLane is a broken input, not
    // evidence that the network omitted its internal links deliberately.
    set<string> internalLaneIDs;
    for (const auto& edge : sumoEdgesRaw)
        if (edge.isInternal)
            for (const auto& lane : edge.lanes) internalLaneIDs.insert(lane.id);
    bool hasVia = false;
    for (const auto& connection : sumoConnectionsRaw) {
        if (connection.via.empty()) continue;
        hasVia = true;
        if (internalLaneIDs.find(connection.via) == internalLaneIDs.end())
            throw runtime_error("SUMO connection references missing internal via lane: " + connection.via);
    }
    for (const auto& junction : sumoJunctionsRaw)
        for (const auto& lane : junction.intLanes)
            if (internalLaneIDs.find(lane) == internalLaneIDs.end())
                throw runtime_error("SUMO junction references missing internal lane: " + lane);

    int nodeID = 0;
    for (const auto &j : sumoJunctionsRaw) {
        if (j.id.empty()) continue;
        sumoNodeStrToID[j.id] = nodeID;
        nodeIDToSumoNodeStr[nodeID] = j.id;
        ++nodeID;
    }
    nodenum = nodeID;
    graphLength.assign(nodenum, vector<pair<int, double>>());
    graphRoadID.assign(nodenum, vector<pair<int, int>>());
    adjNodes.assign(nodenum, set<int>());
    nodes.resize(nodenum);
    for (int i = 0; i < nodenum; ++i) nodes[i].nodeID = i;

    int roadID = 0;
    for (const auto &edge : sumoEdgesRaw) {
        if (edge.isInternal) continue;
        if (edge.id.empty()) continue;
        if (usesCapContextTable()) {
            if (edge.lanes.empty()) throw runtime_error("F13 edge has no lanes: " + edge.id);
            double minLength = std::numeric_limits<double>::infinity(), maxLength = 0;
            double minSpeed = std::numeric_limits<double>::infinity(), maxSpeed = 0;
            for (const auto& lane : edge.lanes) {
                if (!std::isfinite(lane.length) || lane.length <= 0 ||
                    !std::isfinite(lane.speed) || lane.speed <= 0)
                    throw runtime_error("F13 requires positive finite lane length/speed: " + edge.id);
                minLength = min(minLength, lane.length); maxLength = max(maxLength, lane.length);
                minSpeed = min(minSpeed, lane.speed); maxSpeed = max(maxSpeed, lane.speed);
            }
            if (maxLength - minLength > 1e-9 || maxSpeed - minSpeed > 1e-9)
                throw runtime_error("F13 v1 requires uniform lane length/speed within edge: " + edge.id);
        }
        auto fromIt = sumoNodeStrToID.find(edge.from);
        auto toIt = sumoNodeStrToID.find(edge.to);
        if (fromIt == sumoNodeStrToID.end() || toIt == sumoNodeStrToID.end()) {
            cout << "[SUMO Warning] edge has missing junction endpoint and is skipped as normal road: "
                 << edge.id << " from=" << edge.from << " to=" << edge.to << endl;
            continue;
        }
        sumoEdgeStrToRoadID[edge.id] = roadID;
        roadIDToSumoEdgeStr[roadID] = edge.id;

        double lengthSum = 0.0, speedSum = 0.0;
        int validLen = 0, validSpeed = 0;
        for (const auto &lane : edge.lanes) {
            if (lane.length > 0) { lengthSum += lane.length; ++validLen; }
            if (lane.speed > 0) { speedSum += lane.speed; ++validSpeed; }
        }
        double length = validLen > 0 ? lengthSum / validLen : 0.0;
        double speed = validSpeed > 0 ? speedSum / validSpeed : 1.0;
        int fromNode = fromIt->second;
        int toNode = toIt->second;

        Road legacy;
        legacy.roadID = roadID;
        legacy.ID1 = fromNode;
        legacy.ID2 = toNode;
        legacy.length = length;
        legacy.travelTime = max(1.0, length / max(0.1, speed));
        legacy.direction = 0;
        legacy.speedLimit = speed;
        legacy.laneNum = max(1, static_cast<int>(edge.lanes.size()));
        legacy.width = 0;
        legacy.kindNumber = edge.priority;
        legacy.kind = edge.function.empty() ? "sumo" : edge.function;
        roadInfor.push_back(legacy);

        RoadSegment segment;
        segment.roadID = roadID;
        segment.fromNode = fromNode;
        segment.toNode = toNode;
        segment.length = length;
        segment.speedLimit = speed;
        segment.laneNum = legacy.laneNum;
        segment.kindNumber = edge.priority;
        segment.kind = legacy.kind;
        segment.minTravelTime = max(1.0, length / max(0.1, speed));
        roads.push_back(segment);
        roadIDToRoadIndex[roadID] = roadID;

        nodeID2RoadID[make_pair(fromNode, toNode)] = roadID;
        roadID2NodeID[roadID] = make_pair(fromNode, toNode);
        nodeID2minTime[make_pair(fromNode, toNode)] = legacy.travelTime;
        graphLength[fromNode].push_back(make_pair(toNode, length));
        graphRoadID[fromNode].push_back(make_pair(toNode, roadID));
        adjNodes[fromNode].insert(toNode);
        nodes[fromNode].outgoingRoads.push_back(roadID);
        nodes[toNode].incomingRoads.push_back(roadID);
        outgoingRoadsByNode[fromNode].push_back(roadID);
        incomingRoadsByNode[toNode].push_back(roadID);
        ++roadID;
    }
    edgenum = roadID;
    if (edgenum == 0) throw runtime_error("SUMO network has no usable external roads: " + netXmlPath);
    sumoNetworkWithoutInternalLinks = skippedInternalEdges == 0 && !hasVia;

    cout << "[SUMO] parsed edge tags: " << totalEdges << endl;
    cout << "[SUMO] network without internal links: " << sumoNetworkWithoutInternalLinks << endl;
    cout << "[SUMO] parsed normal roads: " << edgenum << endl;
    cout << "[SUMO] skipped internal edges: " << skippedInternalEdges << endl;
    cout << "[SUMO] junctions: " << sumoJunctionsRaw.size() << endl;
    cout << "[SUMO] connections: " << sumoConnectionsRaw.size() << endl;
    cout << "[SUMO] tlLogics: " << sumoSignalPrograms.size() << endl;
    size_t phaseCount = 0;
    for (const auto &p : sumoSignalPrograms) phaseCount += p.phases.size();
    cout << "[SUMO] phases: " << phaseCount << endl;
}

void Graph::classify_node_types_from_sumo_junctions()
{
    for (auto &node : nodes) {
        node.type = NodeType::NormalSplit;
        node.sumoJunctionType.clear();
    }
    for (const auto &j : sumoJunctionsRaw) {
        auto it = sumoNodeStrToID.find(j.id);
        if (it == sumoNodeStrToID.end()) continue;
        int nodeID = it->second;
        if (nodeID < 0 || nodeID >= static_cast<int>(nodes.size())) continue;
        nodes[nodeID].sumoJunctionType = j.type;
        if (j.type == "traffic_light") {
            nodes[nodeID].type = NodeType::SignalizedJunction;
        } else if (j.type == "priority" || j.type == "right_before_left") {
            nodes[nodeID].type = NodeType::UnsignalizedJunction;
        } else {
            nodes[nodeID].type = NodeType::NormalSplit;
        }
    }
}

namespace {
// A canonical, absolute-time cyclic calendar. Adjacent equal states are merged;
// G/g priority is deliberately absent here but remains in the raw connection and
// phase metadata. Equal calendars may come from different static controllers.
struct RoadPairCalendar {
    double cycle = 0.0;
    vector<pair<double, char>> changes;
};

char road_pair_signal_state(char state) {
    if (state == 'G' || state == 'g' || state == 'O') return 'G';
    if (state == 'y' || state == 'Y') return 'y';
    if (state == 'r' || state == 'R') return 'r';
    throw runtime_error("road-pair grouping: unsupported signal state '" + string(1, state) + "'");
}

RoadPairCalendar road_pair_calendar(const SumoConnectionRaw& connection,
                                   const SumoSignalProgram* program) {
    RoadPairCalendar calendar;
    if (connection.tl.empty()) {
        calendar.changes.push_back({0.0, 'G'});
        return calendar;
    }
    if (!program || program->type != "static" || program->phases.empty() ||
        !std::isfinite(program->offset))
        throw runtime_error("road-pair grouping requires one finite sequential static program for TLS " + connection.tl);
    vector<pair<double, char>> local;
    for (size_t p = 0; p < program->phases.size(); ++p) {
        const auto& phase = program->phases[p];
        if (!std::isfinite(phase.duration) || phase.duration <= 0.0)
            throw runtime_error("road-pair grouping: nonpositive/nonfinite phase duration for TLS " + connection.tl);
        if (!phase.nextPhaseIndices.empty()) {
            istringstream input(phase.nextPhaseIndices);
            string token, extra;
            if (!(input >> token) || (input >> extra))
                throw runtime_error("road-pair grouping: branching static phase sequence unsupported");
            size_t consumed = 0;
            const int next = stoi(token, &consumed);
            if (consumed != token.size() || next != static_cast<int>((p + 1) % program->phases.size()))
                throw runtime_error("road-pair grouping requires sequential static phases");
        }
        if (connection.linkIndex < 0 || connection.linkIndex >= static_cast<int>(phase.state.size()))
            throw runtime_error("road-pair grouping: linkIndex outside phase state for TLS " + connection.tl);
        local.push_back({calendar.cycle, road_pair_signal_state(phase.state[connection.linkIndex])});
        calendar.cycle += phase.duration;
    }
    if (!std::isfinite(calendar.cycle) || calendar.cycle <= 0.0)
        throw runtime_error("road-pair grouping: invalid cycle for TLS " + connection.tl);
    for (size_t p = 0; p < local.size(); ++p) {
        if (local[p].second == local[(p + local.size() - 1) % local.size()].second) continue;
        double time = fmod(local[p].first + fmod(program->offset, calendar.cycle), calendar.cycle);
        if (time < 0.0) time += calendar.cycle;
        if (abs(time) < 1e-9 || abs(time - calendar.cycle) < 1e-9) time = 0.0;
        calendar.changes.push_back({time, local[p].second});
    }
    if (calendar.changes.empty()) {
        calendar.cycle = 0.0; // Constant calendars do not depend on period/offset.
        calendar.changes.push_back({0.0, local.front().second});
    } else {
        sort(calendar.changes.begin(), calendar.changes.end());
    }
    return calendar;
}

bool same_road_pair_calendar(const RoadPairCalendar& a, const RoadPairCalendar& b) {
    if (abs(a.cycle - b.cycle) > 1e-9 || a.changes.size() != b.changes.size()) return false;
    for (size_t i = 0; i < a.changes.size(); ++i)
        if (abs(a.changes[i].first - b.changes[i].first) > 1e-9 ||
            a.changes[i].second != b.changes[i].second) return false;
    return true;
}
}

// Static graph construction: SUMO <connection> entries become Movement objects.
// These movements are the runtime dispatch units, but this function itself is not
// part of the event simulation.
void Graph::build_movements_from_sumo_connections()
{
    movements.clear();
    roadPairToMovementID.clear();
    fromToRoadToMovementIDs.clear();
    movementKeyToID.clear();
    outgoingMovementsByRoad.clear();
    incomingMovementsByRoad.clear();
    movementIDsByIntersection.clear();
    for (auto &inter : intersections) inter.movementIDs.clear();

    map<string, vector<const SumoSignalProgram*>> programsByTLS;
    map<pair<string, int>, RoadPairCalendar> calendars;
    vector<RoadPairCalendar> movementCalendars;
    if (mergeRoadPairMovements) {
        if (sumoHasSignalProgramSwitching)
            throw runtime_error("road-pair grouping requires a fixed signal program; WAUT unsupported");
        for (const auto& program : sumoSignalPrograms) programsByTLS[program.tlID].push_back(&program);
    }
    for (size_t connectionIndex = 0; connectionIndex < sumoConnectionsRaw.size(); ++connectionIndex) {
        const auto& c = sumoConnectionsRaw[connectionIndex];
        auto fromIt = sumoEdgeStrToRoadID.find(c.fromEdge);
        auto toIt = sumoEdgeStrToRoadID.find(c.toEdge);
        if (fromIt == sumoEdgeStrToRoadID.end() || toIt == sumoEdgeStrToRoadID.end()) {
            continue;
        }
        int fromRoadID = fromIt->second;
        int toRoadID = toIt->second;
        const auto roadPair = make_pair(fromRoadID, toRoadID);
        string key = to_string(fromRoadID) + "|" + to_string(toRoadID) + "|" + c.tl + "|" + to_string(c.linkIndex);
        const RoadPairCalendar* calendar = nullptr;
        if (mergeRoadPairMovements) {
            if (fromRoadID < 0 || fromRoadID >= static_cast<int>(roads.size()) ||
                toRoadID < 0 || toRoadID >= static_cast<int>(roads.size()) ||
                c.fromLane < 0 || c.fromLane >= roads[fromRoadID].laneNum ||
                c.toLane < 0 || c.toLane >= roads[toRoadID].laneNum)
                throw runtime_error("road-pair grouping: invalid original lane pair for " + c.fromEdge + " -> " + c.toEdge);
            const auto signalKey = make_pair(c.tl, c.linkIndex);
            auto cached = calendars.find(signalKey);
            if (cached == calendars.end()) {
                const SumoSignalProgram* program = nullptr;
                if (!c.tl.empty()) {
                    const auto found = programsByTLS.find(c.tl);
                    if (found == programsByTLS.end() || found->second.size() != 1)
                        throw runtime_error("road-pair grouping requires exactly one static program for TLS " + c.tl);
                    program = found->second.front();
                }
                cached = calendars.emplace(signalKey, road_pair_calendar(c, program)).first;
            }
            calendar = &cached->second;
        }
        int movementID = -1;
        if (mergeRoadPairMovements) {
            const auto existing = roadPairToMovementID.find(roadPair);
            if (existing != roadPairToMovementID.end()) movementID = existing->second;
        } else {
            const auto existing = movementKeyToID.find(key);
            if (existing != movementKeyToID.end()) movementID = existing->second;
        }
        if (movementID >= 0) {
            if (mergeRoadPairMovements && !same_road_pair_calendar(movementCalendars[movementID], *calendar))
                throw runtime_error("road-pair grouping: incompatible complete signal calendars for " +
                    c.fromEdge + " -> " + c.toEdge + " (representative " + movements[movementID].tlID +
                    "/" + to_string(movements[movementID].linkIndex) + ", connection " + c.tl + "/" +
                    to_string(c.linkIndex) + "); connection-level differing calendars are unsupported");
        } else {
            movementID = static_cast<int>(movements.size());
            Movement m;
            m.movementID = movementID;
            m.fromRoadID = fromRoadID;
            m.toRoadID = toRoadID;
            m.turn = c.dir;
            m.tlID = c.tl;
            m.linkIndex = c.linkIndex;
            m.defaultConnectionState = c.state;
            m.alwaysOpen = c.tl.empty();
            int downstreamNode = roads[fromRoadID].toNode;
            if (downstreamNode >= 0 && downstreamNode < static_cast<int>(nodes.size())) {
                m.intersectionID = nodes[downstreamNode].intersectionID;
            }
            movements.push_back(m);
            if (mergeRoadPairMovements) movementCalendars.push_back(*calendar);
            fromToRoadToMovementIDs[roadPair].push_back(movementID);
            if (roadPairToMovementID.find(roadPair) == roadPairToMovementID.end()) {
                roadPairToMovementID[roadPair] = movementID;
            }
            outgoingMovementsByRoad[fromRoadID].push_back(movementID);
            incomingMovementsByRoad[toRoadID].push_back(movementID);
            if (m.intersectionID >= 0 && m.intersectionID < static_cast<int>(intersections.size())) {
                intersections[m.intersectionID].movementIDs.push_back(movementID);
                movementIDsByIntersection[m.intersectionID].push_back(movementID);
            }
        }
        // Preserve every former link-key as a trace alias to its merged movement.
        movementKeyToID[key] = movementID;
        movements[movementID].connectionIndices.push_back(static_cast<int>(connectionIndex));
        auto add_unique_lane = [](vector<string> &v, int x) {
            if (x < 0) return;
            string lane = to_string(x);
            if (find(v.begin(), v.end(), lane) == v.end()) v.push_back(lane);
        };
        add_unique_lane(movements[movementID].fromLanes, c.fromLane);
        add_unique_lane(movements[movementID].toLanes, c.toLane);
    }
    cout << "[SUMO] movements: " << movements.size() << endl;
    build_continuous_speed_path_limits();
}

void Graph::build_continuous_speed_path_limits() {
    // One-time structural preprocessing. Follow the WHOLE internal path, not
    // only the first via lane. Branching continuations are deliberately unknown;
    // distinct complete external lane connections in one movement use the min.
    struct LaneRef { const SumoEdgeRaw* edge; const SumoLaneRaw* lane; };
    unordered_map<string, LaneRef> lanes;
    set<string> duplicateLanes;
    for (const auto& edge : sumoEdgesRaw) {
        if (!edge.isInternal) continue;
        for (const auto& lane : edge.lanes) {
            if (!lanes.emplace(lane.id, LaneRef{&edge, &lane}).second) duplicateLanes.insert(lane.id);
        }
    }
    map<pair<string, int>, vector<const SumoConnectionRaw*>> outgoing;
    for (const auto& connection : sumoConnectionsRaw)
        outgoing[{connection.fromEdge, connection.fromLane}].push_back(&connection);
    vector<bool> seen(movements.size(), false), bad(movements.size(), false);
    for (auto& movement : movements) {
        movement.continuousSpeedPathKnown = sumoNetworkWithoutInternalLinks;
        movement.continuousSpeedPathLimit = sumoNetworkWithoutInternalLinks
            ? numeric_limits<double>::infinity() : numeric_limits<double>::quiet_NaN();
        movement.connectionContinuousSpeedPathKnown.assign(movement.connectionIndices.size(), sumoNetworkWithoutInternalLinks);
        movement.connectionContinuousSpeedPathLimit.assign(movement.connectionIndices.size(), movement.continuousSpeedPathLimit);
    }
    if (sumoNetworkWithoutInternalLinks) return;
    for (auto& movement : movements) {
        const int movementID = movement.movementID;
        for (size_t option = 0; option < movement.connectionIndices.size(); ++option) {
            const auto& connection = sumoConnectionsRaw.at(movement.connectionIndices[option]);
            string laneID = connection.via;
            set<string> visited;
            double limit = numeric_limits<double>::infinity();
            bool complete = false;
            while (!laneID.empty()) {
                const auto lane = lanes.find(laneID);
                if (lane == lanes.end() || duplicateLanes.count(laneID) || !visited.insert(laneID).second) break;
                const auto& current = lane->second;
                if (!std::isfinite(current.lane->speed) || current.lane->speed <= 0.0) break;
                limit = min(limit, current.lane->speed);
                const auto next = outgoing.find({current.edge->id, current.lane->index});
                if (next == outgoing.end() || next->second.size() != 1) break;
                const auto& continuation = *next->second.front();
                if (continuation.toEdge != connection.toEdge || continuation.toLane != connection.toLane) break;
                if (continuation.via.empty()) { complete = true; break; }
                laneID = continuation.via;
            }
            seen[movementID] = true;
            if (!complete || !std::isfinite(limit) || limit <= 0.0) {
                bad[movementID] = true;
            } else {
                movement.connectionContinuousSpeedPathKnown[option] = true;
                movement.connectionContinuousSpeedPathLimit[option] = limit;
                movement.continuousSpeedPathLimit = std::isfinite(movement.continuousSpeedPathLimit)
                    ? min(movement.continuousSpeedPathLimit, limit) : limit;
            }
        }
    }
    for (size_t i = 0; i < movements.size(); ++i)
        movements[i].continuousSpeedPathKnown = seen[i] && !bad[i];
}

// Static graph construction: group SUMO connection lanes by movement so the
// simulator can map movement-level queues onto lane-level storage.
void Graph::build_lane_groups_from_sumo_connections()
{
    laneGroups.clear();
    laneGroupsByRoad.clear();
    roadTurnToLaneGroupID.clear();
    for (auto &m : movements) m.laneGroupID = -1;

    map<int, int> movementToLaneGroup;
    for (auto &m : movements) {
        LaneGroup group;
        group.laneGroupID = static_cast<int>(laneGroups.size());
        group.roadID = m.fromRoadID;
        group.turn = m.turn;
        group.laneIndices.clear();
        for (const string &lane : m.fromLanes) {
            try {
                size_t pos = 0;
                int laneIndex = stoi(lane, &pos);
                if (pos == lane.size()) group.laneIndices.push_back(laneIndex);
            } catch (const exception&) {
                // Ignore non-numeric lane identifiers for lane-group indexing;
                // discharge capacity uses Movement::fromLanes directly.
            }
        }
        sort(group.laneIndices.begin(), group.laneIndices.end());
        group.laneIndices.erase(unique(group.laneIndices.begin(), group.laneIndices.end()), group.laneIndices.end());
        if (group.laneIndices.empty()) group.laneIndices.push_back(0);
        group.allowedToRoads.push_back(m.toRoadID);
        laneGroups.push_back(group);
        m.laneGroupID = group.laneGroupID;
        laneGroupsByRoad[m.fromRoadID].push_back(group.laneGroupID);
        if (roadTurnToLaneGroupID.find(make_pair(m.fromRoadID, m.turn)) == roadTurnToLaneGroupID.end()) {
            roadTurnToLaneGroupID[make_pair(m.fromRoadID, m.turn)] = group.laneGroupID;
        }
    }
    cout << "[SUMO] lane groups: " << laneGroups.size() << endl;
}

void Graph::build_signal_programs_from_sumo_tllogic()
{
    signalPrograms.clear();
    tlIDToSignalProgramID.clear();
    for (const auto &raw : sumoSignalPrograms) {
        SignalProgram p;
        p.tlID = raw.tlID;
        p.offset = raw.offset;
        p.cycleLength = 0;
        for (const auto &rawPhase : raw.phases) {
            SignalPhase phase;
            phase.startTime = p.cycleLength;
            phase.endTime = p.cycleLength + rawPhase.duration;
            phase.state = rawPhase.state;
            p.cycleLength = phase.endTime;
            p.phases.push_back(phase);
        }
        int id = static_cast<int>(signalPrograms.size());
        signalPrograms.push_back(p);
        tlIDToSignalProgramID[p.tlID] = id;
    }

    signals.clear();
    signalIDsByIntersection.clear();
    for (auto &inter : intersections) inter.signalIDs.clear();
    for (auto &m : movements) {
        SignalController s;
        s.signalID = static_cast<int>(signals.size());
        s.intersectionID = m.intersectionID;
        s.movementID = m.movementID;
        s.alwaysOpen = m.alwaysOpen || m.tlID.empty();
        auto programIt = tlIDToSignalProgramID.find(m.tlID);
        if (programIt != tlIDToSignalProgramID.end()) {
            const SignalProgram &p = signalPrograms[programIt->second];
            s.cycleLength = p.cycleLength;
            s.offset = p.offset;
        }
        s.currentState = s.alwaysOpen ? SignalState::AlwaysOpen : SignalState::Red;
        signals.push_back(s);
        m.signalID = s.signalID;
        if (s.intersectionID >= 0 && s.intersectionID < static_cast<int>(intersections.size())) {
            intersections[s.intersectionID].signalIDs.push_back(s.signalID);
            signalIDsByIntersection[s.intersectionID].push_back(s.signalID);
        }
    }
    cout << "[SUMO] signal programs: " << signalPrograms.size() << endl;
    cout << "[SUMO] movement signal controllers: " << signals.size() << endl;
}

// SUMO static graph assembly.
// Prepares RoadSegment/Node/Movement/Signal/WaitingBuffer structures consumed by
// cycle_aware_signal_driven_records; it does not run the event simulation.
void Graph::build_new_graph_structures_from_sumo()
{
    classify_node_types_from_sumo_junctions();
    build_intersections_from_signalized_nodes();
    append_unsignalized_dispatch_intersections();
    build_movements_from_sumo_connections();
    build_lane_groups_from_sumo_connections();
    build_signal_programs_from_sumo_tllogic();
    build_waiting_buffers();
    cout << "[SUMO] waiting buffers: " << waitingBuffers.size() << endl;
    validate_cycle_aware_graph();
}

// Signal helper used by dispatch logic: evaluates movement signal state at time t.
// SignalEventPQ schedules phase transitions; this helper does not discharge vehicles.
SignalState Graph::signalStateAtMovement(int movementID, double t)
{
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return SignalState::Red;
    const Movement &m = movements[movementID];
    if (m.alwaysOpen) return SignalState::AlwaysOpen;
    if (m.tlID.empty()) {
        if (m.signalID >= 0 && m.signalID < static_cast<int>(signals.size())) {
            const SignalController &s = signals[m.signalID];
            if (s.alwaysOpen) return SignalState::AlwaysOpen;
            if (s.cycleLength <= 0) return SignalState::Red;
            double local = fmod(t - s.offset, s.cycleLength);
            if (local < 0.0) local += s.cycleLength;
            if (s.greenStart <= s.greenEnd) {
                return (local >= s.greenStart && local < s.greenEnd) ? SignalState::Green : SignalState::Red;
            }
            return (local >= s.greenStart || local < s.greenEnd) ? SignalState::Green : SignalState::Red;
        }
        return SignalState::AlwaysOpen;
    }
    auto programIt = tlIDToSignalProgramID.find(m.tlID);
    if (programIt == tlIDToSignalProgramID.end()) {
        cout << "[SUMO Warning] missing tlLogic for movement " << movementID << " tl=" << m.tlID << endl;
        return SignalState::Red;
    }
    const SignalProgram &p = signalPrograms[programIt->second];
    if (p.cycleLength <= 0 || p.phases.empty()) return SignalState::Red;
    double local = fmod(t - p.offset, p.cycleLength);
    if (local < 0.0) local += p.cycleLength;
    for (const auto &phase : p.phases) {
        if (local >= phase.startTime && local < phase.endTime) {
            if (m.linkIndex < 0 || m.linkIndex >= static_cast<int>(phase.state.size())) {
                cout << "[SUMO Warning] movement " << movementID << " linkIndex " << m.linkIndex
                     << " outside state size " << phase.state.size() << " for tl=" << m.tlID << endl;
                return SignalState::Red;
            }
            char s = phase.state[m.linkIndex];
            if (s == 'G' || s == 'g' || s == 'O') return SignalState::Green;
            if (s == 'y' || s == 'Y') return SignalState::Yellow;
            if (s == 'r' || s == 'R') return SignalState::Red;
            cout << "[SUMO Warning] unknown signal char '" << s << "' for movement " << movementID << endl;
            return SignalState::Red;
        }
    }
    return SignalState::Red;
}

TurnDir Graph::parseTurnDir(char c)
{
    switch (c) {
        case 'l':
        case 'L':
            return TurnDir::Left;
        case 's':
        case 'S':
            return TurnDir::Straight;
        case 'r':
        case 'R':
            return TurnDir::Right;
        case 't':
        case 'U':
            return TurnDir::UTurn;
        default:
            return TurnDir::Unknown;
    }
}

// Legacy BJ static graph assembly.
// Builds the same structured road/node/movement/signal/waiting-buffer graph that
// the core simulator consumes; this is preparation, not runtime dispatch.
void Graph::build_new_graph_structures(vector<vector<int>>& routeDataForSimulation)
{
    // 现行新算法预处理流水线：
    // 1) 由传统数据映射生成结构化 road/node/intersection
    // 2) 构建 movement/lane-group/signal/waiting-buffer
    // 3) 将 route 转为 roadID + movementID，供 cycle-aware 调度直接使用
    build_road_segments_from_legacy_roads();
    initialize_nodes_from_roads();
    classify_node_types_assume_all_junctions_signalized();
    build_intersections_from_signalized_nodes();
    attach_min_travel_time_to_roads();

    if (connections_to_direction.empty()) {
        readConnectionsToDirections(connections_to_direction_path);
    }
    build_movements_from_connections();
    build_default_lane_groups();
    attach_lane_groups_to_movements();
    build_signal_controllers_assume_default();
    build_waiting_buffers();
    route_nodeID_2_roadID(routeDataForSimulation);
    route_roadID_2_movementID();
    validate_cycle_aware_graph();
}

void Graph::build_road_segments_from_legacy_roads()
{
    roads.clear();
    roadIDToRoadIndex.clear();
    if (roadID2NodeID.empty()) {
        return;
    }

    int maxRoadID = -1;
    for (const auto &kv : roadID2NodeID) {
        maxRoadID = max(maxRoadID, kv.first);
    }
    roads.resize(maxRoadID + 1);

    for (const auto &kv : roadID2NodeID) {
        int roadID = kv.first;
        int fromNode = kv.second.first;
        int toNode = kv.second.second;

        RoadSegment segment;
        segment.roadID = roadID;
        segment.fromNode = fromNode;
        segment.toNode = toNode;
        if (roadID >= 0 && roadID < roadInfor.size()) {
            const Road &legacyRoad = roadInfor[roadID];
            segment.length = legacyRoad.length;
            segment.speedLimit = legacyRoad.speedLimit;
            segment.laneNum = max(1, legacyRoad.laneNum);
            segment.width = legacyRoad.width;
            segment.direction = legacyRoad.direction;
            segment.kindNumber = legacyRoad.kindNumber;
            segment.kind = legacyRoad.kind;
        }
        roads[roadID] = segment;
        roadIDToRoadIndex[roadID] = roadID;
    }
}

void Graph::initialize_nodes_from_roads()
{
    nodes.clear();
    nodeIDToNodeIndex.clear();
    incomingRoadsByNode.clear();
    outgoingRoadsByNode.clear();

    if (nodenum <= 0) {
        return;
    }
    nodes.resize(nodenum);
    for (int nodeID = 0; nodeID < nodenum; ++nodeID) {
        nodes[nodeID].nodeID = nodeID;
        nodes[nodeID].type = NodeType::NormalSplit;
        nodeIDToNodeIndex[nodeID] = nodeID;
    }

    for (const auto &road : roads) {
        if (road.roadID < 0 || road.fromNode < 0 || road.toNode < 0) {
            continue;
        }
        if (road.fromNode >= nodenum || road.toNode >= nodenum) {
            continue;
        }
        nodes[road.fromNode].outgoingRoads.push_back(road.roadID);
        nodes[road.toNode].incomingRoads.push_back(road.roadID);
        outgoingRoadsByNode[road.fromNode].push_back(road.roadID);
        incomingRoadsByNode[road.toNode].push_back(road.roadID);
    }
}

void Graph::classify_node_types_assume_all_junctions_signalized()
{
    for (auto &node : nodes) {
        // Temporary assumption. Replace with real signalized node input later.
        if (node.incomingRoads.size() >= 2 && node.outgoingRoads.size() >= 2) {
            node.type = NodeType::SignalizedJunction;
        } else if (!node.incomingRoads.empty() || !node.outgoingRoads.empty()) {
            node.type = NodeType::NormalSplit;
        } else {
            node.type = NodeType::NormalSplit;
        }
    }
}

void Graph::build_intersections_from_signalized_nodes()
{
    intersections.clear();
    nodeToIntersectionID.clear();

    int intersectionID = 0;
    for (auto &road : roads) {
        road.hasSignalizedDownstream = false;
        road.downstreamIntersectionID = -1;
    }

    for (auto &node : nodes) {
        if (node.type != NodeType::SignalizedJunction) {
            node.intersectionID = -1;
            continue;
        }
        node.intersectionID = intersectionID;
        Intersection inter;
        inter.intersectionID = intersectionID;
        inter.nodeID = node.nodeID;
        inter.incomingRoads = node.incomingRoads;
        inter.outgoingRoads = node.outgoingRoads;
        intersections.push_back(inter);
        nodeToIntersectionID[node.nodeID] = intersectionID;

        for (int incomingRoadID : node.incomingRoads) {
            if (incomingRoadID >= 0 && incomingRoadID < roads.size()) {
                roads[incomingRoadID].hasSignalizedDownstream = true;
                roads[incomingRoadID].downstreamIntersectionID = intersectionID;
            }
        }
        ++intersectionID;
    }
}

// An intersection is also a dispatch/competition group, not only a physical TLS.
// Append groups for actual external connections after all signalized groups so
// existing signalized IDs/order remain unchanged. Keep the imported node type
// and hasSignalizedDownstream flag truthful; no priority/foes rules are inferred.
void Graph::append_unsignalized_dispatch_intersections()
{
    set<int> connectedNodes;
    for (const auto& connection : sumoConnectionsRaw) {
        const auto from = sumoEdgeStrToRoadID.find(connection.fromEdge);
        const auto to = sumoEdgeStrToRoadID.find(connection.toEdge);
        if (from == sumoEdgeStrToRoadID.end() || to == sumoEdgeStrToRoadID.end()) continue;
        const int nodeID = roads[from->second].toNode;
        if (nodeID < 0 || nodeID >= static_cast<int>(nodes.size()) ||
            nodeID != roads[to->second].fromNode)
            throw runtime_error("SUMO external connection has inconsistent junction endpoints");
        connectedNodes.insert(nodeID);
    }
    for (auto& node : nodes) {
        if (node.intersectionID >= 0 || !connectedNodes.count(node.nodeID)) continue;
        Intersection group;
        group.intersectionID = static_cast<int>(intersections.size());
        group.nodeID = node.nodeID;
        group.incomingRoads = node.incomingRoads;
        group.outgoingRoads = node.outgoingRoads;
        node.intersectionID = group.intersectionID;
        nodeToIntersectionID[node.nodeID] = group.intersectionID;
        for (int roadID : node.incomingRoads)
            if (roadID >= 0 && roadID < static_cast<int>(roads.size()))
                roads[roadID].downstreamIntersectionID = group.intersectionID;
        intersections.push_back(std::move(group));
    }
}

void Graph::attach_min_travel_time_to_roads()
{
    for (auto &road : roads) {
        if (road.roadID < 0) continue;
        auto it = nodeID2minTime.find(make_pair(road.fromNode, road.toNode));
        if (it != nodeID2minTime.end()) {
            road.minTravelTime = it->second;
        }
    }
}

// Legacy static graph construction: road-pair connections become Movement objects.
// This prepares dispatch units for the core simulator but does not run dispatch.
void Graph::build_movements_from_connections()
{
    movements.clear();
    roadPairToMovementID.clear();
    outgoingMovementsByRoad.clear();
    incomingMovementsByRoad.clear();
    movementIDsByIntersection.clear();

    int movementID = 0;
    for (const auto &kv : connections_to_direction) {
        int fromRoad = kv.first.first;
        int toRoad = kv.first.second;
        if (fromRoad < 0 || toRoad < 0 || fromRoad >= roads.size() || toRoad >= roads.size()) {
            continue;
        }
        if (roads[fromRoad].roadID < 0 || roads[toRoad].roadID < 0) {
            continue;
        }

        Movement movement;
        movement.movementID = movementID;
        movement.fromRoadID = fromRoad;
        movement.toRoadID = toRoad;
        movement.turn = parseTurnDir(kv.second);
        movement.alwaysOpen = (movement.turn == TurnDir::Right);

        int downstreamNode = roads[fromRoad].toNode;
        if (downstreamNode >= 0 && downstreamNode < nodes.size() &&
            nodes[downstreamNode].type == NodeType::SignalizedJunction) {
            movement.intersectionID = nodes[downstreamNode].intersectionID;
        }

        movements.push_back(movement);
        roadPairToMovementID[make_pair(fromRoad, toRoad)] = movementID;
        outgoingMovementsByRoad[fromRoad].push_back(movementID);
        incomingMovementsByRoad[toRoad].push_back(movementID);

        if (movement.intersectionID >= 0 && movement.intersectionID < intersections.size()) {
            intersections[movement.intersectionID].movementIDs.push_back(movementID);
            movementIDsByIntersection[movement.intersectionID].push_back(movementID);
        }
        ++movementID;
    }
}

// Legacy compatibility path: synthesize lane groups when explicit SUMO lane
// connection data is unavailable. Used only to feed core lane-storage structures.
void Graph::build_default_lane_groups()
{
    laneGroups.clear();
    laneGroupsByRoad.clear();
    roadTurnToLaneGroupID.clear();

    auto build_lane_indices = [](int laneNum, TurnDir turn) -> vector<int> {
        int normalizedLaneNum = max(1, laneNum);
        if (normalizedLaneNum <= 1) {
            return {0};
        }
        if (normalizedLaneNum == 2) {
            if (turn == TurnDir::Left) return {0};
            return {1};
        }
        if (turn == TurnDir::Left) return {0};
        if (turn == TurnDir::Right) return {normalizedLaneNum - 1};
        vector<int> middle;
        for (int i = 1; i < normalizedLaneNum - 1; ++i) {
            middle.push_back(i);
        }
        if (middle.empty()) middle.push_back(1);
        return middle;
    };

    int laneGroupID = 0;
    for (const auto &road : roads) {
        if (road.roadID < 0) continue;
        const vector<TurnDir> turns = {TurnDir::Left, TurnDir::Straight, TurnDir::Right};
        for (TurnDir turn : turns) {
            LaneGroup group;
            group.laneGroupID = laneGroupID;
            group.roadID = road.roadID;
            group.turn = turn;
            group.laneIndices = build_lane_indices(road.laneNum, turn);
            laneGroups.push_back(group);

            laneGroupsByRoad[road.roadID].push_back(laneGroupID);
            roadTurnToLaneGroupID[make_pair(road.roadID, turn)] = laneGroupID;
            ++laneGroupID;
        }
    }
}

// Static mapping step: attach each movement to the lane group that serves its turn.
void Graph::attach_lane_groups_to_movements()
{
    for (auto &movement : movements) {
        auto it = roadTurnToLaneGroupID.find(make_pair(movement.fromRoadID, movement.turn));
        if (it != roadTurnToLaneGroupID.end()) {
            movement.laneGroupID = it->second;
            continue;
        }
        auto fallback = roadTurnToLaneGroupID.find(make_pair(movement.fromRoadID, TurnDir::Straight));
        movement.laneGroupID = (fallback != roadTurnToLaneGroupID.end()) ? fallback->second : -1;
    }
}

void Graph::build_signal_controllers_assume_default()
{
    signals.clear();
    signalIDsByIntersection.clear();
    int signalID = 0;
    constexpr double kDefaultCycleLength = 60.0;
    constexpr double kDefaultGreenStart = 0.0;
    constexpr double kDefaultGreenEnd = 30.0;

    for (auto &movement : movements) {
        if (movement.intersectionID < 0 || movement.intersectionID >= intersections.size()) {
            continue;
        }
        SignalController signal;
        signal.signalID = signalID;
        signal.intersectionID = movement.intersectionID;
        signal.movementID = movement.movementID;
        if (movement.turn == TurnDir::Right) {
            signal.alwaysOpen = true;
            signal.currentState = SignalState::AlwaysOpen;
        } else {
            signal.alwaysOpen = false;
            signal.cycleLength = kDefaultCycleLength;
            signal.greenStart = kDefaultGreenStart;
            signal.greenEnd = kDefaultGreenEnd;
            signal.offset = 0.0;
            signal.currentState = SignalState::Red;
        }
        signals.push_back(signal);
        movement.signalID = signalID;
        intersections[signal.intersectionID].signalIDs.push_back(signalID);
        signalIDsByIntersection[signal.intersectionID].push_back(signalID);
        ++signalID;
    }
}

// Static queue construction: create one WaitingBuffer per movement/lane group and
// store the movementID->bufferID mapping on the upstream RoadSegment.
void Graph::build_waiting_buffers()
{
    waitingBuffers.clear();
    int bufferID = 0;
    for (auto &road : roads) {
        road.movementIDToWaitingBufferID.clear();
    }

    for (const auto &movement : movements) {
        if (movement.intersectionID < 0) {
            continue;
        }
        WaitingBuffer buffer;
        buffer.bufferID = bufferID;
        buffer.roadID = movement.fromRoadID;
        buffer.movementID = movement.movementID;
        buffer.laneGroupID = movement.laneGroupID;

        if (movement.laneGroupID >= 0 && movement.laneGroupID < laneGroups.size()) {
            buffer.laneCount = max(1, laneGroups[movement.laneGroupID].laneCount());
        } else {
            buffer.laneCount = 1;
        }

        waitingBuffers.push_back(buffer);
        if (movement.fromRoadID >= 0 && movement.fromRoadID < roads.size()) {
            roads[movement.fromRoadID].movementIDToWaitingBufferID[movement.movementID] = bufferID;
        }
        ++bufferID;
    }
}

// Route conversion step: transform road sequences into Movement sequences consumed
// by VehicleLabel. Ambiguity handling here is data preparation, not dispatch policy.
void Graph::route_roadID_2_movementID()
{
    routeMovementID.clear();
    routeMovementID.resize(routeRoadID.size());

    int missingMovements = 0;
    int ambiguousMovements = 0;
    for (int i = 0; i < routeRoadID.size(); ++i) {
        if (routeRoadID[i].size() <= 1) {
            continue;
        }
        for (int j = 0; j + 1 < routeRoadID[i].size(); ++j) {
            int fromRoad = routeRoadID[i][j];
            int toRoad = routeRoadID[i][j + 1];
            int selectedMovementID = -1;
            auto multiIt = fromToRoadToMovementIDs.find(make_pair(fromRoad, toRoad));
            if (mergeRoadPairMovements) {
                if (multiIt != fromToRoadToMovementIDs.end() && multiIt->second.size() > 1)
                    throw runtime_error("road-pair grouping: route has multiple movements for one ordered road pair");
                const auto it = roadPairToMovementID.find(make_pair(fromRoad, toRoad));
                if (it != roadPairToMovementID.end()) selectedMovementID = it->second;
                if (selectedMovementID >= 0 && (selectedMovementID >= static_cast<int>(movements.size()) ||
                    movements[selectedMovementID].fromRoadID != fromRoad || movements[selectedMovementID].toRoadID != toRoad))
                    throw runtime_error("road-pair grouping: inconsistent route-to-movement mapping");
            } else if (multiIt != fromToRoadToMovementIDs.end() && !multiIt->second.empty()) {
                selectedMovementID = multiIt->second.front();
                for (int candidate : multiIt->second) {
                    if (candidate >= 0 && candidate < static_cast<int>(movements.size()) &&
                        movements[candidate].linkIndex >= 0) {
                        selectedMovementID = candidate;
                        break;
                    }
                }
                if (multiIt->second.size() > 1) {
                    ++ambiguousMovements;
                }
            } else {
                auto it = roadPairToMovementID.find(make_pair(fromRoad, toRoad));
                if (it != roadPairToMovementID.end()) selectedMovementID = it->second;
            }

            if (selectedMovementID >= 0) {
                routeMovementID[i].push_back(selectedMovementID);
            } else {
                ++missingMovements;
            }
        }
    }
    if (ambiguousMovements > 0) {
        cout << "[Validation] ambiguous fromRoad->toRoad movement choices (preferred valid linkIndex): "
             << ambiguousMovements << endl;
    }
    if (missingMovements > 0) {
        cout << "[Validation] missing movements from route conversion: " << missingMovements << endl;
    }

    int movementPreviewCount = min(5, static_cast<int>(routeMovementID.size()));
    for (int i = 0; i < movementPreviewCount; ++i) {
        cout << "[SUMO Route Preview] movementIDs:";
        for (int movementID : routeMovementID[i]) {
            cout << ' ' << movementID;
        }
        cout << endl;
    }
}

void Graph::validate_cycle_aware_graph()
{
    int invalidRoadEndpoints = 0;
    int missingIntersectionID = 0;
    int missingSignalizedRoadFlag = 0;
    int invalidMovements = 0;
    int missingSignal = 0;
    int missingBuffer = 0;
    int missingLaneGroup = 0;
    int missingRouteMovement = 0;

    for (const auto &road : roads) {
        if (road.roadID < 0) continue;
        if (road.fromNode < 0 || road.toNode < 0 || road.fromNode >= nodenum || road.toNode >= nodenum) {
            ++invalidRoadEndpoints;
        }
    }
    for (const auto &node : nodes) {
        if (node.type == NodeType::SignalizedJunction && node.intersectionID < 0) {
            ++missingIntersectionID;
        }
        if (node.type == NodeType::SignalizedJunction) {
            for (int incomingRoadID : node.incomingRoads) {
                if (incomingRoadID < 0 || incomingRoadID >= roads.size() ||
                    !roads[incomingRoadID].hasSignalizedDownstream) {
                    ++missingSignalizedRoadFlag;
                }
            }
        }
    }
    for (const auto &movement : movements) {
        if (movement.fromRoadID < 0 || movement.toRoadID < 0 ||
            movement.fromRoadID >= roads.size() || movement.toRoadID >= roads.size()) {
            ++invalidMovements;
        }
        if (movement.laneGroupID < 0) {
            ++missingLaneGroup;
        }
        if (movement.intersectionID >= 0) {
            if (!movement.alwaysOpen && movement.signalID < 0) {
                ++missingSignal;
            }
            if (movement.fromRoadID >= 0 && movement.fromRoadID < roads.size()) {
                auto bufferIt = roads[movement.fromRoadID].movementIDToWaitingBufferID.find(movement.movementID);
                if (bufferIt == roads[movement.fromRoadID].movementIDToWaitingBufferID.end()) {
                    ++missingBuffer;
                }
            }
        }
    }
    for (const auto &route : routeRoadID) {
        for (int j = 0; j + 1 < route.size(); ++j) {
            if (roadPairToMovementID.find(make_pair(route[j], route[j + 1])) == roadPairToMovementID.end()) {
                ++missingRouteMovement;
            }
        }
    }

    cout << "[Validation] invalid road endpoints: " << invalidRoadEndpoints << endl;
    cout << "[Validation] missing intersection ids: " << missingIntersectionID << endl;
    cout << "[Validation] missing signalized road flags: " << missingSignalizedRoadFlag << endl;
    cout << "[Validation] invalid movements: " << invalidMovements << endl;
    cout << "[Validation] missing lane groups: " << missingLaneGroup << endl;
    cout << "[Validation] missing signals: " << missingSignal << endl;
    cout << "[Validation] missing buffers: " << missingBuffer << endl;
    cout << "[Validation] missing movements in routes: " << missingRouteMovement << endl;
}

// Read Query with Defined Number
vector<vector<int>> Graph::read_query(string filename, int num)
{
    // Variable Initialization
    int input_num;
    vector<vector<int>> Q;
    int DepartureID;
    int DestinationID;
    double DepartureTime;
    vehicleDepartureTimes.clear();
    vehicleDeparturePositions.clear();
    vehicleDeparturePositionIssues.clear();
    // Read Query Data
    ifstream file_name(filename);
    require_open(file_name, filename, "read_query query data");
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than query data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0)
        input_num = lines;
    else
        input_num = num;
    // Read Query Data
    for (int i=0;i<input_num;i++){
        file_name >> DepartureID >> DestinationID >> DepartureTime;
        Q.push_back({
                DepartureID,
                DestinationID,
                static_cast<int>(floor(DepartureTime))
        });
        vehicleDepartureTimes.push_back(DepartureTime);
    }
    // Close File
    file_name.close();
    /*
    // Print Query Data
    for (int i=0;i<Q.size();i++)
    {
        cout << "query " << i << ": ";
        cout << Q[i][0] << " " << Q[i][1] << " " << Q[i][2] << endl;
    }
    */
    return Q;
}

// Read Route Data with Defined Number
vector<vector<int>> Graph::read_route(string filename, int num)
{
    // Variable Initialization
    vector<vector<int>> Pi;
    int input_num;
    int vertexNum, vertex;
    vector<int> pi;

    int delay_time, low_time, wait_time, driving_num;

    // Read Route Data
    ifstream file_name(filename);
    require_open(file_name, filename, "read_route route data");
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than route data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0){
        input_num = lines;
    }else{
        input_num = num;
    }
    // Read Route Data
    for (int i=0;i<input_num;i++){
        // Read One Route
        pi.clear();
        file_name >> vertexNum;

        // Correctness Check
        if (vertexNum == 0)
            cout << "route num is 0 when read in." << endl;
        for (int j=0;j<vertexNum;j++){
            file_name >> vertex;

            file_name >> delay_time >> low_time >> wait_time >> driving_num;
            route_time_Dict[i][vertex] = {delay_time, low_time, wait_time, driving_num};

            pi.push_back(vertex);
        }
        // Add Route into Route Data
        Pi.push_back(pi);
    }
    // Close File
    file_name.close();

    /*
    // Print Route Data
    for (int i=0;i<Pi.size();i++)
    {
        cout << "route " << i << ": ";
        for (int j=0;j<Pi[i].size();j++)
        {
            cout << Pi[i][j] << " ";
        }
        cout << endl;
    }
    */
    return Pi;
}

// Read time data with defined number
vector<vector<double>> Graph::read_time(
        string filename,
        int num,
        const vector<vector<int>>& query)
{
    // Variable Initialization
    vector<vector<double>> T;
    int input_num;
    int timeNum;
    double time;
    vector<double> t;
    // Read Route Data
    ifstream file_name(filename);
    require_open(file_name, filename, "read_time time data");
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than time data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0){
        input_num = lines;
    }else{
        input_num = num;
    }
    // Read Route Data
    for (int i=0;i<input_num;i++){
        // Read One Route
        t.clear();
        file_name >> timeNum;
        t.push_back(query[i][2]);
        // Correctness Check
        if (timeNum == 0)
            cout << "route num is 0 when read in." << endl;
        for (int j=0;j<timeNum;j++){
            file_name >> time;
            t.push_back(time);
        }
        // Add Route into Route Data
        T.push_back(t);
    }
    // Close File
    file_name.close();
    /*
    // Print Route Data
    for (int i=0;i<T.size();i++)
    {
        cout << "route " << i << ": ";
        for (int j=0;j<T[i].size();j++)
        {
            cout << T[i][j] << " ";
        }
        cout << endl;
    }
    */
    return T;
}


void Graph::min_depar_time(vector<vector<int>> &Q)
{
    if (Q.empty() || Q[0].size() < 3) {
        minDeparture = 0;
        minHour = 0;
        return;
    }

    minDeparture = Q[0][2];
    for (const auto &query : Q) {
        if (query.size() >= 3) {
            minDeparture = min(minDeparture, query[2]);
        }
    }

    minHour = minDeparture / 3600;
    cout << "The earliest departure time is: " << minDeparture << "s." << endl;
    cout << "The earliest departure time is in hour: " << minHour << endl;
}

// Check if route data, query data, and time data size are same
void Graph::check_size(){
    if (queryDataRaw.size() != routeDataRaw.size() || routeDataRaw.size() != timeDataRaw.size()) {
        cout << "Error. Data sizes are different." << endl;
        cout << "query=" << queryDataRaw.size()
             << " route=" << routeDataRaw.size()
             << " time=" << timeDataRaw.size() << endl;
    }
    else{
        cout << "Size of route, query, and time data currently is : " << routeDataRaw.size() << endl;
    }
}

// Convert Route from "Node ID Pair" to "Road ID"
void Graph::route_nodeID_2_roadID(vector<vector<int>> &routeData)
{
    routeRoadID.resize(routeData.size());
    int node01, node02, roadID;
    for (int i = 0; i < static_cast<int>(routeRoadID.size()); i++) {
        if (routeData[i].size() <= 1) continue;
        routeRoadID[i].resize(routeData[i].size() - 1);
        for (int j = 0; j < static_cast<int>(routeRoadID[i].size()); j++) {
            node01 = routeData[i][j];
            node02 = routeData[i][j + 1];
            auto roadIt = nodeID2RoadID.find(make_pair(node01, node02));
            if (roadIt != nodeID2RoadID.end()) {
                roadID = roadIt->second;
                routeRoadID[i][j] = roadID;
            } else {
                cout << "Warning. Unfounded node pairs are: " << node01 << " " << node02 << endl;
            }
        }
    }
}

// Cut route data
vector<vector<int>> Graph::cut_route_data(vector<vector<int>> &routeDataRaw, int avg_length)
{
    // Variable Initialization
    vector<vector<int>> routeData;
    vector<int> placeHolder;
    routeData.assign(routeDataRaw.size(), placeHolder);

    // Go Through Route Data and Select Parts
    for (int i=0;i<routeDataRaw.size();i++){
        // If route length is smaller than defined length, keep original one
        if (routeDataRaw[i].size() <= avg_length){
            routeData[i] = routeDataRaw[i];
        }
        else{
            vector<int>::const_iterator start = routeDataRaw[i].begin();
            vector<int>::const_iterator end = routeDataRaw[i].begin() + avg_length;
            vector<int> cutVector(start, end);
            routeData[i] = cutVector;
        }
    }
    /*
    // Print route data out
    for (int i=0;i<routeData.size();i++){
        cout << "route " << i << " size: " << routeData[i].size() << endl;
        cout << "route data are: ";
        for (int j=0;j<routeData[i].size();j++){
            cout << routeData[i][j] << " ";
        }
        cout << endl;
    }
    */
    return routeData;
}

// Cut query data
vector<vector<int>> Graph::cut_query_data(vector<vector<int>> &queryDataRaw, vector<vector<int>> &routeData, int avg_length)
{
    // Variable initialization
    vector<vector<int>> queryData = queryDataRaw;
    // Go through query data and modify its destination one
    for (int i=0;i<queryData.size();i++){
        queryData[i][1] = routeData[i][routeData[i].size()-1];
    }

    // Correctness Check
    for (int i=0;i<routeData.size();i++){
        if (routeData[i][0] != queryData[i][0])
            cout << "Error! First nodes are not match." << endl;
        if (routeData[i][routeData[i].size()-1] != queryData[i][1])
            cout << "Error! Last nodes are not match." << endl;
    }

    return queryData;
}

// Cut time data
vector<vector<double>> Graph::cut_time_data(
        const vector<vector<double>>& timeDataRaw,
        int avg_length)
{
    // Variable Initialization
    vector<vector<double>> timeData;
    vector<double> placeHolder;
    timeData.assign(timeDataRaw.size(), placeHolder);

    // Go Through Route Data and Select Parts
    for (int i=0;i<timeDataRaw.size();i++){
        // If route length is smaller than defined length, keep original one
        if (timeDataRaw[i].size() <= avg_length){
            timeData[i] = timeDataRaw[i];
        }
        else{
            vector<double>::const_iterator start = timeDataRaw[i].begin();
            vector<double>::const_iterator end = timeDataRaw[i].begin() + avg_length;
            vector<double> cutVector(start, end);
            timeData[i] = cutVector;
        }
    }
    /*
    // Print route data out
    for (int i=0;i<timeData.size();i++){
        cout << "time data " << i << " size: " << timeData[i].size() << endl;
        cout << "time data are: ";
        for (int j=0;j<timeData[i].size();j++){
            cout << timeData[i][j] << " ";
        }
        cout << endl;
    }
    */
    return timeData;
}
