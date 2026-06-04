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
    vector<pair<int, float>> vecp_float;
    vecp_float.clear();
    graphLength.assign(nodenum, vecp_float);         //ID1, ID2, Length
    graphRoadID.assign(nodenum, vecp);         //ID1, ID2, RoadID
    roadInfor.clear();                         //Road Info (ID1, ID2, RoadID, Length, Time)
    set<int> setp; setp.clear();
    adjNodes.assign(nodenum, setp);            //ID1, ID2
    int ID1, ID2, edgeID;
    float length;
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
    vector<pair<int,int>> vecpTIme; vecpTIme.clear();
    graphTime.assign(nodenum, vecp);
    int ID1Time, ID2Time;
    double minTime;
    // Avoid Redundant Information
    set<pair<int,int>> edgeRedunTime;
    // Read Graph Min Travel Time as Weight
    for(int i=0;i<edgenum;i++){
        IFTime>>ID1>>ID2>>minTime;
        minTime = (int) minTime;
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
    int length;
    int direction;
    int speedLimit;
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
    sumoVehicleTypes.clear();
    sumoVehicleTypes["car"] = VehicleType();

    unordered_map<string, vector<string>> sumoRouteIDToEdges;
    bool insideVehicle = false;
    for (const auto &tag : tags) {
        if (tag.name == "vType" && !tag.closing) {
            VehicleType vt;
            vt.id = to_string_attr(tag.attrs, "id", "car");
            vt.accel = to_double_attr(tag.attrs, "accel", vt.accel);
            vt.decel = to_double_attr(tag.attrs, "decel", vt.decel);
            vt.sigma = to_double_attr(tag.attrs, "sigma", vt.sigma);
            vt.length = to_double_attr(tag.attrs, "length", vt.length);
            vt.minGap = to_double_attr(tag.attrs, "minGap", vt.minGap);
            vt.maxSpeed = to_double_attr(tag.attrs, "maxSpeed", vt.maxSpeed);
            if (!vt.id.empty()) sumoVehicleTypes[vt.id] = vt;
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
    int previewPrinted = 0;
    bool haveDepart = false;
    int firstDepart = 0;
    int lastDepart = 0;

    auto process_vehicle = [&](const string &vehicleID,
                               const string &routeRef,
                               const vector<string> &inlineEdges,
                               double depart,
                               const string &vehicleTypeID) -> bool {
        if (maxVehicles > 0 && validVehicles >= maxVehicles) return false;

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

        int departTime = static_cast<int>(round(depart));
        int startNode = roads[firstRoad].fromNode;
        int endNode = roads[lastRoad].toNode;
        queryDataRaw.push_back({startNode, endNode, departTime});
        routeRoadID.push_back(roadIDSequence);
        sumoVehicleIDs.push_back(vehicleID);
        string resolvedTypeID = vehicleTypeID.empty() ? "car" : vehicleTypeID;
        if (sumoVehicleTypes.find(resolvedTypeID) == sumoVehicleTypes.end()) {
            sumoVehicleTypes[resolvedTypeID] = VehicleType();
            sumoVehicleTypes[resolvedTypeID].id = resolvedTypeID;
        }
        vehicleTypeIDs.push_back(resolvedTypeID);
        ++validVehicles;
        if (!haveDepart) {
            firstDepart = departTime;
            haveDepart = true;
        }
        lastDepart = departTime;

        if (previewPrinted < 5) {
            cout << "[SUMO Route Preview] vehicle=" << vehicleID
                 << " depart=" << departTime;
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
    double currentDepart = 0.0;
    vector<string> currentInlineEdges;

    for (const auto &tag : tags) {
        if (maxVehicles > 0 && validVehicles >= maxVehicles) break;

        if (tag.name == "vehicle" && !tag.closing) {
            ++vehiclesScanned;
            currentVehicleID = to_string_attr(tag.attrs, "id", "vehicle_" + to_string(vehiclesScanned - 1));
            currentRouteRef = to_string_attr(tag.attrs, "route");
            currentVehicleTypeID = to_string_attr(tag.attrs, "type", "car");
            currentDepart = to_double_attr(tag.attrs, "depart", 0.0);
            currentInlineEdges.clear();

            if (tag.selfClosing) {
                process_vehicle(currentVehicleID, currentRouteRef, currentInlineEdges, currentDepart, currentVehicleTypeID);
                inVehicle = false;
            } else {
                inVehicle = true;
            }
        } else if (tag.name == "route" && !tag.closing && inVehicle) {
            string edges = to_string_attr(tag.attrs, "edges");
            if (!edges.empty()) currentInlineEdges = split_ws(edges);
        } else if (tag.name == "vehicle" && tag.closing && inVehicle) {
            process_vehicle(currentVehicleID, currentRouteRef, currentInlineEdges, currentDepart, currentVehicleTypeID);
            inVehicle = false;
            currentVehicleID.clear();
            currentRouteRef.clear();
            currentVehicleTypeID.clear();
            currentInlineEdges.clear();
        }
    }

    if (sumoVehicleTypes.find("car") == sumoVehicleTypes.end()) {
        sumoVehicleTypes["car"] = VehicleType();
    }
    cout << "[SUMO Route] vehicle types: " << sumoVehicleTypes.size() << endl;
    cout << "[SUMO Route] route definitions: " << sumoRouteIDToEdges.size() << endl;
    cout << "[SUMO Route] vehicles scanned: " << vehiclesScanned << endl;
    cout << "[SUMO Route] valid vehicles: " << validVehicles << endl;
    cout << "[SUMO Route] skipped vehicles: " << skippedVehicles << endl;
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

void Graph::read_sumo_net_xml(const string& netXmlPath)
{
    ifstream in(netXmlPath.c_str());
    require_open(in, netXmlPath, "read_sumo_net_xml SUMO net XML");
    stringstream buffer;
    buffer << in.rdbuf();
    vector<XmlTagLite> tags = parse_xml_tags_lite(buffer.str());

    sumoNodeStrToID.clear();
    nodeIDToSumoNodeStr.clear();
    sumoEdgeStrToRoadID.clear();
    roadIDToSumoEdgeStr.clear();
    tlIDToSignalProgramID.clear();
    sumoEdgesRaw.clear();
    sumoJunctionsRaw.clear();
    sumoConnectionsRaw.clear();
    sumoSignalPrograms.clear();
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
            c.fromLane = to_int_attr(tag.attrs, "fromLane", -1);
            c.toLane = to_int_attr(tag.attrs, "toLane", -1);
            c.via = to_string_attr(tag.attrs, "via");
            c.tl = to_string_attr(tag.attrs, "tl");
            c.linkIndex = to_int_attr(tag.attrs, "linkIndex", -1);
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
            p.offset = to_int_attr(tag.attrs, "offset", 0);
            sumoSignalPrograms.push_back(p);
            currentTL = &sumoSignalPrograms.back();
            if (tag.selfClosing) currentTL = nullptr;
        } else if (tag.name == "tlLogic" && tag.closing) {
            currentTL = nullptr;
        } else if (tag.name == "phase" && !tag.closing && currentTL != nullptr) {
            SumoSignalPhase phase;
            phase.duration = to_int_attr(tag.attrs, "duration", 0);
            phase.startTime = currentTL->cycleLength;
            phase.endTime = phase.startTime + phase.duration;
            phase.state = to_string_attr(tag.attrs, "state");
            currentTL->cycleLength = phase.endTime;
            currentTL->phases.push_back(phase);
        }
    }

    int nodeID = 0;
    for (const auto &j : sumoJunctionsRaw) {
        if (j.id.empty()) continue;
        sumoNodeStrToID[j.id] = nodeID;
        nodeIDToSumoNodeStr[nodeID] = j.id;
        ++nodeID;
    }
    nodenum = nodeID;
    graphLength.assign(nodenum, vector<pair<int, float>>());
    graphRoadID.assign(nodenum, vector<pair<int, int>>());
    adjNodes.assign(nodenum, set<int>());
    nodes.resize(nodenum);
    for (int i = 0; i < nodenum; ++i) nodes[i].nodeID = i;

    int roadID = 0;
    for (const auto &edge : sumoEdgesRaw) {
        if (edge.isInternal) continue;
        if (edge.id.empty()) continue;
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
        legacy.length = static_cast<int>(round(length));
        legacy.travelTime = max(1, static_cast<int>(round(length / max(0.1, speed))));
        legacy.direction = 0;
        legacy.speedLimit = static_cast<int>(round(speed));
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
        graphLength[fromNode].push_back(make_pair(toNode, static_cast<float>(length)));
        graphRoadID[fromNode].push_back(make_pair(toNode, roadID));
        adjNodes[fromNode].insert(toNode);
        nodes[fromNode].outgoingRoads.push_back(roadID);
        nodes[toNode].incomingRoads.push_back(roadID);
        outgoingRoadsByNode[fromNode].push_back(roadID);
        incomingRoadsByNode[toNode].push_back(roadID);
        ++roadID;
    }
    edgenum = roadID;

    cout << "[SUMO] parsed edge tags: " << totalEdges << endl;
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
    for (auto &node : nodes) node.type = NodeType::NormalSplit;
    for (const auto &j : sumoJunctionsRaw) {
        auto it = sumoNodeStrToID.find(j.id);
        if (it == sumoNodeStrToID.end()) continue;
        int nodeID = it->second;
        if (nodeID < 0 || nodeID >= static_cast<int>(nodes.size())) continue;
        if (j.type == "traffic_light") {
            nodes[nodeID].type = NodeType::SignalizedJunction;
        } else if (j.type == "priority" || j.type == "right_before_left") {
            nodes[nodeID].type = NodeType::UnsignalizedJunction;
        } else {
            nodes[nodeID].type = NodeType::NormalSplit;
        }
    }
}

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

    for (const auto &c : sumoConnectionsRaw) {
        auto fromIt = sumoEdgeStrToRoadID.find(c.fromEdge);
        auto toIt = sumoEdgeStrToRoadID.find(c.toEdge);
        if (fromIt == sumoEdgeStrToRoadID.end() || toIt == sumoEdgeStrToRoadID.end()) {
            continue;
        }
        int fromRoadID = fromIt->second;
        int toRoadID = toIt->second;
        string key = to_string(fromRoadID) + "|" + to_string(toRoadID) + "|" + c.tl + "|" + to_string(c.linkIndex);
        int movementID;
        auto existing = movementKeyToID.find(key);
        if (existing != movementKeyToID.end()) {
            movementID = existing->second;
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
            if (downstreamNode >= 0 && downstreamNode < static_cast<int>(nodes.size()) &&
                nodes[downstreamNode].type == NodeType::SignalizedJunction) {
                m.intersectionID = nodes[downstreamNode].intersectionID;
            }
            movements.push_back(m);
            movementKeyToID[key] = movementID;
            fromToRoadToMovementIDs[make_pair(fromRoadID, toRoadID)].push_back(movementID);
            if (roadPairToMovementID.find(make_pair(fromRoadID, toRoadID)) == roadPairToMovementID.end()) {
                roadPairToMovementID[make_pair(fromRoadID, toRoadID)] = movementID;
            }
            outgoingMovementsByRoad[fromRoadID].push_back(movementID);
            incomingMovementsByRoad[toRoadID].push_back(movementID);
            if (m.intersectionID >= 0 && m.intersectionID < static_cast<int>(intersections.size())) {
                intersections[m.intersectionID].movementIDs.push_back(movementID);
                movementIDsByIntersection[m.intersectionID].push_back(movementID);
            }
        }
        auto add_unique_lane = [](vector<string> &v, int x) {
            if (x < 0) return;
            string lane = to_string(x);
            if (find(v.begin(), v.end(), lane) == v.end()) v.push_back(lane);
        };
        add_unique_lane(movements[movementID].fromLanes, c.fromLane);
        add_unique_lane(movements[movementID].toLanes, c.toLane);
    }
    cout << "[SUMO] movements: " << movements.size() << endl;
}

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

void Graph::build_new_graph_structures_from_sumo()
{
    classify_node_types_from_sumo_junctions();
    build_intersections_from_signalized_nodes();
    build_movements_from_sumo_connections();
    build_lane_groups_from_sumo_connections();
    build_signal_programs_from_sumo_tllogic();
    build_waiting_buffers();
    cout << "[SUMO] waiting buffers: " << waitingBuffers.size() << endl;
    validate_cycle_aware_graph();
}

SignalState Graph::signalStateAtMovement(int movementID, int t)
{
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return SignalState::Red;
    const Movement &m = movements[movementID];
    if (m.alwaysOpen) return SignalState::AlwaysOpen;
    if (m.tlID.empty()) {
        if (m.signalID >= 0 && m.signalID < static_cast<int>(signals.size())) {
            const SignalController &s = signals[m.signalID];
            if (s.alwaysOpen) return SignalState::AlwaysOpen;
            if (s.cycleLength <= 0) return SignalState::Red;
            int local = (t - s.offset) % s.cycleLength;
            if (local < 0) local += s.cycleLength;
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
    int local = (t - p.offset) % p.cycleLength;
    if (local < 0) local += p.cycleLength;
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
        case 'U':
            return TurnDir::UTurn;
        default:
            return TurnDir::Unknown;
    }
}

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
    constexpr int kDefaultCycleLength = 60;
    constexpr int kDefaultGreenStart = 0;
    constexpr int kDefaultGreenEnd = 30;

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
            signal.offset = 0;
            signal.currentState = SignalState::Red;
        }
        signals.push_back(signal);
        movement.signalID = signalID;
        intersections[signal.intersectionID].signalIDs.push_back(signalID);
        signalIDsByIntersection[signal.intersectionID].push_back(signalID);
        ++signalID;
    }
}

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
            if (multiIt != fromToRoadToMovementIDs.end() && !multiIt->second.empty()) {
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
    int DepartureTime;
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
        Q.push_back({DepartureID,DestinationID,DepartureTime});
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
vector<vector<int>> Graph::read_time(string filename, int num, vector<vector<int>> query)
{
    // Variable Initialization
    vector<vector<int>> T;
    int input_num;
    int timeNum, time;
    vector<int> t;
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
vector<vector<int>> Graph::cut_time_data(vector<vector<int>> &timeDataRaw, int avg_length)
{
    // Variable Initialization
    vector<vector<int>> timeData;
    vector<int> placeHolder;
    timeData.assign(timeDataRaw.size(), placeHolder);

    // Go Through Route Data and Select Parts
    for (int i=0;i<timeDataRaw.size();i++){
        // If route length is smaller than defined length, keep original one
        if (timeDataRaw[i].size() <= avg_length){
            timeData[i] = timeDataRaw[i];
        }
        else{
            vector<int>::const_iterator start = timeDataRaw[i].begin();
            vector<int>::const_iterator end = timeDataRaw[i].begin() + avg_length;
            vector<int> cutVector(start, end);
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
