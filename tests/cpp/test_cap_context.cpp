#include "head.h"
#include "test_cap_context_fixture.h"
#include "test_cap_context_abc_fixture.h"
#include "cap_context_json.h"
#include <functional>
#include <cstdio>

namespace {
int checks=0;
void check(bool value,const std::string& message) {
    ++checks;if(!value)throw std::runtime_error(message);
}
void near(double value,double expected,const std::string& message) {
    check(std::isfinite(value)&&std::abs(value-expected)<=1e-10,message);
}
void rejects(const std::function<void()>& call,const std::string& message) {
    ++checks;try{call();}catch(const std::runtime_error&){return;}
    throw std::runtime_error("expected rejection: "+message);
}
void scan_tests() {
    const CapContextRecord ego{10,0,10,100,10}; // horizon 10, finish 20
    const CapContextRecord same{0,0,9,100,10};
    const CapContextRecord distant_slow{0,0,0,100,5};
    const CapContextRecord weak{0,0,10,104,10};
    const CapContextRecord boundary{0,0,10,105,10};
    const CapContextRecord strong{0,0,10,160,10};
    const CapContextRecord second{1,0,10,130,10};
    const CapContextRecord equal{1,0,10,160,10};
    check(scan_cap_context({},ego,10).context==0,"empty context");
    check(scan_cap_context({same},ego,10).context==0,"equal speed earlier entry");
    check(scan_cap_context({distant_slow},ego,10).context==0,"distant slow just finishes before ego");
    check(scan_cap_context({weak},ego,10).context==1,"weak context");
    const auto at_p=scan_cap_context({boundary},ego,10);
    near(at_p.potential,.05,"P exact boundary");check(at_p.context==2,"P=.05 strong group");
    const auto two=scan_cap_context({strong,second},ego,10);
    near(two.equivalent_count,1.5,"E is not count");check(two.context==2,"two cars still low E");
    const auto high=scan_cap_context({strong,equal},ego,10);
    near(high.equivalent_count,2,"E exact boundary");check(high.context==3,"E=2 high group");
    check(scan_cap_context({CapContextRecord{0,0,10,300,10}},ego,10).potential>1,"P unrestricted tail");
    check(scan_cap_context({ego},ego,10).eligible_count==0,"ego excluded");
    auto pending=strong;pending.planned_enter_time=11;
    const auto future=scan_cap_context({pending},CapContextRecord{10,0,13,100,10},10);
    check(future.eligible_count==0&&future.context==0,"freeze d excludes pending before future ego entry");
    check(scan_cap_context({pending},CapContextRecord{10,0,13,100,10},11).eligible_count==1,
          "entry exactly at decision eligible");
    const auto tiny=scan_cap_context({CapContextRecord{0,0,10,100.000000005,10}},ego,10);
    check(tiny.context==0&&tiny.potential==0&&tiny.equivalent_count==0,"epsilon zeroing");
    rejects([&]{scan_cap_context({},ego,11);},"decision after ego entry");
    rejects([&]{scan_cap_context({},CapContextRecord{10,0,10,100,0},10);},"zero cap");
    rejects([&]{scan_cap_context({},CapContextRecord{10,0,10,0,10},10);},"zero horizon");
    near(cap_context_free_speed(13.89,10000/3.6,1.2,13.89),13.89,"SF saturation");
    near(cap_context_free_speed(30,10,.5,13.89),5,"desired speed active cap");
    near(cap_context_free_speed(30,50,.5,13.89),6.945,"road limit active cap");
    rejects([]{cap_context_free_speed(10,50,std::numeric_limits<double>::quiet_NaN(),10);},"missing SF");
    rejects([]{cap_context_free_speed(10,0,1,10);},"invalid desired speed");
    near(cap_context_distance(545.6,true,5.1),540.5,"first distance");
    near(cap_context_distance(545.6,false,5.1),545.6,"ordinary road distance");
    rejects([]{cap_context_distance(5,true,5.1);},"base beyond end");
    CapContextLedger ledger;
    ledger.reserve(0,0,strong);ledger.reserve(0,0,equal);
    auto other_lane=second;other_lane.vehicle_id=2;ledger.reserve(0,1,other_lane);
    check(ledger.query(0,0,ego,10).context==3,"same lane only");
    rejects([&]{ledger.reserve(0,0,strong);},"duplicate reservation");
    rejects([&]{ledger.erase(0,1);},"stale route release");
    ledger.erase(0,0);
    check(ledger.query(0,0,ego,10).context==2&&ledger.size()==2,"swap-delete retained neighbor");
    ledger.erase(1,0);ledger.reserve(0,0,CapContextRecord{1,1,10,100,5});
    check(ledger.size()==2,"route revisit identity");
    ledger.clear();check(ledger.size()==0,"clear ledger");
}

void prepare(Graph& graph,bool enabled=true,bool abc=false) {
    graph.roads.resize(2);
    for(int i=0;i<2;++i) {
        auto& road=graph.roads[i];road.roadID=i;road.length=100;road.speedLimit=10;
        road.laneNum=1;road.fromNode=i;road.toNode=i+1;
    }
    graph.sumoVehicleTypes["car"]=VehicleType();
    graph.sumoVehicleTypes["car"].maxSpeed=10;
    graph.vehicleTypeIDs.assign(3,"car");
    graph.vehicleSpeedFactors={.5,1,1};
    graph.vehicleHasExplicitSpeedFactor.assign(3,true);
    graph.vehicleDeparturePositions.assign(3,5.1);
    graph.vehicleDeparturePositionIssues.assign(3,"");
    graph.vehicles.resize(3);
    for(int i=0;i<3;++i) {
        auto& vehicle=graph.vehicles[i];vehicle.vehicleID=i;vehicle.roadIndex=0;
        vehicle.routeRoadIDs={0,1};vehicle.currentRoadID=0;vehicle.valid=true;
    }
    graph.initializeRoadLaneStorage();graph.enableBasicFeatureLogging=true;
    graph.speedNetUseAcceleration=false;graph.speedNetUseFollowing=false;
    if(enabled) {
        std::istringstream input(abc ? cap_context_abc_test::fixture() : cap_context_test::fixture(),std::ios::binary);
        graph.capContextTable.load(input,abc);
        graph.travelTimeTableFormat=abc ? TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RLE_V1
                                      : TravelTimeTableFormat::CAMS_CAP_CONTEXT_RLE_V1;
        graph.travelTimeMode=TravelTimeMode::TABLE;
    } else graph.travelTimeMode=TravelTimeMode::SPEED_NET;
}
void abc_graph_tests() {
    // Eligible predecessor finishes freely at 9.49s. The logical reservation
    // remains until actual release, so ABC must distinguish B from C at d.
    Graph graph;prepare(graph,true,true);
    graph.vehicleSpeedFactors.assign(3,1.0);
    check(graph.usesCapContextTable()&&graph.usesABCCapContextTable(),"ABC version dispatch");
    graph.reserveLaneOccupancy(0,0,0);
    graph.predictRoadTravelTime(0,0,-1,0,0);
    check(graph.basicFeatureSnapshots.back().platoon_context==0,"ABC empty A");
    graph.reserveLaneOccupancy(1,0,0);
    const auto b_time=graph.predictRoadTravelTime(0,1,-1,1,0);
    const auto b=graph.basicFeatureSnapshots.back();
    check(b.platoon_context==4&&b.context_details.legacy_context==0,"ABC live nonoverdue B");
    near(b_time,graph.capContextTable.lookup(100,2,2,10,4),"runtime B uses axis4");
    graph.releaseLaneOccupancy(1);
    graph.reserveLaneOccupancy(2,0,0);
    const auto c_time=graph.predictRoadTravelTime(0,2,-1,12,0);
    const auto c=graph.basicFeatureSnapshots.back();
    check(c.platoon_context==5&&c.context_details.eligible_count==1,"ABC overdue C");
    near(c.context_details.max_overdue_seconds,2.51,"overdue uses free finish");
    near(c_time,graph.capContextTable.lookup(100,2,2,10,5),"runtime C uses axis5");
    graph.releaseLaneOccupancy(0);graph.releaseLaneOccupancy(2);
    check(graph.capContextLedger.size()==0,"ABC releases ledger entries");

    Graph downstream;prepare(downstream,true,true);
    downstream.vehicleSpeedFactors.assign(3,1.0);
    downstream.vehicles[0].roadIndex=1;downstream.reserveLaneOccupancy(0,1,0);
    downstream.predictRoadTravelTime(1,0,0,8,0,5);
    downstream.vehicles[1].roadIndex=1;downstream.reserveLaneOccupancy(1,1,0);
    downstream.predictRoadTravelTime(1,1,0,9,0,6);
    check(downstream.basicFeatureSnapshots.back().context_details.pending_count==1&&
          downstream.basicFeatureSnapshots.back().platoon_context==0,"ABC excludes pending at discharge");
    downstream.releaseLaneOccupancy(1);
    downstream.vehicles[2].roadIndex=1;downstream.reserveLaneOccupancy(2,1,0);
    downstream.predictRoadTravelTime(1,2,0,20,0,17);
    check(downstream.basicFeatureSnapshots.back().platoon_context==4,
          "ABC overdue measured at decision17, not planned entry20; predecessor finish18");
    downstream.releaseLaneOccupancy(0);downstream.releaseLaneOccupancy(2);
    check(downstream.capContextLedger.size()==0,"ABC downstream/terminal cleanup");
}

void graph_tests() {
    Graph graph;prepare(graph);
    graph.speedNetUseSpeedFactor=false;graph.speedNetDepartPosition=false;
    graph.reserveLaneOccupancy(0,0,0);
    const double first=graph.predictRoadTravelTime(0,0,-1,0,0);
    const auto& f=graph.basicFeatureSnapshots.back();
    near(f.road_length,100,"new table physical length key");
    near(f.context_distance,94.9,"first geometry independent of position-off");
    near(f.free_speed_cap,5,"cap independent of SF-off");
    check(f.platoon_context==0&&graph.capContextLedger.size()==1,"first snapshot then register");
    near(first,2,"new table not divided again by SF");
    graph.reserveLaneOccupancy(1,0,0);
    graph.predictRoadTravelTime(0,1,-1,1,0);
    check(graph.basicFeatureSnapshots.back().platoon_context==2,"following fast ego observes slow predecessor");
    graph.releaseLaneOccupancy(0);
    check(graph.capContextLedger.size()==1,"successful release removes upstream record");
    graph.vehicles[0].roadIndex=1;
    graph.reserveLaneOccupancy(0,1,0);
    graph.predictRoadTravelTime(1,0,0,8,0,5);
    near(graph.basicFeatureSnapshots.back().context_decision_time,5,"freeze discharge time");
    near(graph.basicFeatureSnapshots.back().time,8,"planned downstream entry");
    near(graph.basicFeatureSnapshots.back().context_distance,100,"downstream full geometry");
    graph.releaseLaneOccupancy(1);graph.vehicles[1].roadIndex=1;
    graph.reserveLaneOccupancy(1,1,0);
    graph.predictRoadTravelTime(1,1,0,9,0,6);
    check(graph.basicFeatureSnapshots.back().context_details.eligible_count==0,
          "pending reserved predecessor excluded at d although active at ego entry");
    graph.releaseLaneOccupancy(0);
    graph.releaseLaneOccupancy(1);
    check(graph.capContextLedger.size()==0,"all exits remove records");
    Graph missing;prepare(missing);missing.vehicleHasExplicitSpeedFactor[0]=false;
    missing.reserveLaneOccupancy(0,0,0);
    rejects([&]{missing.predictRoadTravelTime(0,0,-1,0,0);},"missing SF fatal");
    check(missing.capContextLedger.size()==0,"failed query does not insert record");
    Graph unknown;prepare(unknown);unknown.vehicleDeparturePositionIssues[0]="unsupported departPos=random";
    unknown.speedNetDepartPosition=false;unknown.reserveLaneOccupancy(0,0,0);
    rejects([&]{unknown.predictRoadTravelTime(0,0,-1,0,0);},"unsupported base fatal despite old off switch");
    Graph legacy;prepare(legacy,false);legacy.vehicleHasExplicitSpeedFactor[0]=false;
    legacy.reserveLaneOccupancy(0,0,0);
    near(legacy.predictRoadTravelTime(0,0,-1,0,0),9.49,"legacy formula unchanged with missing SF");
    check(legacy.capContextLedger.size()==0,"legacy mode never maintains F13 ledger");
    legacy.speedNetDepartPosition=false;
    near(legacy.predictRoadTravelTime(0,0,-1,0,0),10,"legacy full-road switch preserved");
    for(bool position_flag:{false,true}) for(bool sf_flag:{false,true}) {
        Graph flags;prepare(flags);flags.speedNetDepartPosition=position_flag;flags.speedNetUseSpeedFactor=sf_flag;
        flags.reserveLaneOccupancy(0,0,0);
        near(flags.predictRoadTravelTime(0,0,-1,0,0),first,"new table independent of both legacy flags");
        near(flags.basicFeatureSnapshots.back().context_distance,94.9,"new geometry fixed across old flags");
    }
}
void network_contract_tests() {
    struct NetFixture {
        std::string path;
        explicit NetFixture(const std::string& lanes) {
            static int sequence=0;path="cap_context_network_test_"+std::to_string(++sequence)+".net.xml";
            std::ifstream existing(path);if(existing.good())throw std::runtime_error("existing network fixture");
            std::ofstream out(path);
            out<<"<net><edge id=\"a\" from=\"x\" to=\"y\">"<<lanes<<"</edge>"
                "<junction id=\"x\" type=\"priority\"/><junction id=\"y\" type=\"priority\"/></net>";
        }
        ~NetFixture(){std::remove(path.c_str());}
    };
    const std::string base="<lane id=\"a_0\" index=\"0\" length=\"100\" speed=\"10\"/>";
    for(const std::string other:{"<lane id=\"a_1\" index=\"1\" length=\"101\" speed=\"10\"/>",
            "<lane id=\"a_1\" index=\"1\" length=\"100\" speed=\"11\"/>",
            "<lane id=\"a_1\" index=\"1\" length=\"0\" speed=\"10\"/>",
            "<lane id=\"a_1\" index=\"1\" length=\"100\" speed=\"nan\"/>"}) {
        NetFixture file(base+other);Graph graph;graph.travelTimeMode=TravelTimeMode::TABLE;
        graph.travelTimeTableFormat=TravelTimeTableFormat::CAMS_CAP_CONTEXT_RLE_V1;
        rejects([&]{graph.read_sumo_net_xml(file.path);},"new table rejects unsupported lane geometry");
    }
    NetFixture valid(base);Graph graph;graph.travelTimeMode=TravelTimeMode::TABLE;
    graph.travelTimeTableFormat=TravelTimeTableFormat::CAMS_CAP_CONTEXT_RLE_V1;
    graph.read_sumo_net_xml(valid.path);check(graph.roads.size()==1,"uniform lane accepted");
    NetFixture mixed(base+"<lane id=\"a_1\" index=\"1\" length=\"120\" speed=\"12\"/>");
    Graph legacy;legacy.read_sumo_net_xml(mixed.path);
    near(legacy.roads[0].length,110,"legacy network averaging unchanged");
}
void terminal_contract_tests() {
    struct RouteFixture {
        std::string path;
        explicit RouteFixture(const std::string& attributes) {
            static int sequence=0;path="cap_context_route_test_"+std::to_string(++sequence)+".rou.xml";
            std::ifstream existing(path);if(existing.good())throw std::runtime_error("existing route fixture");
            std::ofstream out(path);
            out<<"<routes><vType id=\"car\" maxSpeed=\"10\" length=\"5\" minGap=\"2.5\"/>"
                "<route id=\"r\" edges=\"a b\"/><vehicle id=\"v\" type=\"car\" route=\"r\" depart=\"0\" speedFactor=\"1\" "
               <<attributes<<"/></routes>";
        }
        ~RouteFixture(){std::remove(path.c_str());}
    };
    for(const std::string attributes:{"arrivalPos=\"25\"","arrivalPos=\"random\"",
                                     "arrivalEdge=\"0\"","arrivalEdge=\"\""}) {
        RouteFixture input(attributes);Graph graph;prepare(graph);
        graph.sumoEdgeStrToRoadID={{"a",0},{"b",1}};
        rejects([&]{graph.read_sumo_route_xml(input.path,-1);},"unsupported terminal geometry rejected for new table");
        Graph legacy;prepare(legacy,false);legacy.sumoEdgeStrToRoadID={{"a",0},{"b",1}};
        legacy.read_sumo_route_xml(input.path,-1);
        check(legacy.sumoVehicleIDs.size()==1,"old route behavior unchanged for terminal metadata");
    }
    for(const std::string attributes:{"","arrivalPos=\"max\"","arrivalPos=\"\""}) {
        RouteFixture input(attributes);Graph graph;prepare(graph);graph.sumoEdgeStrToRoadID={{"a",0},{"b",1}};
        graph.read_sumo_route_xml(input.path,-1);check(graph.sumoVehicleIDs.size()==1,"default max terminal accepted");
    }
}
void shared_events(const std::string& path) {
    std::ifstream input(path);std::ostringstream buffer;buffer<<input.rdbuf();
    const auto document=cap_context_json::JsonParser(buffer.str()).parse();
    CapContextLedger ledger;
    for(const auto& event:document.at("events").array) {
        const auto op=event.at("op").string;
        const int id=static_cast<int>(event.at("vehicle_id").number);
        const int route=static_cast<int>(event.at("route_index").number);
        if(op=="release") {ledger.erase(id,route);continue;}
        const CapContextRecord record{id,route,event.at("planned_enter_time").number,
            event.at("distance").number,event.at("free_speed_cap").number};
        if(op=="reserve") {ledger.reserve(0,0,record);continue;}
        const auto result=ledger.query(0,0,record,event.at("decision_time").number);
        check(result.context==event.at("expected_context").number,"shared context");
        near(result.potential,event.at("expected_P").number,"shared P");
        near(result.equivalent_count,event.at("expected_E").number,"shared E");
        check(result.eligible_count==event.at("expected_eligible").number,"shared eligible count");
        std::cout<<std::setprecision(17)<<"{\"name\":\""<<event.at("name").string
                 <<"\",\"context\":"<<result.context<<",\"P\":"<<result.potential
                 <<",\"E\":"<<result.equivalent_count<<",\"eligible_count\":"<<result.eligible_count<<"}\n";
    }
}
}
int main(int argc,char**argv) {
    try {
        if(argc==3&&std::string(argv[1])=="--event-fixture") {shared_events(argv[2]);return 0;}
        scan_tests();graph_tests();abc_graph_tests();network_contract_tests();terminal_contract_tests();
        std::cout<<"Cap/context feature checks passed: "<<checks<<"\n";return 0;
    }
    catch(const std::exception& error){std::cerr<<error.what()<<"\n";return 1;}
}
