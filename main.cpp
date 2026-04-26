#include "head.h"

int main() {


    // Class "Graph"
    Graph g;

    // Step 1: Data Cleaning
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 1: Data Cleaning" << endl;
    cout << "-------------------------------------" << endl;

    cout << "Data Cleaning Done." << endl;

    // Step 2: Data Preparation
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 2: DATA PREPARE" << endl;
    cout << "-------------------------------------" << endl;

    // Read Road Network
    g.read_graph();        // Can Be "one-way" (edge:651748) or "two-way" (edge:774660)

    /*int ID1 = g.roadID2NodeID[18358].first;
    int ID2 = g.roadID2NodeID[25901].second;
    vector<int> test_path = g.Dijg.buildDictionary Start _vetex(ID1, ID2);
    g.(test_path);*/

    // Read Query Data // 192484
    int readNum = 192484;    // Number of Routes for Simulation Operation
    // int readNum = 150000;
    g.percent = 1;
    g.small = 0; g.big = 0;
    // int readNum = 173235;    // 90%
    // int readNum = 153987;    // 80%
    // int readNum = 134739;    // 70%
    // int readNum = 115490;    // 60%
    // int readNum = 96242;    // 50%



    g.queryDataRaw = g.read_query(g.queryPath, readNum);
    cout << "Length of query is: " << g.queryDataRaw.size() << endl;
    g.routeDataRaw = g.read_route(g.route_path, readNum);
    cout << "Length of route is: " << g.routeDataRaw.size() << endl;
    g.timeDataRaw = g.read_time(g.time_path, readNum, g.queryDataRaw);
    cout << "Length of time is: " << g.timeDataRaw.size() << endl;
    // tuple<std::vector<int>, int> temp = g.read_time_no_wait(g.time_path_no_wait, readNum);
    // g.time_no_wait = get<0>(temp);
    // int route_avg_length = get<1>(temp);
    int avg_length = 30;    // Parameter (Average Route Length)
    bool cut = false;		// If cut data or not
    /*if (route_avg_length != avg_length) {
        if (cut == false and route_avg_length == 0){
            cout << "No cut." << endl;
        } else {
            cout << "Error. avg route length defined in no wait travel time is: " << route_avg_length << endl;
        }
    }*/
    // Check if route data, query data, and time data size are same
    g.check_size();
    // Remove data with duplicate values
    // g.removeDuplicates();

    // Split Route and Query Data to Average Length
    vector<vector<int>> queryData;
    vector<vector<int>> routeData;
    vector<vector<int>> timeData;
    if (cut == true){
        /*pair<vector<vector<int>>, vector<vector<int>>> dataCombine;
		dataCombine = g.data_length_modify(g.queryDataRaw, g.routeDataRaw, avg_length);
        queryData = dataCombine.first;
        routeData = dataCombine.second;*/
        routeData = g.cut_route_data(g.routeDataRaw, avg_length);
        queryData = g.cut_query_data(g.queryDataRaw, routeData, avg_length);
        timeData = g.cut_time_data(g.timeDataRaw, avg_length);
    }
    else
    {
        queryData = g.queryDataRaw;
        routeData = g.routeDataRaw;
        timeData = g.timeDataRaw;
    }
    // Find Min Departure Time from Queries
    g.min_depar_time(queryData);
    // Convert Route from "Node ID Pair" to "Road ID"
    g.route_nodeID_2_roadID(routeData);
    // Classify Each Road with A Unique Latency Function
    g.classify_latency_function();
    // Define Flow Base
    g.minRange = 20;    // minRange -> Min Time Range
    g.flowIni = 0;      // flowIni -> Initialized Flow Base Value
    g.flow_base_ini(g.minRange, g.flowIni);

    // Step 3: ALGORITHM I SIMULATION
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 3: ALGORITHM I SIMULATION" << endl;
    cout << "-------------------------------------" << endl;

    bool range = true;

    // Estimate average travel time of ground truth
    float AGVTime = g.AVG_estimation(routeData, timeData);
    //
    g.read_edge_feature_2_map(g.edge_id_to_features_path);
    //
    cout << "g.buildDictionary Start " << endl;
    g.buildDictionary(g.model_catch_dic_path);

    g.readConnectionsToDirections(g.connections_to_direction_path);
    // Algorithm I: Simulation
    bool server = false;             // connect Python server
    bool catching = false;            // apply model catching dictionary
    bool write = false;               // store input features to construct model catching dictionary
    bool latency = true;            // if apply latency function to estimate travel time
    string te_choose = "catching";    // decide use which results from "server", "catching", and "latency"
    cout << "Simulation Start." << endl;
    vector<vector<pair<int, float>>> ETA = g.alg1Records(
            queryData, routeData, range, server, catching, write, latency, te_choose);
    cout << "Catching Found: " << g.catching_found << " & Catching No-Found: " << g.catching_no_found << endl;
    // Estimate average travel time of ground truth
    /*float AGVTime = g.AVG_estimation(routeData, timeData);*/
    // Estimate travel time MSE between simulated results and truth
    float MSE = g.MSE_estimation(timeData, ETA);
    // Export record information on each edge for further analysis
    // g.export_time_records("1k", routeData, routeDataEdge, timeData, ETA);
    g.Traffic_Prediction(g.ETA_result);

    cout << "Number of small is: " << g.small << endl;
    cout << "Number of big is: " << g.big << endl;

    /*
    // Step 4: ROUTE DATA UPDATE OPERATIONS PREPARATION
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 4: ROUTE DATA UPDATE OPERATIONS PREPARATION" << endl;
    cout << "-------------------------------------" << endl;

    // Check Simulation
    bool simulation = 1;    // simulation = 1 -> records based on simulation operation
    g.check_simulation(simulation);
    // Convert Time Records from Node ID Pairs to Road ID
    g.nodeID_2_roadID_in_records(g.timeFlowChange);
    // Check Correctness of Time Records
    g.time_record_correct_check();
    // Split Time Flow Change into Time Slices
    g.split_2_time_slices(g.route_timeFlowChange);

    // Step 5: ROUTE DATA INSERTION OPERATIONS
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 5: ROUTE DATA INSERTION OPERATIONS" << endl;
    cout << "-------------------------------------" << endl;

    bool terminal = true;   // Parameter (Decide If Active Terminal Condition)
    bool parallel = false;  // Decide If Parallel Operation

    // Generate New Insertion Data & Write into Files
    // Do Not Have to Generate Every Time. Only Generate When Change Data or Numbers
    if (simulation == 1) {    // simulation = 1 -> records based on simulation operation
        int newGenerNum = 1000;     // Number of Routes for Insertion Operation
        cout << "Add " << newGenerNum << " New Route." << endl;
        srand((unsigned)time(NULL));
        g.data_generation(g.routeRoadPath, g.deparTimePath, g.routeNodePath, newGenerNum, avg_length, cut);
        // Variable Initialization
        g.Pi.reserve(readNum + newGenerNum);
        g.Pi = routeData;
        g.routeDataSize = routeData.size();
    }
    else{    // simulation = 0 -> same data with input of simulation operation
        cout << "Inserting " << readNum << " routes for clear road network" << endl;
        // Variable Initialization
        g.Pi.clear();
        g.routeDataSize = 0;
    }
    // Read Generated Route Data
    g.read_new_data(g.routeRoadPath, g.deparTimePath, g.routeNodePath);
    // "ETA_result" Initialization
    g.ETA_initialization(simulation, false); // bool -> not print ETA

    // Insertion Operation (Main)
    g.update_operation_insertion(parallel, true, range, false);   // w Termination (terminal == true; range == true & print == false)
    g.update_operation_insertion(parallel, false, range, false);  //w/o Termination (terminal == false; range == true & print == false)
    // Insertion Operation w Parallel
    // Split New Route Data into Groups
    parallel = true;
    int threadNum = 7;		// Define Thread Number
    g.multi_new_data_initial();
    g.update_operation_parallel(g.newDataMulti, parallel, true, range, false, threadNum);  // Parallel w Termination (range == true & print == false)
    g.update_operation_parallel(g.newDataMulti, parallel, false, range, false, threadNum);  // Parallel w/o Termination (range == true & print == false)
    */

    /*
    // Step 6: TRAVEL TIME ESTIMATION (ETA) UPDATE
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 6: TRAVEL TIME ESTIMATION (ETA) UPDATE" << endl;
    cout << "-------------------------------------" << endl;

    // "ETA_result" Update
    g.ETA_update();

    // Step 7: ROUTE DATA DELETION OPERATIONS
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 7: ROUTE DATA INSERTION OPERATIONS" << endl;
    cout << "-------------------------------------" << endl;

    // Generate New Deletion Data & Write into Files
    // Do Not Have to Generate Every Time. Only Generate When Change Data or Numbers
    int deleteNum = 1000;   // // Number of Routes for Deletion Operation
    parallel = false;
    cout << "Generate " << deleteNum << " existing route for deletion operation." << endl;
    srand((unsigned)time(NULL));
    g.dele_data_generation(g.routeRoadPathD, g.routeNodePathD, g.routeIndexpath, deleteNum);
    // Read Generated Route Data
    g.read_deletion_data(g.routeRoadPathD, g.routeIndexpath);
    // Deletion Operation
    g.update_operation_deletion(parallel, true, range, false);   // w Termination (range == true & print == false)
    g.update_operation_deletion(parallel, false, range, false);  // w/o Termination (range == true & print == false)
    // Deletion Operation Parallel
    // Split Target Route Data into Groups
    parallel = true;
    g.multi_del_data_initial();
    g.del_operation_parallel(g.delDataMulti, parallel, true, range, false, threadNum);	 // Parallel w Termination (range == true & print == false)
    g.del_operation_parallel(g.delDataMulti, parallel, false, range, false, threadNum);  // Parallel w/o Termination (range == true & print == false)
    */

    // Step 8: ... ...
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 8: ... ..." << endl;
    cout << "-------------------------------------" << endl;
    std::cout << "All Operations Done." << std::endl;
    return 0;
}
