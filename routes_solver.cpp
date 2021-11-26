#include "routes_solver.h"
 
namespace operations_research{

    // @brief Generate distance matrix.
    std::vector<std::vector<int64_t>> ComputeEuclideanDistanceMatrix(
        const std::vector<std::vector<int>>& locations) {
        std::vector<std::vector<int64_t>> distances =
            std::vector<std::vector<int64_t>>(
                locations.size(), std::vector<int64_t>(locations.size(), int64_t{ 0 }));
        for (int fromNode = 0; fromNode < locations.size(); fromNode++) {
            for (int toNode = 0; toNode < locations.size(); toNode++) {
                if (fromNode != toNode)
                    distances[fromNode][toNode] = static_cast<int64_t>(
                        std::max(abs(locations[toNode][0] - locations[fromNode][0]),
                            abs(locations[toNode][1] - locations[fromNode][1])));
            }
        }
        return distances;
    }

    // @brief Change the route index into array.
    std::vector<std::vector<int>> GetRoutes(const Assignment& solution,
        const RoutingModel& routing,
        const RoutingIndexManager& manager) {
        // Get vehicle routes and store them in a two dimensional array, whose
        // i, j entry is the node for the jth visit of vehicle i.
        std::vector<std::vector<int>> routes(manager.num_vehicles());
        // Get routes.
        for (int vehicle_id = 0; vehicle_id < manager.num_vehicles(); ++vehicle_id) {
            int64_t index = routing.Start(vehicle_id);
            routes[vehicle_id].push_back(manager.IndexToNode(index).value());
            while (!routing.IsEnd(index)) {
                index = solution.Value(routing.NextVar(index));
                routes[vehicle_id].push_back(manager.IndexToNode(index).value());
            }
        }
        return routes;
    }
    
    vector<vector<int>> GenerateSamples(int num_samples)
    //void GenerateSamples(int num_samples,vector<vector<int>>& source_data)
    {
        vector<vector<int>> source_data(num_samples, vector<int>(2, 0));
        int min_range = -500, max_range = 500;
        random_device rand_dev;
        //mt19937 generator(rand_dev());
        default_random_engine  generator(rand_dev());
        generator.seed(10);
        uniform_int_distribution<int> random_dist(min_range, max_range);
        // generate x;
        for (int i = 0; i < num_samples; i++)
            source_data[i][0] = random_dist(generator);
        // generate y;
        for (int i = 0; i < num_samples; i++)
            source_data[i][1] = random_dist(generator);
        return source_data;
    }

    int* CalculateStartEndPoints(vector<vector<int>>& scan_data)
    {
        int start = 0, end = 0;
        int index[2] = { 0 };
        int min_val = INT_MAX, max_val = INT_MIN;
        for (int i = 0; i < scan_data.size(); i++)
        {
            if (scan_data[i][0] + scan_data[i][1] < min_val)
            {
                min_val = scan_data[i][0] + scan_data[i][1];
                start = i;
            }
            else if (scan_data[i][0] + scan_data[i][1] > max_val)
            {
                max_val = scan_data[i][0] + scan_data[i][1];
                end = i;
            }

        }
        index[0] = start, index[1] = end;
        return index;
    }

    vector<vector<int64_t>> ComputeChebyshevDistanceMatrix(int num_samples, vector<vector<int>>& scan_data)
    //void ComputeChebyshevDistanceMatrix(int num_samples, vector<vector<int>> & scan_data, vector<vector<int64_t>>& chebyshev_dist)
    {
        vector<vector<int64_t>> chebyshev_dist(num_samples, vector<int64_t>(num_samples, 0));
        for (int i = 0; i < num_samples; i++)
        {
            for (int j = i; j < num_samples; j++)
            {
                int64_t dist = max(abs(scan_data[i][0] - scan_data[j][0]), abs(scan_data[i][1] - scan_data[j][1]));
                chebyshev_dist[i][j] = dist;
                chebyshev_dist[j][i] = dist;
            }
        }
        return chebyshev_dist;
    }

    // [START solution_printer]
    //! @brief Print the solution.
    //! @param[in] manager Index manager used.
    //! @param[in] routing Routing solver used.
    //! @param[in] solution Solution found by the solver.
    void PrintSolution(const RoutingIndexManager& manager,
        const RoutingModel& routing, const Assignment& solution) {
        // Inspect solution.
        LOG(INFO) << "Objective: " << solution.ObjectiveValue() << " miles";
        int64_t index = routing.Start(0);
        LOG(INFO) << "Route:";
        int64_t distance{ 0 };
        std::stringstream route;
        while (routing.IsEnd(index) == false) {
            route << manager.IndexToNode(index).value() << " -> ";
            int64_t previous_index = index;
            index = solution.Value(routing.NextVar(index));
            distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{ 0 });
        }
        LOG(INFO) << route.str() << manager.IndexToNode(index).value();
        LOG(INFO) << "Route distance: " << distance << "miles";
        LOG(INFO) << "";
        LOG(INFO) << "Advanced usage:";
        LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
    }
    // [END solution_printer]

    // @brief Solve the Tsp problem by define the start and end points.
    vector<int> Tsp(DefineStartDataModel data) {
        // Create Routing Index Manager
        // [START index_manager]
        //RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
        //    data.depot);
        RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
        data.starts, data.ends);
        // [END index_manager]

        // Create Routing Model.
        // [START routing_model]
        RoutingModel routing(manager);
        // [END routing_model]

        // [START transit_callback]
        const int transit_callback_index = routing.RegisterTransitCallback(
            [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                auto from_node = manager.IndexToNode(from_index).value();
                auto to_node = manager.IndexToNode(to_index).value();
                return data.distance_matrix[from_node][to_node];
            });
        // [END transit_callback]

        // Define cost of each arc.
        // [START arc_cost]
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
        // [END arc_cost]

        // Setting first solution heuristic.
        // [START parameters]
        // 
        RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        searchParameters.set_first_solution_strategy(
            FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION);

        //RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        //searchParameters.set_local_search_metaheuristic(
        //    LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        //searchParameters.mutable_time_limit()->set_seconds(2);
        //searchParameters.set_log_search(false);
        // [END parameters]

        // Solve the problem.
        // [START solve]
        //chrono::steady_clock::time_point inner_begin_time = chrono::steady_clock::now();
        const Assignment* solution = routing.SolveWithParameters(searchParameters);
        //chrono::steady_clock::time_point inner_end_time2 = chrono::steady_clock::now();
        //cout << "Inner Solve cost time: " << chrono::duration_cast<chrono::microseconds>(inner_end_time2 - inner_begin_time).count() / 1000.0 << endl;
        // [END solve]

        // Print solution on console.
        // [START print_solution]
        PrintSolution(manager, routing, *solution);
        // [END print_solution]

        // Generate the routes into an array.
        vector<vector<int>> scan_route = GetRoutes(*solution, routing, manager);
        return scan_route[0];
    }

    // @brief Solve the Tsp problem by random start point and return to the start point.
    vector<int> Tsp(RandomStartDataModel data) {
        // Create Routing Index Manager
        // [START index_manager]
        //RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
        //    data.depot);
        RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
            data.depot);
        // [END index_manager]

        // Create Routing Model.
        // [START routing_model]
        RoutingModel routing(manager);
        // [END routing_model]

        // [START transit_callback]
        const int transit_callback_index = routing.RegisterTransitCallback(
            [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                auto from_node = manager.IndexToNode(from_index).value();
                auto to_node = manager.IndexToNode(to_index).value();
                return data.distance_matrix[from_node][to_node];
            });
        // [END transit_callback]

        // Define cost of each arc.
        // [START arc_cost]
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
        // [END arc_cost]

        // Setting first solution heuristic.
        // [START parameters]
        // 
        RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        searchParameters.set_first_solution_strategy(
            FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION); //LOCAL_CHEAPEST_ARC 10355  PATH_CHEAPEST_ARC 10146   LOCAL_CHEAPEST_INSERTION 10139  PARALLEL_CHEAPEST_INSERTION 10310
                                                        //GLOBAL_CHEAPEST_ARC 10018    FIRST_UNBOUND_MIN_VALUE 10097.  PATH_MOST_CONSTRAINED_ARC   10173

        //RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        //searchParameters.set_local_search_metaheuristic(
        //    LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);   // AUTOMATIC  10146 115ms; GREEDY_DESCENT 10146  114 ms;  GUIDED_LOCAL_SEARCH   9990miles 2000ms; 
        //                                                      // SIMULATED_ANNEALING 9844miles 2000ms;   TABU_SEARCH    9844miles 2000ms
        //searchParameters.mutable_time_limit()->set_seconds(5); 
        //searchParameters.set_log_search(false);
        // [END parameters]

        // Solve the problem.
        // [START solve]
        //chrono::steady_clock::time_point inner_begin_time = chrono::steady_clock::now();
        const Assignment* solution = routing.SolveWithParameters(searchParameters);
        //chrono::steady_clock::time_point inner_end_time2 = chrono::steady_clock::now();
        //cout << "Inner Solve cost time: " << chrono::duration_cast<chrono::microseconds>(inner_end_time2 - inner_begin_time).count() / 1000.0 << endl;
        // [END solve]

        // Print solution on console.
        // [START print_solution]
        PrintSolution(manager, routing, *solution);
        // [END print_solution]

        // Generate the routes into an array.
        vector<vector<int>> scan_route = GetRoutes(*solution, routing, manager);
        return scan_route[0];
    }
}  // namespace operations_research