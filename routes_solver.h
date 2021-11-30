#pragma once
#ifndef __ROUTES_SOLVER_H__
#define __ROUTES_SOLVER_H__


#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
#include <random>
#include <chrono>
#include <cmath>
#include <cstdint>
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace std;

namespace operations_research {

    // @brief Generate distance matrix.
    std::vector<std::vector<int64_t>> ComputeEuclideanDistanceMatrix(
        const std::vector<std::vector<int>>& locations);


    std::vector<std::vector<int>> GetRoutes(const Assignment& solution,
        const RoutingModel& routing,
        const RoutingIndexManager& manager);

    // [START data_model] Random start the routes.
    struct RandomStartDataModel {
        std::vector<std::vector<int64_t>> distance_matrix;
        int num_vehicles = 1;
        RoutingIndexManager::NodeIndex depot;
    };
    // [END data_model]

    // @brief Define the start and end points of the routes.
    struct DefineStartDataModel {
        std::vector<std::vector<int64_t>> distance_matrix;
        int num_vehicles = 1;
        std::vector<RoutingIndexManager::NodeIndex> starts;
        std::vector<RoutingIndexManager::NodeIndex> ends;
    };

    // @brief Generate Samples using random sampling. The distribution is uniform distribution.
    //void GenerateSamples(int num_samples, vector<vector<int>> & source_data);
    vector<vector<int>> GenerateSamples(int num_samples, int max_range, int min_range);

    // @brief Computue the distance matrix given the sample points.
    vector<vector<int64_t>> ComputeChebyshevDistanceMatrix(int num_samples, vector<vector<int>>& scan_data);
    //void ComputeChebyshevDistanceMatrix(int num_samples, vector<vector<int>>& scan_data, vector<vector<int64_t>>& chebyshev_dist);

    //// @brief Calculate the start and end points of the sample points. 
    //// Start point closet to [-5,-5]; End point closet to [5, 5].
    int* CalculateStartEndPoints(vector<vector<int>>& scan_data);

    // [START solution_printer]
    //! @brief Print the solution.
    //! @param[in] manager Index manager used.
    //! @param[in] routing Routing solver used.
    //! @param[in] solution Solution found by the solver.
    void PrintSolution(const RoutingIndexManager& manager,
        const RoutingModel& routing, const Assignment& solution);
    // [END solution_printer]

    vector<int> Tsp(RandomStartDataModel data);
    vector<int> Tsp(DefineStartDataModel data);
}  // namespace operations_research

#endif // !__ROUTES_SOLVER_H__