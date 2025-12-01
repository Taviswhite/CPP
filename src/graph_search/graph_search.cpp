std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path; // The final path should be placed here.
    initGraph(graph); // Make sure all the node values are reset.
    
    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    
    // Priority queue for A* (min-heap based on f-cost)
    auto compare = [&graph](int a, int b) {
        return graph.nodes[a].f_cost > graph.nodes[b].f_cost;
    };
    std::priority_queue<int, std::vector<int>, decltype(compare)> open_set(compare);
    
    std::vector<bool> in_open_set(graph.width * graph.height, false);
    std::vector<bool> in_closed_set(graph.width * graph.height, false);
    
    // Initialize start node
    graph.nodes[start_idx].g_cost = 0;
    graph.nodes[start_idx].h_cost = heuristic(start_idx, goal_idx, graph);
    graph.nodes[start_idx].f_cost = graph.nodes[start_idx].g_cost + graph.nodes[start_idx].h_cost;
    graph.nodes[start_idx].parent = -1;
    
    open_set.push(start_idx);
    in_open_set[start_idx] = true;
    
    while (!open_set.empty()) {
        int current_idx = open_set.top();
        open_set.pop();
        in_open_set[current_idx] = false;
        in_closed_set[current_idx] = true;
        
        Cell current_cell = idxToCell(current_idx, graph);
        graph.visited_cells.push_back(current_cell);
        
        // Check if we reached the goal
        if (current_idx == goal_idx) {
            return tracePath(goal_idx, graph);
        }
        
        // Explore neighbors
        std::vector<int> neighbors = getNeighbors(current_idx, graph);
        for (int neighbor_idx : neighbors) {
            if (in_closed_set[neighbor_idx] || isObstacle(neighbor_idx, graph)) {
                continue;
            }
            
            double tentative_g_cost = graph.nodes[current_idx].g_cost + 
                                    distance(current_idx, neighbor_idx, graph);
            
            if (!in_open_set[neighbor_idx]) {
                // New node
                graph.nodes[neighbor_idx].g_cost = tentative_g_cost;
                graph.nodes[neighbor_idx].h_cost = heuristic(neighbor_idx, goal_idx, graph);
                graph.nodes[neighbor_idx].f_cost = graph.nodes[neighbor_idx].g_cost + 
                                                  graph.nodes[neighbor_idx].h_cost;
                graph.nodes[neighbor_idx].parent = current_idx;
                
                open_set.push(neighbor_idx);
                in_open_set[neighbor_idx] = true;
            } else if (tentative_g_cost < graph.nodes[neighbor_idx].g_cost) {
                // Better path found
                graph.nodes[neighbor_idx].g_cost = tentative_g_cost;
                graph.nodes[neighbor_idx].f_cost = graph.nodes[neighbor_idx].g_cost + 
                                                  graph.nodes[neighbor_idx].h_cost;
                graph.nodes[neighbor_idx].parent = current_idx;
            }
        }
    }
    
    return path; // Return empty path if no solution found
}
