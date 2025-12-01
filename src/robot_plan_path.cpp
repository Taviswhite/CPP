#include <iostream>
#include <cmath>
#include <string>
#include <mbot_bridge/robot.h>
#include <path_planning/utils/graph_utils.h>
#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/viz_utils.h>
#include <path_planning/graph_search/graph_search.h>
#include <path_planning/graph_search/distance_transform.h>

int main(int argc, char const *argv[])
{
    float goal_x = 0, goal_y = 0;
    
    if (argc < 2) {
        std::cerr << "Please provide the path to a map file as input.\n";
        return -1;
    }
    
    if (argc == 4) {
        goal_x = std::stof(argv,[object Object],);
        goal_y = std::stof(argv,[object Object],);
    }
    
    std::string map_file = argv,[object Object],;
    GridGraph graph;
    loadFromFile(map_file, graph);
    
    // TODO: Call your distance transform function if using checkCollisionFast().
    // HINT: You may want to set a new value for graph.collision_radius.
    
    // Set collision radius for safer navigation (adjust based on robot size)
    graph.collision_radius = 0.15; // 15cm radius, adjust as needed
    
    // Compute distance transform for fast collision checking
    distanceTransform(graph);
    
    Cell goal = posToCell(goal_x, goal_y, graph);
    
    // Initialize the robot.
    mbot_bridge::MBot robot;
    
    // Get the robot's SLAM pose.
    std::vector<float> pose = robot.readSlamPose();
    if (pose.size() == 0) {
        std::cerr << "No pose information! Can't plan." << std::endl;
        return -1;
    }
    
    Cell start = posToCell(pose,[object Object],, pose,[object Object],, graph);
    std::vector<Cell> path;
    
    // TODO: Call graph search function and put the result in path.
    
    // Validate start and goal positions
    if (isObstacle(cellToIdx(start.i, start.j, graph), graph)) {
        std::cerr << "Start position is in an obstacle!" << std::endl;
        return -1;
    }
    
    if (isObstacle(cellToIdx(goal.i, goal.j, graph), graph)) {
        std::cerr << "Goal position is in an obstacle!" << std::endl;
        return -1;
    }
    
    // Use A* search for optimal path planning
    path = aStarSearch(graph, start, goal);
    
    // Check if path was found
    if (path.empty()) {
        std::cerr << "No path found from start to goal!" << std::endl;
        return -1;
    }
    
    std::cout << "Path found with " << path.size() << " waypoints." << std::endl;
    
    // Drive the robot along the planned path
    robot.drivePath(cellsToPoses(path, graph));
    
    // Save the path output file for visualization in the nav app.
    generatePlanFile(start, goal, path, graph);
    
    return 0;
}
}
