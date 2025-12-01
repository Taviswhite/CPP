#ifndef PATH_PLANNING_GRAPH_SEARCH_GRAPH_UTILS_H
#define PATH_PLANNING_GRAPH_SEARCH_GRAPH_UTILS_H

#include <vector>
#include <string>
#include <array>

#define HIGH 1e6
#define ROBOT_RADIUS 0.137

struct Cell
{
    int i, j; // Row and column index of the cell in the graph.
    
    // Constructor for convenience
    Cell() : i(-1), j(-1) {}
    Cell(int row, int col) : i(row), j(col) {}
    
    // Equality operator for comparisons
    bool operator==(const Cell& other) const {
        return i == other.i && j == other.j;
    }
};

/**
 * CellNode struct to store information needed for path planning.
 */
struct CellNode
{
    // Basic properties
    bool visited;           // Whether this node has been visited
    bool in_open_set;      // Whether this node is in the open set (for A*)
    
    // Path tracking
    int parent_idx;        // Index of parent node (-1 if no parent)
    Cell parent_cell;      // Parent cell coordinates
    
    // Cost information
    float g_cost;          // Cost from start to this node
    float h_cost;          // Heuristic cost from this node to goal
    float f_cost;          // Total cost (g + h)
    float distance;        // Distance from start (for BFS/DFS)
    
    // Constructor with default values
    CellNode() : 
        visited(false), 
        in_open_set(false),
        parent_idx(-1), 
        parent_cell(-1, -1),
        g_cost(HIGH), 
        h_cost(0), 
        f_cost(HIGH),
        distance(HIGH) {}
    
    // Reset function for graph initialization
    void reset() {
        visited = false;
        in_open_set = false;
        parent_idx = -1;
        parent_cell = Cell(-1, -1);
        g_cost = HIGH;
        h_cost = 0;
        f_cost = HIGH;
        distance = HIGH;
    }
};

struct GridGraph
{
    GridGraph() : 
        width(-1), 
        height(-1), 
        origin_x(0), 
        origin_y(0), 
        meters_per_cell(0), 
        collision_radius(0.15), 
        threshold(-100) // TODO: Adjust threshold based on your occupancy grid
    {};
    
    // Map properties
    int width, height;                          // Width and height of the map in cells
    float origin_x, origin_y;                  // The (x, y) coordinate corresponding to cell (0, 0) in meters
    float meters_per_cell;                     // Width of a cell in meters
    float collision_radius;                    // The radius to use to check collisions
    int8_t threshold;                          // Threshold to check if a cell is occupied or not
    
    // Map data
    std::vector<int8_t> cell_odds;             // The odds that a cell is occupied
    std::vector<float> obstacle_distances;      // The distance from each cell to the nearest obstacle
    std::vector<Cell> visited_cells;           // A list of visited cells. Used for visualization
    
    // Node data for path planning algorithms
    std::vector<CellNode> nodes;               // Node information for each cell in the graph
    
    // Helper methods
    void initializeNodes() {
        if (width > 0 && height > 0) {
            nodes.resize(width * height);
            for (auto& node : nodes) {
                node.reset();
            }
        }
    }
    
    // Get node by cell coordinates
    CellNode& getNode(int i, int j) {
        int idx = i * width + j;  // Assuming row-major order
        return nodes[idx];
    }
    
    const CellNode& getNode(int i, int j) const {
        int idx = i * width + j;
        return nodes[idx];
    }
    
    // Get node by index
    CellNode& getNode(int idx) {
        return nodes[idx];
    }
    
    const CellNode& getNode(int idx) const {
        return nodes[idx];
    }
};

// Function declarations (keeping your original signatures)
bool isLoaded(const GridGraph& graph);
bool loadFromFile(const std::string& file_path, GridGraph& graph);
std::string mapAsString(GridGraph& graph);
void initGraph(GridGraph& graph);

// Coordinate conversion functions
int cellToIdx(int i, int j, const GridGraph& graph);
Cell idxToCell(int idx, const GridGraph& graph);
Cell posToCell(float x, float y, const GridGraph& graph);
std::vector<float> cellToPos(int i, int j, const GridGraph& graph);

// Bounds and collision checking
bool isCellInBounds(int i, int j, const GridGraph& graph);
bool isIdxOccupied(int idx, const GridGraph& graph);
bool isCellOccupied(int i, int j, const GridGraph& graph);
bool checkCollisionFast(int idx, const GridGraph& graph);
bool checkCollision(int idx, const GridGraph& graph);

// Graph traversal
std::vector<int> findNeighbors(int idx, const GridGraph& graph);

// Path planning utilities
int getParent(int idx, const GridGraph& graph);
float getScore(int idx, const GridGraph& graph);
int findLowestScore(const std::vector<int>& node_list, const GridGraph& graph);
std::vector<Cell> tracePath(int goal, const GridGraph& graph);

// Utility function for converting cells to poses
static std::vector<std::array<float, 3>> cellsToPoses(std::vector<Cell>& path, GridGraph& graph)
{
    std::vector<std::array<float, 3>> pose_path;
    
    for (Cell& cell : path) {
        std::array<float, 3> pose;
        auto position = cellToPos(cell.i, cell.j, graph);
        pose,[object Object], = position,[object Object],;  // x
        pose,[object Object], = position,[object Object],;  // y
        pose,[object Object], = 0;            // theta (assuming 2D planning)
        pose_path.push_back(pose);
    }
    
    return pose_path;
}

// Additional utility functions you might need
namespace PathPlanningUtils {
    // Manhattan distance heuristic
    inline float manhattanDistance(const Cell& a, const Cell& b) {
        return static_cast<float>(abs(a.i - b.i) + abs(a.j - b.j));
    }
    
    // Euclidean distance heuristic
    inline float euclideanDistance(const Cell& a, const Cell& b) {
        float di = static_cast<float>(a.i - b.i);
        float dj = static_cast<float>(a.j - b.j);
        return sqrt(di * di + dj * dj);
    }
    
    // Check if two cells are the same
    inline bool cellsEqual(const Cell& a, const Cell& b) {
        return a.i == b.i && a.j == b.j;
    }
}

#endif // PATH_PLANNING_GRAPH_SEARCH_GRAPH_UTILS_H
