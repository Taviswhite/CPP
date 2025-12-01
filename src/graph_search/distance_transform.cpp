#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <queue>
#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>
#include <path_planning/graph_search/distance_transform.h>

/**
 * Slow but straightforward distance transform implementation.
 * For each free cell, finds the minimum distance to any occupied cell.
 */
void distanceTransformSlow(GridGraph& graph)
{
    // Initialize the obstacle_distances vector
    graph.obstacle_distances.resize(graph.width * graph.height);
    
    // For each cell in the grid
    for (int i = 0; i < graph.height; i++) {
        for (int j = 0; j < graph.width; j++) {
            int current_idx = cellToIdx(i, j, graph);
            
            // If the cell is occupied, distance is 0
            if (isIdxOccupied(current_idx, graph)) {
                graph.obstacle_distances[current_idx] = 0.0f;
                continue;
            }
            
            // Find minimum distance to any occupied cell
            float min_distance = std::numeric_limits<float>::max();
            
            // Check all cells in the grid
            for (int oi = 0; oi < graph.height; oi++) {
                for (int oj = 0; oj < graph.width; oj++) {
                    int obstacle_idx = cellToIdx(oi, oj, graph);
                    
                    // If this cell is occupied, calculate distance
                    if (isIdxOccupied(obstacle_idx, graph)) {
                        float distance = sqrt(pow(i - oi, 2) + pow(j - oj, 2)) * graph.meters_per_cell;
                        min_distance = std::min(min_distance, distance);
                    }
                }
            }
            
            // Store the minimum distance found
            graph.obstacle_distances[current_idx] = min_distance;
        }
    }
}

/**
 * Manhattan distance transform using multi-pass algorithm.
 * More efficient than the slow version.
 */
void distanceTransformManhattan(GridGraph& graph)
{
    // Initialize the distance array
    graph.obstacle_distances.resize(graph.width * graph.height);
    
    // Initialize distances: 0 for obstacles, infinity for free space
    for (int i = 0; i < graph.height; i++) {
        for (int j = 0; j < graph.width; j++) {
            int idx = cellToIdx(i, j, graph);
            if (isIdxOccupied(idx, graph)) {
                graph.obstacle_distances[idx] = 0.0f;
            } else {
                graph.obstacle_distances[idx] = std::numeric_limits<float>::max();
            }
        }
    }
    
    // Forward pass: top-left to bottom-right
    for (int i = 0; i < graph.height; i++) {
        for (int j = 0; j < graph.width; j++) {
            int current_idx = cellToIdx(i, j, graph);
            
            if (!isIdxOccupied(current_idx, graph)) {
                float current_dist = graph.obstacle_distances[current_idx];
                
                // Check left neighbor
                if (j > 0) {
                    int left_idx = cellToIdx(i, j - 1, graph);
                    current_dist = std::min(current_dist, graph.obstacle_distances[left_idx] + graph.meters_per_cell);
                }
                
                // Check top neighbor
                if (i > 0) {
                    int top_idx = cellToIdx(i - 1, j, graph);
                    current_dist = std::min(current_dist, graph.obstacle_distances[top_idx] + graph.meters_per_cell);
                }
                
                graph.obstacle_distances[current_idx] = current_dist;
            }
        }
    }
    
    // Backward pass: bottom-right to top-left
    for (int i = graph.height - 1; i >= 0; i--) {
        for (int j = graph.width - 1; j >= 0; j--) {
            int current_idx = cellToIdx(i, j, graph);
            
            if (!isIdxOccupied(current_idx, graph)) {
                float current_dist = graph.obstacle_distances[current_idx];
                
                // Check right neighbor
                if (j < graph.width - 1) {
                    int right_idx = cellToIdx(i, j + 1, graph);
                    current_dist = std::min(current_dist, graph.obstacle_distances[right_idx] + graph.meters_per_cell);
                }
                
                // Check bottom neighbor
                if (i < graph.height - 1) {
                    int bottom_idx = cellToIdx(i + 1, j, graph);
                    current_dist = std::min(current_dist, graph.obstacle_distances[bottom_idx] + graph.meters_per_cell);
                }
                
                graph.obstacle_distances[current_idx] = current_dist;
            }
        }
    }
}

/**
 * 1D Euclidean distance transform using the envelope method.
 * This is a helper function for the 2D Euclidean distance transform.
 */
std::vector<float> distanceTransformEuclidean1D(std::vector<float>& init_dt)
{
    int n = init_dt.size();
    std::vector<float> dt(n);
    
    if (n == 0) return dt;
    if (n == 1) {
        dt,[object Object], = init_dt,[object Object],;
        return dt;
    }
    
    // Envelope method for 1D distance transform
    std::vector<int> v(n);      // Indices of parabola vertices
    std::vector<float> z(n + 1); // Locations of boundaries between parabolas
    
    int k = 0;  // Index of rightmost parabola in lower envelope
    v,[object Object], = 0;
    z,[object Object], = -std::numeric_limits<float>::max();
    z,[object Object], = std::numeric_limits<float>::max();
    
    // Build lower envelope
    for (int q = 1; q < n; q++) {
        float s;
        do {
            int r = v[k];
            s = ((init_dt[q] + q * q) - (init_dt[r] + r * r)) / (2 * q - 2 * r);
            if (s <= z[k]) {
                k--;
            } else {
                break;
            }
        } while (k >= 0);
        
        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = std::numeric_limits<float>::max();
    }
    
    // Fill in values of distance transform
    k = 0;
    for (int q = 0; q < n; q++) {
        while (z[k + 1] < q) {
            k++;
        }
        int r = v[k];
        dt[q] = sqrt((q - r) * (q - r) + init_dt[r]);
    }
    
    return dt;
}

/**
 * 2D Euclidean distance transform using separable algorithm.
 * Applies 1D distance transform along rows, then columns.
 */
void distanceTransformEuclidean2D(GridGraph& graph)
{
    // Initialize the distance array
    graph.obstacle_distances.resize(graph.width * graph.height);
    
    // Step 1: Initialize distances
    for (int i = 0; i < graph.height; i++) {
        for (int j = 0; j < graph.width; j++) {
            int idx = cellToIdx(i, j, graph);
            if (isIdxOccupied(idx, graph)) {
                graph.obstacle_distances[idx] = 0.0f;
            } else {
                graph.obstacle_distances[idx] = std::numeric_limits<float>::max();
            }
        }
    }
    
    // Step 2: Transform along rows (horizontal direction)
    for (int i = 0; i < graph.height; i++) {
        std::vector<float> row(graph.width);
        
        // Extract row
        for (int j = 0; j < graph.width; j++) {
            int idx = cellToIdx(i, j, graph);
            row[j] = graph.obstacle_distances[idx];
        }
        
        // Apply 1D distance transform
        std::vector<float> transformed_row = distanceTransformEuclidean1D(row);
        
        // Store back to graph
        for (int j = 0; j < graph.width; j++) {
            int idx = cellToIdx(i, j, graph);
            graph.obstacle_distances[idx] = transformed_row[j];
        }
    }
    
    // Step 3: Transform along columns (vertical direction)
    for (int j = 0; j < graph.width; j++) {
        std::vector<float> col(graph.height);
        
        // Extract column
        for (int i = 0; i < graph.height; i++) {
            int idx = cellToIdx(i, j, graph);
            col[i] = graph.obstacle_distances[idx];
        }
        
        // Apply 1D distance transform
        std::vector<float> transformed_col = distanceTransformEuclidean1D(col);
        
        // Store back to graph
        for (int i = 0; i < graph.height; i++) {
            int idx = cellToIdx(i, j, graph);
            graph.obstacle_distances[idx] = transformed_col[i] * graph.meters_per_cell;
        }
    }
}

// Optional: Breadth-First Search based distance transform (alternative to slow method)
void distanceTransformBFS(GridGraph& graph)
{
    // Initialize distances
    graph.obstacle_distances.resize(graph.width * graph.height);
    std::queue<Cell> queue;
    
    // Initialize: obstacles have distance 0, others have infinite distance
    for (int i = 0; i < graph.height; i++) {
        for (int j = 0; j < graph.width; j++) {
            int idx = cellToIdx(i, j, graph);
            if (isIdxOccupied(idx, graph)) {
                graph.obstacle_distances[idx] = 0.0f;
                queue.push(Cell(i, j));
            } else {
                graph.obstacle_distances[idx] = std::numeric_limits<float>::max();
            }
        }
    }
    
    // BFS to propagate distances
    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    while (!queue.empty()) {
        Cell current = queue.front();
        queue.pop();
        
        int current_idx = cellToIdx(current.i, current.j, graph);
        float current_dist = graph.obstacle_distances[current_idx];
        
        // Check all 4-connected neighbors
        for (const auto& dir : directions) {
            int ni = current.i + dir.first;
            int nj = current.j + dir.second;
            
            if (isCellInBounds(ni, nj, graph)) {
                int neighbor_idx = cellToIdx(ni, nj, graph);
                float new_dist = current_dist + graph.meters_per_cell;
                
                if (new_dist < graph.obstacle_distances[neighbor_idx]) {
                    graph.obstacle_distances[neighbor_idx] = new_dist;
                    queue.push(Cell(ni, nj));
                }
            }
        }
    }
}
