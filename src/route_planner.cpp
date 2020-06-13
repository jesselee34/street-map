#include <iostream>
#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}


// Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->h_value = this->CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value;
            neighbor->parent = current_node;
            neighbor->visited = true;
            open_list.push_back(neighbor);
        }
    }
}

bool Compare (RouteModel::Node *a, RouteModel::Node *b) {
    int f1 = a->g_value + a->h_value;
    int f2 = b->g_value + b->h_value;
    return f1 > f2;
}

// Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *result;

    std::sort(open_list.begin(), open_list.end(), Compare);
    result = open_list.back();
    open_list.pop_back();
    return result;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *cur = current_node;

    // Implement your solution here.
    while (cur->x != this->start_node->x && cur->y != this->start_node->y) {
        if (path_found.empty()) {
            path_found.push_back(*cur);
        } else {
            path_found.insert(path_found.begin(), *cur);
        }

        distance += cur->distance(*cur->parent);
        cur = cur->parent;
    }

    // Include the starting node in the path
    path_found.insert(path_found.begin(), *this->start_node);
    distance += start_node->distance(*start_node->parent);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = this->start_node;

    // TODO: Implement your solution here.
    while (current_node->x != this->end_node->x && current_node->y != this->end_node->y) {
        this->AddNeighbors(current_node);
        current_node = this->NextNode();
    }

    m_Model.path = this->ConstructFinalPath(current_node);
}