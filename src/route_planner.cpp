#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float HValue = node->distance(*(this->end_node));
    return HValue;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populating the neighbors.
    current_node->FindNeighbors();
  
    for (auto *neighbor : (*current_node).neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
      
      	// Use CalculateHValue to implement the calculation of h-Value.
        neighbor->h_value = this->CalculateHValue(neighbor);
      
        this->open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}

// Define a helper function on your own to sort the open_list according to the f value (sum of the h value and g value).
bool Compare(RouteModel::Node *node_a, RouteModel::Node *node_b) {
    return (node_a->g_value+node_a->h_value) > (node_b->g_value+node_b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sorting open nodes.
    std::sort(this->open_list.begin(), this->open_list.end(), &Compare);
  
    // Create a pointer to the node in the list with the lowest f value.
    RouteModel::Node *least_fValue_Node = this->open_list.back();
  
    // Remove that node from the open_list.
    this->open_list.pop_back();
  
    return least_fValue_Node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector.
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != this->start_node) {
        distance += current_node->distance(*(current_node->parent));
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    if(current_node->parent == nullptr)  
    	path_found.push_back(*(this->start_node));
  
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);
 
    RouteModel::Node *current_node = nullptr;

    while (!this->open_list.empty()) {
      current_node = this->NextNode();

      // if we've found the goal node, we're done.
      if (current_node == this->end_node) {
        // A* complete, we have a path.
        this->m_Model.path = ConstructFinalPath(current_node);
        return;
      }
      // else, continue A* Search.
      this->AddNeighbors(current_node);
    }
}
