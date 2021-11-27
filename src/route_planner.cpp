#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto n : current_node->neighbors) {
    n->parent = current_node;
    n->g_value = current_node->g_value + n->distance(*current_node);
    n->h_value = CalculateHValue(n);
    n->visited = true;
    open_list.push_back(n);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {

  std::sort(open_list.begin(), open_list.end(),
            [](RouteModel::Node *a, RouteModel::Node *b) {
              float f1 = a->g_value + a->h_value;
              float f2 = b->g_value + b->h_value;
              return f1 > f2;
            });
  RouteModel::Node *Nptr = open_list.back();
  open_list.pop_back();
  return Nptr;
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (start_node != current_node) {
    distance += current_node->distance(*current_node->parent);
    path_found.push_back(*current_node);
    current_node = current_node->parent;
  }
  path_found.push_back(*current_node);

  // std::sort(path_found.begin(),path_found.end(),[](RouteModel::Node a ,
  // RouteModel::Node b ){
  //     return a.h_value < b.h_value;
  // });
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  // TODO: Implement your solution here.
  start_node->visited = true;
  open_list.push_back(start_node);
  while (open_list.size() > 0) {
    current_node = NextNode();
    if (current_node == end_node) {
      m_Model.path = ConstructFinalPath(current_node);
      std::cout << m_Model.path.size();
      break;
    }
    AddNeighbors(current_node);
  }
}