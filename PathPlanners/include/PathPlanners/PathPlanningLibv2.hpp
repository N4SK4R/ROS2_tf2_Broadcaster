#ifndef PATH_PLANNING_LIB_V2_HPP
#define PATH_PLANNING_LIB_V2_HPP

#include <bits/stdc++.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/msg/point.hpp>


struct Path_Node
{
    geometry_msgs::msg::Point point;
    std::shared_ptr<Path_Node> parent;
    int G_cost;
    int F_cost;
};

struct CompareNode {
    bool operator()(const std::shared_ptr<Path_Node>& a, const std::shared_ptr<Path_Node>& b) const {
        return a->F_cost > b->F_cost;
    }
};

class PathPlanners {
public:
    virtual std::vector<geometry_msgs::msg::Point> get_plan(std::vector<std::vector<std::vector<int>>> map, geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) = 0; // Pure virtual function
};

#endif // PATH_PLANNING_LIB_V2_HPP