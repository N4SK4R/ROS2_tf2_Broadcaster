#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <bits/stdc++.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/msg/point.hpp>
#include <PathPlanners/PathPlanningLibv2.hpp>


class Dijkstra : public PathPlanners {
public:
    Dijkstra() {
        std::cout << "Dijkstra obj init" << std::endl;
    }

    std::vector<geometry_msgs::msg::Point> get_plan(std::vector<std::vector<std::vector<int>>> map, geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) override {
        this->map = map;
        this->goal = goal;
        this->start = start;
        this->diagonal_traversal = false; // Set diagonal traversal if needed
        this->print_map();
        return find_path(start, goal);
    }

private:
    std::vector<std::vector<std::vector<int>>> map;
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point goal;
    bool diagonal_traversal;

    std::vector<geometry_msgs::msg::Point> find_path(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        if (start.z == goal.z) {
            return find_path_on_level_heuristic(start, goal);
        } else {
            geometry_msgs::msg::Point start_elevator = find_nearest_elevator(start);

            if (start_elevator.y == -1)
                return {};

            geometry_msgs::msg::Point dest_elevator = start_elevator;
            dest_elevator.z = goal.z;

            auto source_to_elevator = find_path_on_level_heuristic(start, start_elevator);
            auto elevator_to_dest = find_path_on_level_heuristic(dest_elevator, goal);

            if (!source_to_elevator.empty() && !elevator_to_dest.empty()) {
                source_to_elevator.insert(source_to_elevator.end(), elevator_to_dest.begin(), elevator_to_dest.end());
                return source_to_elevator;
            } else {
                std::cout << "Error - 03 : failed to get source to destination, try a different map" << std::endl;
                return {};
            }
        }
    }

    std::vector<geometry_msgs::msg::Point> find_path_on_level_heuristic(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        std::priority_queue<std::shared_ptr<Path_Node>, std::vector<std::shared_ptr<Path_Node>>, CompareNode> pq;
        std::vector<std::vector<std::vector<bool>>> visited(map.size(), std::vector<std::vector<bool>>(map[0].size(), std::vector<bool>(map[0][0].size(), false)));

        auto start_node = std::make_shared<Path_Node>();
        start_node->point = start;
        start_node->parent = nullptr;
        start_node->G_cost = 0; // Initialize G_cost to 0 for the start node

        pq.push(start_node);

        while (!pq.empty()) {
            auto current = pq.top();
            pq.pop();

            int x = static_cast<int>(current->point.x);
            int y = static_cast<int>(current->point.y);
            int z = static_cast<int>(current->point.z);

            if (current->point == goal) {
                std::cout << "Goal Reached at (" << goal.x << " " << goal.y << " " << goal.z << ")" << std::endl;
                auto path = reconstruct_path(current);
                std::cout << "path length: " << path.size() << std::endl;
                return path;
            }

            if (visited[x][y][z]) {
                continue;
            }
            visited[x][y][z] = true;

            std::vector<std::pair<int, int>> directions;
            if (this->diagonal_traversal) {
                directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
            } else {
                directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
            }

            for (const auto& [dy, dx] : directions) {
                int new_y = y + dy;
                int new_x = x + dx;

                if (new_y >= 0 && new_y < static_cast<int>(map[0].size()) && new_x >= 0 && new_x < static_cast<int>(map[0][0].size()) && map[z][new_y][new_x] != 0) {
                    int new_G_cost = current->G_cost + 1; // Assuming each step has a cost of 1

                    if (diagonal_traversal && dx != 0 && dy != 0) {
                        new_G_cost += 1; // Additional cost for diagonal movements if enabled
                    }

                    if (map[z][new_y][new_x] > 1) {
                        new_G_cost *= map[x][new_y][new_x]; // Adjust cost if there are specific node costs
                    }

                    auto new_node = std::make_shared<Path_Node>();
                    new_node->point.z = static_cast<double>(z);
                    new_node->point.y = static_cast<double>(new_y);
                    new_node->point.x = static_cast<double>(new_x);
                    new_node->parent = current;
                    new_node->G_cost = new_G_cost;

                    pq.push(new_node);
                }
            }
        }

        return {};
    }


    geometry_msgs::msg::Point find_nearest_elevator(geometry_msgs::msg::Point start) {
        int min_dist = INT_MAX;
        geometry_msgs::msg::Point elev;
        elev.z = start.z;
        elev.y = -1;
        elev.x = -1;

        for (int i = 0; i < static_cast<int>(map[0].size()); ++i) {
            for (int j = 0; j < static_cast<int>(map[0][0].size()); ++j) {
                if (map[start.z][i][j] == -1) {
                    int dist = std::abs(i - start.y) + std::abs(start.x - j);
                    if (min_dist > dist) {
                        min_dist = dist;
                        elev.y = i;
                        elev.x = j;
                    }
                }
            }
        }
        return elev;
    }

    int heuristic(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        return std::abs(start.x - goal.x) + std::abs(start.y - goal.y) + std::abs(start.z - goal.z);
    }

    std::vector<geometry_msgs::msg::Point> reconstruct_path(const std::shared_ptr<Path_Node>& end_node) {
        std::vector<geometry_msgs::msg::Point> path;
        std::shared_ptr<Path_Node> current = end_node;

        while (current != nullptr) {
            path.push_back(current->point);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void print_map(){
        for(int i=0 ; i<static_cast<int>(map.size()) ; ++i){
            for(int j = 0 ; j<static_cast<int>(map[0].size()) ; ++j){
                for(int k = 0 ; k<static_cast<int>(map[0][0].size()) ; ++k){
                    std::cout<<map[i][j][k]<<" ";
                }
                std::cout<<std::endl;
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }
};

#endif // DIJKSTRA_HPP