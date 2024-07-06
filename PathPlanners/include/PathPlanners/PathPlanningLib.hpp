#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <optional>
#include <bits/stdc++.h>
#include <iostream>
#include <geometry_msgs/msg/point.hpp>

struct Path_Node
{
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point parent;
    int G_cost;
    bool operator<(const Path_Node& other) const
    {
        return G_cost > other.G_cost; // Use '>' for min-heap
    }
};

class PathPlanners
{
public:
    PathPlanners(std::vector<std::vector<std::vector<int>>> map, std::vector<int> start, std::vector<int> goal)
        : map(map), start(start), goal(goal) {}
    virtual std::vector<std::vector<int>> get_map() = 0;

protected:
    std::vector<std::vector<std::vector<int>>> map;
    std::vector<int> start;
    std::vector<int> goal;
};

class Astar : public PathPlanners
{
public:
    Astar(std::vector<std::vector<std::vector<int>>> map, std::vector<int> start, std::vector<int> goal, bool diagonal_traversal = false)
        : PathPlanners(map, start, goal), diagonal_traversal(diagonal_traversal) {}

    std::vector<std::vector<int>> get_map() override {
        print_map();
        return find_path(start[0], start[1], start[2], goal[0], goal[1], goal[2]);
    }

private:
    bool diagonal_traversal;

    std::vector<std::vector<int>> find_path(int source_x, int source_y, int source_z, int dest_x, int dest_y, int dest_z) {
        if (source_z == dest_z) {
            return find_path_on_level_heuristic(source_x, source_y, source_z, dest_x, dest_y, dest_z);
        } else {
            auto start_elevator = find_nearest_elevator(source_x, source_y, source_z);
            if (!start_elevator.first && !start_elevator.second) {
                return {};
            }

            auto source_to_elevator = find_path_on_level_heuristic(source_x, source_y, source_z, start_elevator.first, start_elevator.second, source_z);
            auto elevator_to_dest = find_path_on_level_heuristic(start_elevator.first, start_elevator.second, dest_z, dest_x, dest_y, dest_z);

            if (!source_to_elevator.empty() && !elevator_to_dest.empty()) {
                source_to_elevator.insert(source_to_elevator.end(), elevator_to_dest.begin(), elevator_to_dest.end());
                return source_to_elevator;
            } else {
                std::cout << "Error - 03 : failed to get source to destination, try a different map" << std::endl;
                return {};
            }
        }
    }

    void print_map() {
        std::cout << "Map Contents:" << std::endl;

        for(int z = 0; z < 3; z++) {
            std::cout << "Level " << z << ":" << std::endl;
            for(int x = 0; x < 10; x++) {
                for(int y = 0; y < 10; y++) {
                    std::cout << map[z][x][y] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    }

    std::vector<std::vector<int>> find_path_on_level_heuristic(int source_x, int source_y, int source_z, int dest_x, int dest_y, int dest_z) {
        std::priority_queue<Path_Node> pq;
        std::vector<std::vector<std::vector<bool>>> visited(3, 
            std::vector<std::vector<bool>>(10, 
                std::vector<bool>(10, false)));

        geometry_msgs::msg::Point start_point;
        start_point.x = static_cast<double>(source_x);
        start_point.y = static_cast<double>(source_y);
        start_point.z = static_cast<double>(source_z);

        geometry_msgs::msg::Point parent_point;
        parent_point.x = -1;
        parent_point.y = -1;
        parent_point.z = -1;

        pq.push({start_point, parent_point, 0});

        while (!pq.empty()) {
            Path_Node current = pq.top();
            pq.pop();

            int x = static_cast<int>(current.point.x);
            int y = static_cast<int>(current.point.y);
            int z = static_cast<int>(current.point.z);

            if (x == dest_x && y == dest_y && z == dest_z) {
                return reconstruct_path(current);
            }

            if (visited[z][x][y]) continue;
            visited[z][x][y] = true;

            std::vector<std::pair<int, int>> directions;
            if (diagonal_traversal) {
                directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
            } else {
                directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
            }

            for (const auto& [dx, dy] : directions) {
                int new_x = x + dx;
                int new_y = y + dy;
                if (new_x >= 0 && new_x < 10 && new_y >= 0 && new_y < 10 && map[z][new_x][new_y] != 0) {
                    double heuristic_factor = std::abs(new_x - dest_x) + std::abs(new_y - dest_y);
                    double new_cost = current.G_cost + 1;
                    if (diagonal_traversal && dx != 0 && dy != 0) {
                        new_cost += 0.4;
                    }
                    if (map[z][new_x][new_y] > 1) {
                        heuristic_factor *= map[z][new_x][new_y];
                    }
                    geometry_msgs::msg::Point new_point;
                    new_point.x = static_cast<double>(new_x);
                    new_point.y = static_cast<double>(new_y);
                    new_point.z = static_cast<double>(z);

                    pq.push({new_point, current.point, static_cast<int>(new_cost + heuristic_factor)});
                }
            }
        }

        return {};
    }

    std::pair<int, int> find_nearest_elevator(int x, int y, int z) {
        int min_dist = std::numeric_limits<int>::max();
        std::pair<int, int> nearest_elevator;

        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                if (map[z][i][j] == -1) {
                    int dist = std::abs(i - x) + std::abs(j - y);
                    if (dist < min_dist) {
                        min_dist = dist;
                        nearest_elevator = std::make_pair(i, j);
                    }
                }
            }
        }

        return nearest_elevator;
    }

    std::vector<std::vector<int>> reconstruct_path(const Path_Node& end_node) {
        std::vector<std::vector<int>> path;
        Path_Node current = end_node;

        while (current.parent.x != -1) {
            path.push_back({static_cast<int>(current.point.x), static_cast<int>(current.point.y), static_cast<int>(current.point.z)});
            geometry_msgs::msg::Point parent = current.parent;
            geometry_msgs::msg::Point temp;
            temp.x  = -1;
            temp.y = -1;
            temp.z = -1;
            current = {current.parent, temp, 0};
        }
        path.push_back({static_cast<int>(current.point.x), static_cast<int>(current.point.y), static_cast<int>(current.point.z)});

        std::reverse(path.begin(), path.end());
        return path;
    }
};