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
    std::shared_ptr<Path_Node> parent;
    int G_cost;
    int F_cost;
    bool operator<(const Path_Node& other) const
    {
        return F_cost > other.F_cost;
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
        if (source_x == dest_x) {
            return find_path_on_level_heuristic(source_x, source_y, source_z, dest_x, dest_y, dest_z);
        } else {
            auto start_elevator = find_nearest_elevator(source_x, source_y, source_z);
            if (!start_elevator.first) {
                return {};
            }

            auto source_to_elevator = find_path_on_level_heuristic(source_x, source_y, source_z, source_x, start_elevator.first, start_elevator.second);
            auto elevator_to_dest = find_path_on_level_heuristic(dest_x, start_elevator.first, start_elevator.second, dest_x, dest_y, dest_z);

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

        for(int i=0 ; i<map.size() ; ++i){
            for(int j=0 ; j<map[0].size() ; ++j){
                for(int k=0 ; k<map[0][0].size() ; ++k){
                    std::cout<<map[i][j][k]<<" ";
                }
                std::cout<<std::endl;
            }
            std::cout<<std::endl<<std::endl;
        }
        std::cout<<std::endl;
    }

    std::vector<std::vector<int>> find_path_on_level_heuristic(int source_x, int source_y, int source_z, int dest_x, int dest_y, int dest_z) {
        std::priority_queue<std::shared_ptr<Path_Node>> pq;
        std::vector<std::vector<std::vector<bool>>> visited(map.size(), 
        std::vector<std::vector<bool>>(map[0].size(), 
                std::vector<bool>(map[0][0].size(), false)));

        auto start_node = std::make_shared<Path_Node>();
        start_node->point.x = static_cast<double>(source_x);
        start_node->point.y = static_cast<double>(source_y);
        start_node->point.z = static_cast<double>(source_z);
        start_node->parent = nullptr;
        start_node->G_cost = 0;
        start_node->F_cost = heuristic(source_x, source_y, source_z, dest_x, dest_y, dest_z);

        pq.push(start_node);

        while (!pq.empty()) {
            auto current = pq.top();
            pq.pop();

            int x = static_cast<int>(current->point.x);
            int y = static_cast<int>(current->point.y);
            int z = static_cast<int>(current->point.z);

            if (x == dest_x && y == dest_y && z == dest_z) {
                std::cout << "Goal reached at (" << x << ", " << y << ", " << z << ")" << std::endl;
                auto path = reconstruct_path(current);
                std::cout << "Path length: " << path.size() << std::endl;
                return path;
            }

            if (visited[x][y][z]) continue;
            visited[x][y][z] = true;

            std::vector<std::pair<int, int>> directions;
            if (diagonal_traversal) {
                directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
            } else {
                directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
            }

            for (const auto& [dy, dz] : directions) {
                int new_y = y + dy;
                int new_z = z + dz;
                
                if (new_y >= 0 && new_y < map[0].size() && new_z >= 0 && new_z < map[0][0].size() && map[x][new_y][new_z] != 0) {
                    int new_G_cost = current->G_cost + 1;
                    if (diagonal_traversal && dz != 0 && dy != 0) {
                        new_G_cost += 1;  // Changed from 0.4 to 1 for integer costs
                    }
                    if (map[x][new_y][new_z] > 1) {
                        new_G_cost *= map[x][new_y][new_z];
                    }

                    auto new_node = std::make_shared<Path_Node>();
                    new_node->point.x = static_cast<double>(x);
                    new_node->point.y = static_cast<double>(new_y);
                    new_node->point.z = static_cast<double>(new_z);
                    new_node->parent = current;
                    new_node->G_cost = new_G_cost;
                    new_node->F_cost = new_G_cost + heuristic(x, new_y, new_z, dest_x, dest_y, dest_z);

                    pq.push(new_node);
                }
            }
        }

        return {};
    }

    // Add this heuristic function
    int heuristic(int x1, int y1, int z1, int x2, int y2, int z2) {
        return std::abs(x2 - x1) + std::abs(y2 - y1) + std::abs(z2 - z1);
    }

    std::pair<int, int> find_nearest_elevator(int x, int y, int z) {
        int min_dist = std::numeric_limits<int>::max();
        std::pair<int, int> nearest_elevator;

        for (int i = 0; i < map[0].size(); ++i) {
            for (int j = 0; j < map[0][0].size(); ++j) {
                if (map[x][i][j] == -1) {
                    int dist = std::abs(i - y) + std::abs(j - z);
                    if (dist < min_dist) {
                        min_dist = dist;
                        nearest_elevator = std::make_pair(i, j);
                    }
                }
            }
        }

        return nearest_elevator;
    }

    std::vector<std::vector<int>> reconstruct_path(const std::shared_ptr<Path_Node>& end_node) {
        std::vector<std::vector<int>> path;
        std::shared_ptr<Path_Node> current = end_node;

        while (current != nullptr) {
            path.push_back({static_cast<int>(current->point.x), static_cast<int>(current->point.y), static_cast<int>(current->point.z)});
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};