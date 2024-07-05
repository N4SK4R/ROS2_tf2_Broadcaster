#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <vector>
#include <climits>


enum Status {FREE, OCCUPIED, START, GOAL};

struct Path
{
    std::string serial_id;
    double time_of_plan;
    std::vector<geometry_msgs::msg::Point> point_list;
};

struct Grid_node
{
    Status stat;                                
    bool is_closed;                             
    int G_cost;                              
    int F_cost;    // F=G+H
    int pos[2];                                 
    geometry_msgs::msg::Point parent;                

    bool operator<(Grid_node other) const       // Comparison function used by A* to sort the nodes currentently in the 'open_list' list
    {
        if (F_cost == other.F_cost)     
        {
            if (pos[0] == other.pos[0])         
            {
                return pos[1] < other.pos[1];   
            }
            return pos[0] < other.pos[0];       
        }
        return F_cost < other.F_cost;   
    }
};

struct Path_Node
{
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point parent;
    int G_cost;
    bool operator<(Grid_node other) const       // Comparison function used by A* to sort the nodes currentently in the 'open_list' list
    {
        return G_cost < other.G_cost;
    }
};
