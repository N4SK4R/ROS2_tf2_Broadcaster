#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <my_robot_interfaces/srv/get_plan.hpp>
#include "PathPlanners/CoreComponents.hpp"

#include <vector>
#include <climits>

using std::vector;
using namespace std::placeholders;


class Motion_Planner
{
public:
    explicit Motion_Planner(std::shared_ptr<rclcpp::Node> node, const int period = 10)
        : node_(node), period_(period)
    {
        // RCLCPP_INFO(node_->get_logger(), "This is %d", PI);
        service_ = node_->create_service<my_robot_interfaces::srv::GetPlan>("/get_plan", std::bind(&Motion_Planner::planner_get_plan,this,_1,_2));
        RCLCPP_INFO(node_->get_logger(), "Motion Planner Service Ready");
    }
private:

    std::shared_ptr<rclcpp::Node> node_;                                                                       
    rclcpp::Service<my_robot_interfaces::srv::GetPlan>::SharedPtr service_;                               

    const int period_;                                               // Time in seconds for the agent to traverse the whole path - defaults to 10 seconds
    vector<Path> archived_paths;                                    
    //vector<multi_agent_planner::agent_info> agent_start_poses;      // Holds the most up-to-date pose of each agent

 
    struct Path planner_plan_path(const geometry_msgs::msg::Point start_point, const geometry_msgs::msg::Point goal_point, const std::string serial_id, const vector<geometry_msgs::msg::Point> collisions)
    {
        vector<Grid_node> open_list;
        Path final_path {};
        final_path.serial_id = serial_id;

        Grid_node grid[10+1][10+1];

        for (int i=0; i <= 10; i++)
        {
            for (int j=0; j <= 10; j++)
            {
                grid[i][j].stat = FREE;
                grid[i][j].pos[0] = i;
                grid[i][j].pos[1] = j;
                grid[i][j].G_cost = INT_MAX;
            }
        }

        int start[] = {(int)start_point.x, (int)start_point.y};
        int goal[] = {(int)goal_point.x, (int)goal_point.y};

        grid[start[0]][start[1]].stat = START;
        grid[start[0]][start[1]].G_cost = 0;
        grid[goal[0]][goal[1]].stat = GOAL;

        open_list.push_back(grid[start[0]][start[1]]);

        int x_nbr_arr[] = {1, 0, -1, 0};
        int y_nbr_arr[] = {0, 1, 0, -1};
        int x_nbr=0, y_nbr=0;

        int x_current=0, y_current=0;
        Grid_node current;


        while (open_list.size() != 0)
        {
  
        current = open_list.at(0);
        x_current = current.pos[0];
        y_current = current.pos[1];
        open_list.erase(open_list.begin());
 
        grid[x_current][y_current].is_closed = true;

        if (grid[x_current][y_current].stat == GOAL)
        {

            geometry_msgs::msg::Point goal;
            goal.x = x_current;
            goal.y = y_current;
            final_path.point_list.push_back(goal);

            while (x_current != start[0] || y_current != start[1])
            {
                final_path.point_list.insert(final_path.point_list.begin(), grid[x_current][y_current].parent);
                x_current = final_path.point_list.at(0).x;
                y_current = final_path.point_list.at(0).y;
            }
            final_path.time_of_plan = node_->now().seconds();
            break;
        }

        // Otherwise, look at the neighboring nodes
        for (size_t i{0}; i < 4; i++)
        {
            x_nbr = x_current + x_nbr_arr[i];
            y_nbr = y_current + y_nbr_arr[i];
            
            
            if (x_nbr >= 0 && x_nbr <= 10 && y_nbr >= 0 && y_nbr <= 10)
            {
                if (!grid[x_nbr][y_nbr].is_closed && grid[x_nbr][y_nbr].stat != OCCUPIED)
                {
                    int tentative_G_cost = current.G_cost + 10;
                    if (tentative_G_cost < grid[x_nbr][y_nbr].G_cost)
                    {
                        grid[x_nbr][y_nbr].G_cost = tentative_G_cost;
                        grid[x_nbr][y_nbr].parent.x = x_current;
                        grid[x_nbr][y_nbr].parent.y = y_current;

                        // Calculate the total cost using Manhattan Distance heuristic 
                        grid[x_nbr][y_nbr].F_cost = grid[x_nbr][y_nbr].G_cost + 10*(abs(x_nbr - goal[0]) + abs(y_nbr - goal[1]));
                        open_list.push_back(grid[x_nbr][y_nbr]);
                    }
                }
            }
        }

        // sort the list of nodes using the function described in the 'Grid_Node' structure
        std::sort(open_list.begin(), open_list.end());
        }
        return final_path;
    }


    //geometry_msgs::msg::Point planner_check_collision(const struct Path currentent_path);


    void planner_get_plan(std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> req ,std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> res)
    {
        geometry_msgs::msg::Point start_point, goal_point;
        goal_point.x = req->goal_pose.position.x;
        goal_point.y = req->goal_pose.position.y;

        start_point.x = 0.0;
        start_point.y = 0.0;

        vector<geometry_msgs::msg::Point> collisions;
        geometry_msgs::msg::Point collision_location;
        Path currentent_path {};

        RCLCPP_INFO(node_->get_logger(), "Using A* algorithm");

        currentent_path = planner_plan_path(start_point, goal_point, req->serial_id, collisions);

        res->path = currentent_path.point_list;
    }


};