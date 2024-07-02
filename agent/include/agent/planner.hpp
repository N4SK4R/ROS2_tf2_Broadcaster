#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <my_robot_interfaces/srv/get_plan.hpp>

#include <vector>
#include <climits>

using std::vector;
using namespace std::placeholders;

struct Path
{
    std::string serial_id;
    double time_of_plan;
    vector<geometry_msgs::msg::Point> point_list;
};


enum Status {FREE, OCCUPIED, START, GOAL};


struct Grid_node
{
    Status stat;                                
    bool is_closed;                             
    int past_cost;                              // G cost
    int total_cost;                             // F cost
    int pos[2];                                 
    geometry_msgs::msg::Point parent;                

    bool operator<(Grid_node other) const       // Comparison function used by A* to sort the nodes currently in the 'open' list
    {
        if (total_cost == other.total_cost)     
        {
            if (pos[0] == other.pos[0])         
            {
                return pos[1] < other.pos[1];   
            }
            return pos[0] < other.pos[0];       
        }
        return total_cost < other.total_cost;   
    }
};

class Motion_Planner
{
public:
    explicit Motion_Planner(std::shared_ptr<rclcpp::Node> node, const int period = 10)
        : node_(node), period_(period)
    {
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
        vector<Grid_node> open;
        Path final_path {};
        final_path.serial_id = serial_id;

        if(collisions.size() == 0){
            return final_path;
        }

        Grid_node grid[10+1][10+1];

        for (int i{0}; i <= 10; i++)
        {
            for (int j{0}; j <= 10; j++)
            {
                grid[i][j].stat = FREE;
                grid[i][j].pos[0] = i;
                grid[i][j].pos[1] = j;
                grid[i][j].past_cost = INT_MAX;
            }
        }

        int start[] = {(int)start_point.x, (int)start_point.y};
        int goal[] = {(int)goal_point.x, (int)goal_point.y};
        grid[start[0]][start[1]].stat = START;
        grid[start[0]][start[1]].past_cost = 0;
        grid[goal[0]][goal[1]].stat = GOAL;
        open.push_back(grid[start[0]][start[1]]);

        int x_nbr_arr[] = {1, 0, -1, 0};
        int y_nbr_arr[] = {0, 1, 0, -1};
        int x_nbr=0, y_nbr=0;

        int x_curr=0, y_curr=0;
        Grid_node curr;


        while (open.size() != 0)
        {
  
        curr = open.at(0);
        x_curr = curr.pos[0];
        y_curr = curr.pos[1];
        open.erase(open.begin());
 
        grid[x_curr][y_curr].is_closed = true;

        if (grid[x_curr][y_curr].stat == GOAL)
        {

            geometry_msgs::msg::Point goal;
            goal.x = x_curr;
            goal.y = y_curr;
            final_path.point_list.push_back(goal);

            while (x_curr != start[0] || y_curr != start[1])
            {
                final_path.point_list.insert(final_path.point_list.begin(), grid[x_curr][y_curr].parent);
                x_curr = final_path.point_list.at(0).x;
                y_curr = final_path.point_list.at(0).y;
            }
            final_path.time_of_plan = node_->now().seconds();
            break;
        }

        // Otherwise, look at the neighboring nodes
        for (size_t i{0}; i < 4; i++)
        {
            x_nbr = x_curr + x_nbr_arr[i];
            y_nbr = y_curr + y_nbr_arr[i];
            
            
            if (x_nbr >= 0 && x_nbr <= 10 && y_nbr >= 0 && y_nbr <= 10)
            {
                if (!grid[x_nbr][y_nbr].is_closed && grid[x_nbr][y_nbr].stat != OCCUPIED)
                {
                    int tentative_past_cost = curr.past_cost + 10;
                    if (tentative_past_cost < grid[x_nbr][y_nbr].past_cost)
                    {
                        grid[x_nbr][y_nbr].past_cost = tentative_past_cost;
                        grid[x_nbr][y_nbr].parent.x = x_curr;
                        grid[x_nbr][y_nbr].parent.y = y_curr;

                        // Calculate the total cost using Manhattan Distance heuristic 
                        grid[x_nbr][y_nbr].total_cost = grid[x_nbr][y_nbr].past_cost + 10*(abs(x_nbr - goal[0]) + abs(y_nbr - goal[1]));
                        open.push_back(grid[x_nbr][y_nbr]);
                    }
                }
            }
        }

        // sort the list of nodes using the function described in the 'Grid_Node' structure
        std::sort(open.begin(), open.end());
        }
        return final_path;
    }


    //geometry_msgs::msg::Point planner_check_collision(const struct Path current_path);


    void planner_get_plan(std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> req ,std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> res)
    {
        geometry_msgs::msg::Point start_point, goal_point;
        goal_point.x = req->goal_pose.position.x;
        goal_point.y = req->goal_pose.position.y;

        start_point.x = 0.0;
        start_point.y = 0.0;

        vector<geometry_msgs::msg::Point> collisions;
        geometry_msgs::msg::Point collision_location;
        Path current_path {};

        RCLCPP_INFO(node_->get_logger(), "Using A* algorithm");

        current_path = planner_plan_path(start_point, goal_point, req->serial_id, collisions);

        res->path = current_path.point_list;
    }


};