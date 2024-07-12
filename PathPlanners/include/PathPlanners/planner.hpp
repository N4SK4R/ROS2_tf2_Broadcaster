// #include <rclcpp/rclcpp.hpp>

// #include <visualization_msgs/msg/marker.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/point.hpp>

// #include <my_robot_interfaces/srv/get_plan.hpp>
// #include <my_robot_interfaces/msg/agent_info.hpp>

// #include <vector>
// #include <climits>

// using std::vector;
// using namespace std::placeholders;

// struct Path
// {
//     std::string serial_id;
//     double time_of_plan;
//     vector<geometry_msgs::msg::Point> point_list;
// };


// enum Status {FREE, OCCUPIED, START, GOAL};


// struct Grid_node
// {
//     Status stat;                                
//     bool is_closed;                             
//     int G_cost;                              
//     int F_cost;    // F=G+H
//     int pos[2];                                 
//     geometry_msgs::msg::Point parent;                

//     bool operator<(Grid_node other) const       // Comparison function used by A* to sort the nodes currently in the 'open_list' list
//     {
//         if (F_cost == other.F_cost)     
//         {
//             if (pos[0] == other.pos[0])         
//             {
//                 return pos[1] < other.pos[1];   
//             }
//             return pos[0] < other.pos[0];       
//         }
//         return F_cost < other.F_cost;   
//     }
// };

// class Motion_Planner
// {
// public:
//     explicit Motion_Planner(std::shared_ptr<rclcpp::Node> node, const int period = 10)
//         : node_(node), period(period)
//     {
//         service_ = node_->create_service<my_robot_interfaces::srv::GetPlan>("/get_plan", std::bind(&Motion_Planner::planner_get_plan,this,_1,_2));
//         sub_agent_info_ = node_->create_subscription<my_robot_interfaces::msg::AgentInfo>("/agent_info", 100, std::bind(&Motion_Planner::planner_agent_pose_callback, this, _1));
//         RCLCPP_INFO(node_->get_logger(), "Motion Planner Service Ready");
//     }
// private:

//     std::shared_ptr<rclcpp::Node> node_;                                                                       
//     rclcpp::Service<my_robot_interfaces::srv::GetPlan>::SharedPtr service_;   
//     rclcpp::Subscription<my_robot_interfaces::msg::AgentInfo>::SharedPtr sub_agent_info_;                            

//     const int period;                                               
//     vector<Path> archived_paths;                                    
//     vector<my_robot_interfaces::msg::AgentInfo> agent_poses;      

 
//     struct Path planner_plan_path(const geometry_msgs::msg::Point start_point, const geometry_msgs::msg::Point goal_point, const std::string serial_id, const vector<geometry_msgs::msg::Point> collisions)
//     {
//         vector<Grid_node> open_list;
//         Path final_path;
//         final_path.serial_id = serial_id;

//         Grid_node grid[10+1][10+1];

//         for (int i=0; i <= 10; i++)
//         {
//             for (int j=0; j <= 10; j++)
//             {
//                 grid[i][j].stat = FREE;
//                 grid[i][j].pos[0] = i;
//                 grid[i][j].pos[1] = j;
//                 grid[i][j].G_cost = INT_MAX;
//             }
//         }

//         for (auto p : collisions)grid[(int)p.x][(int)p.y].stat = OCCUPIED;
    
//         int start[] = {(int)start_point.x, (int)start_point.y};
//         int goal[] = {(int)goal_point.x, (int)goal_point.y};

//         grid[start[0]][start[1]].stat = START;
//         grid[start[0]][start[1]].G_cost = 0;
//         grid[goal[0]][goal[1]].stat = GOAL;

//         open_list.push_back(grid[start[0]][start[1]]);

//         int x_nbr_arr[] = {1, 0, -1, 0};
//         int y_nbr_arr[] = {0, 1, 0, -1};
//         int x_nbr=0, y_nbr=0;

//         int x_current=0, y_current=0;
//         Grid_node current;


//         while (open_list.size() != 0)
//         {
  
//         current = open_list.at(0);
//         x_current = current.pos[0];
//         y_current = current.pos[1];
//         open_list.erase(open_list.begin());
 
//         grid[x_current][y_current].is_closed = true;

//         if (grid[x_current][y_current].stat == GOAL)
//         {

//             geometry_msgs::msg::Point goal;
//             goal.x = x_current;
//             goal.y = y_current;
//             final_path.point_list.push_back(goal);

//             while (x_current != start[0] || y_current != start[1])
//             {
//                 final_path.point_list.insert(final_path.point_list.begin(), grid[x_current][y_current].parent);
//                 x_current = final_path.point_list.at(0).x;
//                 y_current = final_path.point_list.at(0).y;
//             }
//             // the timestamp is added to the final_path object
//             final_path.time_of_plan = node_->now().seconds();
//             break;
//         }

//         // Otherwise, look at the neighboring nodes
//         for (size_t i{0}; i < 4; i++)
//         {
//             x_nbr = x_current + x_nbr_arr[i];
//             y_nbr = y_current + y_nbr_arr[i];
            
            
//             if (x_nbr >= 0 && x_nbr <= 10 && y_nbr >= 0 && y_nbr <= 10)
//             {
//                 if (!grid[x_nbr][y_nbr].is_closed && grid[x_nbr][y_nbr].stat != OCCUPIED)
//                 {
//                     int tentative_G_cost = current.G_cost + 10;
//                     if (tentative_G_cost < grid[x_nbr][y_nbr].G_cost)
//                     {
//                         grid[x_nbr][y_nbr].G_cost = tentative_G_cost;
//                         grid[x_nbr][y_nbr].parent.x = x_current;
//                         grid[x_nbr][y_nbr].parent.y = y_current;

//                         // Calculate the total cost using Manhattan Distance heuristic 
//                         grid[x_nbr][y_nbr].F_cost = grid[x_nbr][y_nbr].G_cost + 10*(abs(x_nbr - goal[0]) + abs(y_nbr - goal[1]));
//                         open_list.push_back(grid[x_nbr][y_nbr]);
//                     }
//                 }
//             }
//         }

//         // sort the list of nodes using the function described in the 'Grid_Node' structure
//         std::sort(open_list.begin(), open_list.end());
//         }
//         return final_path;
//     }


//     geometry_msgs::msg::Point planner_check_collision(const struct Path current_path)
//     {
//         geometry_msgs::msg::Point collision_point;
//         collision_point.x = -1;
//         collision_point.y = -1;

//         for (auto path_obj : archived_paths)
//         {
//         if (path_obj.serial_id != current_path.serial_id)
//         {
            
//             if ((current_path.time_of_plan - path_obj.time_of_plan) >= period)
//             {

//                 for (auto current_path_point : current_path.point_list)
//                 {
//                     if (path_obj.point_list.back().x == current_path_point.x && path_obj.point_list.back().y == current_path_point.y)
//                     {
//                         return current_path_point;
//                     }
//                 }
//             }

//             else
//             {
//                 for (size_t i=0; i < path_obj.point_list.size(); i++)
//                 {

//                     for (size_t j=0; j < current_path.point_list.size(); j++)
//                     {

//                         if (path_obj.point_list.at(i).x == current_path.point_list.at(j).x && path_obj.point_list.at(i).y == current_path.point_list.at(j).y)
//                         {
//                             double time_offset = current_path.time_of_plan - path_obj.time_of_plan;

//                             double segments_in_currentPath  = current_path.point_list.size() - 1;
//                             double sec_per_seg_current_path = period/segments_in_currentPath;

//                             double segments_in_archived_path = path_obj.point_list.size() - 1;
//                             double sec_per_seg_archived_path = period/segments_in_archived_path;

//                             double entry_time_current_path = (j - 1) * sec_per_seg_current_path;
//                             double exit_time_current_path  = (j + 1) * sec_per_seg_current_path;

//                             double entry_time_archived_path = (i - 1) * sec_per_seg_archived_path - time_offset;
//                             double exit_time_archived_path = (i + 1)  * sec_per_seg_archived_path - time_offset;

//                             if (!(exit_time_current_path <= entry_time_archived_path || entry_time_current_path >=exit_time_archived_path))
//                             {
//                                 return current_path.point_list.at(j);
//                             }
//                         }
//                     }
//                 }
//             }
//         }
//     }

//         return collision_point;
//     }


//     void planner_get_plan(std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> req ,std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> res)
//     {
//         geometry_msgs::msg::Point start_point,goal_point;
//         goal_point.x = req->goal_pose.position.x;
//         goal_point.y = req->goal_pose.position.y;

//         bool found = false;
//         for (auto agent : agent_poses)
//         {
//             if (agent.serial_id == req->serial_id)
//             {
//                 start_point.x = agent.pose.position.x;
//                 start_point.y = agent.pose.position.y;
//                 found = true;
//                 break;
//             }
//         }

//         if (!found)RCLCPP_INFO(node_->get_logger(),"%s Does not Exist", req->serial_id.c_str());

//         vector<geometry_msgs::msg::Point> collisions;
//         geometry_msgs::msg::Point collision_location;
//         Path current_path;

//         //do{
//         current_path = planner_plan_path(start_point, goal_point, req->serial_id, collisions);

//         collision_location = planner_check_collision(current_path);

//         RCLCPP_INFO(node_->get_logger(),"Collsion Location (%f,%f)", collision_location.x,collision_location.y);

//             //collisions.push_back(collision_location);

//         //} while(collision_location.x != -1 && collision_location.y != -1 && current_path.point_list.size() != 0);

//         res->path = current_path.point_list;

//         for (auto path_obj : archived_paths)
//         {
//             if (path_obj.serial_id == current_path.serial_id)
//             {
//                 path_obj.time_of_plan = current_path.time_of_plan;
//                 path_obj.point_list = current_path.point_list;
//             }
//         }
//     }

//     void planner_agent_pose_callback(my_robot_interfaces::msg::AgentInfo msg)
//     {   

//         bool found = false;

//         msg.pose.position.x = round(msg.pose.position.x);
//         msg.pose.position.y = round(msg.pose.position.y);

//         for (int i=0; i < int(agent_poses.size()); i++)
//         {
//             if (agent_poses.at(i).serial_id == msg.serial_id)
//             {
//                 agent_poses.at(i) = msg;
//                 found = true;
//                 break;
//             }
//         }

//         if (!found)
//         {
//             agent_poses.push_back(msg);

//             Path new_agent_path;
//             geometry_msgs::msg::Point new_point;

//             new_agent_path.serial_id = msg.serial_id;
//             new_agent_path.time_of_plan = node_->now().seconds();
//             new_point.x = msg.pose.position.x;
//             new_point.y = msg.pose.position.y;

//             new_agent_path.point_list.push_back(new_point);
//             archived_paths.push_back(new_agent_path);

//         }
//     }

// };