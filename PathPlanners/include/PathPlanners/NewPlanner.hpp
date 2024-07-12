#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/msg/agent_info.hpp>
#include <my_robot_interfaces/srv/get_map.hpp>
#include <my_robot_interfaces/srv/get_plan.hpp>
//#include <PathPlanners/CoreComponents.hpp>
#include <PathPlanners/A_star.hpp>
#include <PathPlanners/Dijkstra.hpp>
#include <my_robot_interfaces/srv/update_map.hpp>


#include <bits/stdc++.h>
using namespace std;

struct Path
{
    std::string serial_id;
    double time_of_plan;
    std::vector<geometry_msgs::msg::Point> point_list;
};


class Path_Planner {
public:
    Path_Planner(std::shared_ptr<rclcpp::Node> node, const int period = 10) : node_(node), period_(period) {
        path_planning_service_ = node_->create_service<my_robot_interfaces::srv::GetPlan>(
            "/get_plan", std::bind(&Path_Planner::planner_get_plan, this, std::placeholders::_1, std::placeholders::_2));
        sub_agent_info_ = node_->create_subscription<my_robot_interfaces::msg::AgentInfo>("/agent_info", 100, std::bind(&Path_Planner::planner_agent_pose_callback, this, std::placeholders::_1));
        update_map_client_ = node_->create_client<my_robot_interfaces::srv::UpdateMap>("/update_map");
        get_map_client_ = node_->create_client<my_robot_interfaces::srv::GetMap>("/get_map");
        

        RCLCPP_INFO(node_->get_logger(), "Motion Planner Service Ready");
        timer_ = node_->create_wall_timer(std::chrono::duration<double>(5.0) , bind(&Path_Planner::init_map, this));
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Service<my_robot_interfaces::srv::GetPlan>::SharedPtr path_planning_service_;
    rclcpp::Client<my_robot_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Client<my_robot_interfaces::srv::UpdateMap>::SharedPtr update_map_client_;
    rclcpp::Subscription<my_robot_interfaces::msg::AgentInfo>::SharedPtr sub_agent_info_;   
    rclcpp::TimerBase::SharedPtr timer_;
    const int period_ = 10;
    vector<vector<vector<int>>> global_map;
    vector<Path> archived_paths; 
    //unordered_map<string,Path> agent_paths_hmap;
    unordered_map<string,my_robot_interfaces::msg::AgentInfo>agent_poses_hmap;
    AStar astar;
    //Dijkstra dijkstra;

    //

    int map_x, map_y, map_z;
    bool map_is_initialized;

    void init_map() {
            RCLCPP_INFO(node_->get_logger(), "Initializing Map");

            auto request = std::make_shared<my_robot_interfaces::srv::GetMap::Request>();
            auto future_result = get_map_client_->async_send_request(request,
                std::bind(&Path_Planner::responseCallback, this, std::placeholders::_1));
            // Set flag here; responseCallback will finalize initialization
    }

    void responseCallback(rclcpp::Client<my_robot_interfaces::srv::GetMap>::SharedFuture future) {
        //RCLCPP_INFO(node_->get_logger(), "Fresh map recieved");
        auto result = future.get();
        map_z = result->map[result->map.size() - 3];
        map_y = result->map[result->map.size() - 2];
        map_x = result->map[result->map.size() - 1]; 

        global_map.resize(map_z, vector<vector<int>>(map_y, vector<int>(map_x)));

        update_map(result->map);
    }

    void update_map(const vector<int>& map) {
        //RCLCPP_INFO(node_->get_logger(), "Updating Map");

        for(int i = 0 ; i < map_z ; i++) {
            for(int j = 0 ; j < map_y ; j++) {
                for(int k = 0 ; k < map_x ; k++) {
                    global_map[i][j][k] = map[i * map_y * map_x + j * map_x + k];
                }
            }
        }
        //RCLCPP_INFO(node_->get_logger(),"Map Dimensions: %d %d %d", map_x, map_y, map_z);
        //print_map();
    }

    void print_map() {
        RCLCPP_INFO(node_->get_logger(), "Map Contents:");

        for(int i = 0 ; i < map_z ; i++) {
            for(int j = 0 ; j < map_y ; j++) {
                for(int k = 0 ; k < map_x ; k++) {
                    std::cout << global_map[i][j][k] << " ";
                }
                std::cout<<std::endl;
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }


    // collision avoider : 

    geometry_msgs::msg::Point planner_check_collision(const struct Path current_path)
    {
        geometry_msgs::msg::Point collision_point;
        collision_point.z = -1;
        collision_point.y = -1;
        collision_point.x = -1;

        for (auto path_obj : archived_paths)
        {
        if (path_obj.serial_id != current_path.serial_id)
        {
            
            if ((current_path.time_of_plan - path_obj.time_of_plan) >= period_)
            {

                for (auto current_path_point : current_path.point_list)
                {
                    if (path_obj.point_list.back().x == current_path_point.x && path_obj.point_list.back().y == current_path_point.y && path_obj.point_list.back().z == current_path_point.z)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Collision Detected at %f %f %f", current_path_point.x, current_path_point.y, current_path_point.z);
                        return current_path_point;
                    }
                }
            }

            else
            {
                for (size_t i=0; i < path_obj.point_list.size(); i++)
                {

                    for (size_t j=0; j < current_path.point_list.size(); j++)
                    {

                        const double EPSILON = 1e-6;
                    if (std::abs(path_obj.point_list.at(i).x - current_path.point_list.at(j).x) < EPSILON &&
                        std::abs(path_obj.point_list.at(i).y - current_path.point_list.at(j).y) < EPSILON &&
                        std::abs(path_obj.point_list.at(i).z - current_path.point_list.at(j).z) < EPSILON)
                        {
                            double time_offset = current_path.time_of_plan - path_obj.time_of_plan;

                            double segments_in_currentPath  = current_path.point_list.size() - 1;
                            double sec_per_seg_current_path = period_/segments_in_currentPath;

                            double segments_in_archived_path = path_obj.point_list.size() - 1;
                            double sec_per_seg_archived_path = period_/segments_in_archived_path;

                            double entry_time_current_path = (j - 1) * sec_per_seg_current_path;
                            double exit_time_current_path  = (j + 1) * sec_per_seg_current_path;

                            double entry_time_archived_path = (i - 1) * sec_per_seg_archived_path - time_offset;
                            double exit_time_archived_path = (i + 1)  * sec_per_seg_archived_path - time_offset;

                            if (!(exit_time_current_path <= entry_time_archived_path || entry_time_current_path >=exit_time_archived_path))
                            {
                                RCLCPP_INFO(node_->get_logger(), "Collision Detected at %f %f %f", current_path.point_list.at(j).x, current_path.point_list.at(j).y, current_path.point_list.at(j).z);
                                return current_path.point_list.at(j);
                            }
                        }
                    }
                }
            }
        }
    }

        return collision_point;
    }


    // callback for getting plan 

    void planner_get_plan(const std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> request,
        std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> response) {

        RCLCPP_INFO(node_->get_logger(), "Plan Request Received");

        if (agent_poses_hmap.find(request->serial_id) == agent_poses_hmap.end()){ 
            RCLCPP_INFO(node_->get_logger(),"%s Does not Exist", request->serial_id.c_str());
            return;
        }

        geometry_msgs::msg::Point start_point,goal_point;

        //goal point init
        goal_point.x = request->goal_pose.position.x;
        goal_point.y = request->goal_pose.position.y;
        goal_point.z = request->goal_pose.position.z;

        //start point init
        start_point.x = agent_poses_hmap[request->serial_id].pose.position.x;
        start_point.y = agent_poses_hmap[request->serial_id].pose.position.y;
        start_point.z = agent_poses_hmap[request->serial_id].pose.position.z;

        //unassign the old path from the global map 
        for(int i=0 ; i<static_cast<int>(archived_paths.size()) ; ++i){
            if(archived_paths[i].serial_id == request->serial_id){
                //if(archived_paths[i].point_list.size() == 1) break; // if the path is just the agent's current position

                auto new_global_map = this->generate_new_map(archived_paths[i].point_list , false);
                this->global_map = new_global_map;
                RCLCPP_INFO(node_->get_logger(), "Unassigning Old Path");

                this->publish_new_map(pre_process_map());
                archived_paths.erase(archived_paths.begin() + i);
            }
            else{
                if(archived_paths[i].point_list.back() == goal_point){
                    RCLCPP_INFO(node_->get_logger(), "Goal Point is already occupied by another agent");
                    response->path = {start_point, start_point};
                    return;
                }
            }
        }

        // main planner 
        geometry_msgs::msg::Point collision_location;
        Path current_path;
        current_path.serial_id = request->serial_id;
        auto new_tempo_map = global_map;
        int tries = 10;

        // check for collision
        do{
            tries--;
            current_path.point_list = astar.get_plan(new_tempo_map,start_point,goal_point);
            current_path.time_of_plan = node_->now().seconds();
            collision_location = planner_check_collision(current_path);
            if(collision_location.x != -1)
            {
                new_tempo_map[collision_location.z][collision_location.y][collision_location.x] = 0;
            }
            if(collision_location == goal_point || current_path.point_list.size() == 0){
                RCLCPP_INFO(node_->get_logger(), "Goal unreachable as there is an object at the destination");
                response->path = {start_point, start_point};
                return;
            }

        }while(collision_location.x != -1 && tries > 0);

        // if collision is not found even after 10 tries
        if(collision_location.x != -1){
            RCLCPP_INFO(node_->get_logger(), "Collision Detected at %f %f %f and it cannot be resolved !", collision_location.x, collision_location.y, collision_location.z);
            return;
        }

        response->path = current_path.point_list;

        archived_paths.push_back(current_path);

        for(auto point : current_path.point_list){
            RCLCPP_INFO(node_->get_logger(), "Path Point: %f %f %f", point.z, point.y, point.x);
        }

        // old planner v1

        // std::vector<int> goal = {
        //     static_cast<int>(request->goal_pose.position.x),
        //     static_cast<int>(request->goal_pose.position.y),
        //     static_cast<int>(request->goal_pose.position.z)
        // };
        // std::vector<int> start = {0,0,0};

        // RCLCPP_INFO(node_->get_logger(), "Preparing and shipping response");

        // for(auto point : path) {
        //     geometry_msgs::msg::Point p;
        //     // print the messages onto the terminal
        //     RCLCPP_INFO(node_->get_logger(), "Path Point: %d %d %d", point[0], point[1], point[2]);
        //     p.x = point[0];
        //     p.y = point[1];
        //     p.z = point[2];
        //     response->path.push_back(p);
        // }


        // we finally update the map with the new map 
        global_map = this->generate_new_map(current_path.point_list , true);
        this->publish_new_map(pre_process_map());
    }


    // callback for updating the map

    void publish_new_map(std::vector<int> new_map) {
        RCLCPP_INFO(node_->get_logger(), "Publishing new map onto map server");
        auto request = std::make_shared<my_robot_interfaces::srv::UpdateMap::Request>();
        request->map = new_map;

        auto future = update_map_client_->async_send_request(request);
    }


    std::vector<vector<vector<int>>> generate_new_map(std::vector<geometry_msgs::msg::Point> &path , bool add_path = true) {
        RCLCPP_INFO(node_->get_logger(), "Generating new map");
        auto new_global_map = global_map;
        if(add_path){
            for(auto point : path) {
                if(new_global_map[point.z][point.y][point.x] >= 1)
                    new_global_map[point.z][point.y][point.x] += 1;
            }
        
        }
        else{
            for(auto point : path) {
                if(new_global_map[point.z][point.y][point.x] > 1)
                    new_global_map[point.z][point.y][point.x] -= 1;
            }
        
        }
        return new_global_map;
    }

    vector<int> pre_process_map(){

        std::vector<int> new_map;
        for(int i=0 ; i<map_z ; ++i){
            for(int j = 0 ; j<map_y ; ++j){
                for(int k = 0 ; k<map_x ; ++k){
                    new_map.push_back(this->global_map[i][j][k]);
                }
            }
        }

        // add in the dimensions of the map at the end 
        new_map.push_back(map_z);
        new_map.push_back(map_y);
        new_map.push_back(map_x);
        
        return new_map;
    }

    void print_Msg(std::shared_ptr<my_robot_interfaces::srv::UpdateMap::Response> response){
        RCLCPP_INFO(node_->get_logger(), "Map Update Result: %d", response->result);
    }

    void planner_agent_pose_callback(my_robot_interfaces::msg::AgentInfo msg)
    {   

        msg.pose.position.x = round(msg.pose.position.x);
        msg.pose.position.y = round(msg.pose.position.y);
        msg.pose.position.z = round(msg.pose.position.z);

        //check in hmap for the serial_id 
        if(agent_poses_hmap.find(msg.serial_id) != agent_poses_hmap.end()){
            agent_poses_hmap[msg.serial_id] = msg;
        }
        else{
            agent_poses_hmap[msg.serial_id] = msg;
            // add the agent's position into the archived paths 
            Path new_agent_path;

            geometry_msgs::msg::Point new_point;

            new_agent_path.serial_id = msg.serial_id;
            new_agent_path.time_of_plan = node_->now().seconds();
            new_point.x = msg.pose.position.x;
            new_point.y = msg.pose.position.y;
            new_point.z = msg.pose.position.z;

            new_agent_path.point_list.push_back(new_point);
            archived_paths.push_back(new_agent_path);

        }
    }

};


