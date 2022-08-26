#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <bits/stdc++.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_lib/param_loader.h>
#include <pluginlib/class_list_macros.h>
#include <coverage_planning/CPPServiceCall.h>
// #include <coverage_planning/UpdateMap.h>
#include <coverage_planning/WaypointVector.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
namespace ns_boustrophedon{
    class coverage_planning_node_class : public nodelet::Nodelet{
        public:
        virtual void onInit();
        //ros::NodeHandle nh;
        ros::ServiceClient                      coverage_planning_trajectory_service_client;
        ros::ServiceServer                      CPP_call_file;
        ros::ServiceServer                      curent_area_finder;
        ros::Subscriber                         ground_waypoint_sub;
        class coordinates_node{
            public:
            float coordinates_x;
            float coordinates_y;
            float coordinates_z;
            coordinates_node* next;
            coordinates_node* prev;
        };
        class area_node{
            public:
            coordinates_node* prev_coordinate1;
            coordinates_node* prev_coordinate2;
            coordinates_node* next_coordinate1;
            coordinates_node* next_coordinate2;
            bool concavity = true;
        };
        std::string                             _uav_name_;
        std::vector<area_node*>                 divided_area;
        coordinates_node*                       head = new coordinates_node();
        coordinates_node*                       temp1;
        coordinates_node*                       prev_coordinate1;
        coordinates_node*                       prev_coordinate2;
        coordinates_node*                       next_coordinate1;
        coordinates_node*                       next_coordinate2;  
        bool                                    file_read = false;
        bool                                    sorting_status = false;
        bool                                    area_division_type = false;
        bool                                    concavity = false;
        bool                                    next_boundary = false;
        bool                                    prev_boundary = false;
        bool                                    no_fly_zone = false;
        float                                   abselon =0.5;
        std::string                             _coverage_planning_area_;
        float                                   sweeping_dist;
        mrs_msgs::Reference                     waypoint;
        mrs_msgs::Reference                     ground_waypoint;
        std::vector <coordinates_node*>         concave_points;
        //this is wrong and need to be corrected
        std::vector <geometry_msgs::Point>      NoFlyZoneVector;
        std::vector <coordinates_node*>::iterator concave_points_iterator;
        std::vector<mrs_msgs::Reference>        visited_waypoint_vector;
        void cycle_area_node(area_node* temp_area_node);
        mrs_msgs::PathSrv::Request              Pathreq;
        mrs_msgs::PathSrv::Response             Pathres;
        // coverage_planning::UpdateMap::Request   UpdateMapReq;
        // coverage_planning::UpdateMap::Response  UpdateMapRes;
        
        bool read(coverage_planning::CPPServiceCall::Request& Pathreq, coverage_planning::CPPServiceCall::Response& Pathres);
        void update_map(const coverage_planning::WaypointVector::ConstPtr& msg);
        // bool curent_area_finder(coverage_planning::UpdateMap::Request& UpdateMapReq,  coverage_planning::UpdateMap::Response& UpdateMapRes);
        void coordinate_finder();
        int concavity_indentifier();
        void sorting(std::vector <coordinates_node*>& concave_points);
        void boustrophedon_area_division(coordinates_node* next_coordinate1, coordinates_node* next_coordinate2, coordinates_node* prev_coordinate1, coordinates_node* prev_coordinate2);
        void boustrophedon_matrix();
        coverage_planning_node_class::coordinates_node* extremum (coordinates_node* prev_coordinate, coordinates_node* next_coordinate, bool action);
        float max(float length1, float legth2);
        float dist (coordinates_node* node1, coordinates_node* node2);
        void trajectory_planner(coordinates_node* side_coordinate1, coordinates_node* side_coordinate2);
    };
}