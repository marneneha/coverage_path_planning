#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h> 
#include <nodelet/nodelet.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/SetBool.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/RtkGps.h>
#include <mrs_msgs/UavState.h>
#include <mrs_lib/param_loader.h>
#include <std_srvs/Trigger.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_lib/transformer.h>
#include <nav_msgs/Odometry.h>

namespace ns_wrapper{
	class coverage_planning_wrapper_class : public nodelet::Nodelet{
		public:
		virtual void onInit();
        ros::ServiceClient                  motor_client;
        ros::ServiceClient                  arm_client;
        ros::ServiceClient                  set_mode_client;
        ros::ServiceClient                  goto_client;
        ros::ServiceClient                  takeoff_client;
        ros::ServiceClient                  m_control_manager_stop_following_service_client;
        ros::ServiceClient                  m_trajectory_generator_service_client;
        ros::ServiceClient                  m_control_manager_start_following_service_client;
        ros::ServiceServer                  m_service_server_follow_path;
        std::vector<mrs_msgs::Reference>    m_points_to_follow;
        mrs_msgs::Path                      m_path_message_template;
        std_srvs::SetBool					motor_request;
        mavros_msgs::CommandBool 			arm_request;
        mavros_msgs::SetMode 				srv_setMode;
        std_srvs::Trigger					srv_takeoff;
        mrs_msgs::Vec4						goto_request;
        mrs_msgs::PathSrv::Request          req;
        bool                                takeoff_status=false;
        int                                 m_sequence_counter = 0;
        mrs_lib::Transformer m_transformer;
		void takeoff();
		void goal(boost::array<double,4> new_waypoint);
		bool callback_goal(mrs_msgs::PathSrv::Request &req, mrs_msgs::PathSrv::Response &res);
        void add_heading_to_path(mrs_msgs::Path &path);
        void update_path_message_template(const mrs_msgs::PathSrv::Request &req);

	};
}