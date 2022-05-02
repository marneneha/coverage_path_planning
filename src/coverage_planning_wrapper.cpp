#include<coverage_planning_wrapper.h>
#include <pluginlib/class_list_macros.h>


namespace ns_wrapper{
void coverage_planning_wrapper_class::onInit(){
		ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
		mrs_lib::ParamLoader param_loader(nh, "coverage_planning_wrapper");
		//param_loader.loadParam("simulation", _simulation_);
		//param_loader.loadParam("frame_id", _frame_id_);		
		std::string motor_service_name = "/uav1/control_manager/motors";
		motor_client = (nh.serviceClient<std_srvs::SetBool>(motor_service_name));
		std::string arm_service_name = "/uav1/mavros/cmd/arming";
		arm_client = (nh.serviceClient<mavros_msgs::CommandBool>(arm_service_name));
		std::string takeoff_service_name = "/uav1/uav_manager/takeoff";
		takeoff_client = (nh.serviceClient<std_srvs::Trigger>(takeoff_service_name));
		std::string mode_service_name = "/uav1/mavros/set_mode";
		set_mode_client = (nh.serviceClient<mavros_msgs::SetMode>(mode_service_name));
		m_control_manager_stop_following_service_client = nh.serviceClient<std_srvs::Trigger>("/uav1/control_manager/stop_trajectory_tracking");
        m_control_manager_start_following_service_client = nh.serviceClient<std_srvs::Trigger>("/uav1/control_manager/start_trajectory_tracking");
	    m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>("/uav1/trajectory_generation/path");
	    m_service_server_follow_path = nh.advertiseService("/uav1/coverage_planning/path_to_follow", &coverage_planning_wrapper_class::callback_goal,this);
	    m_transformer = mrs_lib::Transformer("coverage_planning_wrapper_class");
        m_transformer.setDefaultPrefix("uav1");	
        takeoff();
	}

bool coverage_planning_wrapper_class::callback_goal(mrs_msgs::PathSrv::Request &req, mrs_msgs::PathSrv::Response &res){
	if(takeoff_status){
		//send trajectory
		res.success = false;
        auto transform = m_transformer.getTransform(req.path.header.frame_id, "gps_origin");
		auto path_transformed = req.path.points;
		//transforming each point
        for (auto &p: path_transformed) {
            mrs_msgs::ReferenceStamped ref;
            ref.reference = p;
            ref.header.frame_id = req.path.header.frame_id;

            geometry_msgs::Point point = p.position;
            double altitude = point.z;

            auto points_transformed = m_transformer.transform(point, transform.value());
            if (!points_transformed.has_value()) {
                ROS_ERROR("[PathFollower]: Error. Could not transform path");
                res.message = "Error. Could not transform path";
                return true;
            }

            p.position = points_transformed.value();
            if (req.path.header.frame_id == "latlon_origin") {
                p.position.z = altitude;
            }
        }
		//to final sending function
		update_path_message_template(req);
        req.path.header.seq = m_sequence_counter++;
        req.path.header.frame_id = "gps_origin";
        req.path.points = path_transformed;
        add_heading_to_path(req.path);

		std_srvs::Trigger trigger;
        m_control_manager_stop_following_service_client.call(trigger);
	    req.path.fly_now = false;
        m_trajectory_generator_service_client.call(req, res);
        m_points_to_follow.clear();
        if (!res.success) {
            ROS_ERROR_STREAM("[PathFollower]: Error while calling trajectory generation. Message: " << res.message);
        } else {
            std_srvs::Trigger trigger_start_tracking;
            if (!m_control_manager_start_following_service_client.call(trigger_start_tracking) || !trigger_start_tracking.response.success) {
                ROS_ERROR_STREAM("[PathFollower]: Could not start trajectory tracking. Message: " << trigger_start_tracking.response.message);
                res.message = "Could not call service to start the trajectory tracking";
            }
        }	
        return true;
	}

}
void coverage_planning_wrapper_class::takeoff(){
    //motor on
    motor_request.request.data = 1;
	motor_client.call(motor_request);
	while(!motor_request.response.success)
	{
		ros::Duration(.1).sleep();
		motor_client.call(motor_request);
	}

    //arm
    ros::Duration(5).sleep();
	arm_request.request.value = true;
	arm_client.call(arm_request);
	while(!arm_request.response.success)
	{
		ros::Duration(.1).sleep();
		arm_client.call(arm_request);
	}

    if(arm_request.response.success){
			ROS_INFO("Arming Successful");	
	}

    //offboard mode
    ros::Duration(5).sleep();
	srv_setMode.request.base_mode = 0;
	srv_setMode.request.custom_mode = "offboard";
	set_mode_client.call(srv_setMode);

	if(set_mode_client.call(srv_setMode)){
		ROS_INFO("setmode send ok");
	}else{
	      ROS_ERROR("Failed SetMode");
	}
	//takeoff
	if(arm_request.response.success){
		ros::Duration(5).sleep();
		takeoff_client.call(srv_takeoff);
		std::cout << __FILE__ << ":" << __LINE__ << "i got in if after arming"<<"service responce"<<srv_takeoff.response.success<<std::endl; 
		for(int i=0;i<100;i++)
		{
		//std::cout << __FILE__ << ":" << __LINE__ << "i got in while after arming"  <<std::endl; 
			ros::Duration(.1).sleep();
			takeoff_client.call(srv_takeoff);
		}

		if(takeoff_client.call(srv_takeoff)){
			ROS_INFO("takeoff %d", srv_takeoff.response.success);
		}else{
			ROS_ERROR("Failed Takeoff");
			takeoff();
		}
		}
		takeoff_status =  true;
}
void coverage_planning_wrapper_class::add_heading_to_path(mrs_msgs::Path &path) {
        bool reverse = false;
        double old_heading = 0;
        bool first_not_straight = true;
        //TODO: move this staff to path_generator and combine them somehow
        for (size_t i = 1; i + 2 < path.points.size(); ++i) {
            auto angle1 = std::atan2(path.points[i].position.y - path.points[i - 1].position.y,
                               path.points[i].position.x - path.points[i - 1].position.x);
            auto angle2 = std::atan2(path.points[i + 1].position.y - path.points[i].position.y,
                                     path.points[i + 1].position.x - path.points[i].position.x);

            auto angle3 = std::atan2(path.points[i + 2].position.y - path.points[i + 1].position.y,
                                    path.points[i + 2].position.x - path.points[i + 1].position.x);

            if (std::abs(angle2 - angle1) <= 1e-2) {
                old_heading = path.points[i].heading = angle1 + (reverse ? M_PI : 0);
                first_not_straight = true;
            } else if (std::abs(angle2 - angle3) <= 1e-2) {
                old_heading = path.points[i].heading = angle2 + (reverse ? M_PI : 0);
                first_not_straight = true;
            } else {
                if (first_not_straight) {
                    first_not_straight = false;
                    reverse = !reverse;
                }

            }
            path.points[i].heading = old_heading;
        }
    }
void coverage_planning_wrapper_class::update_path_message_template(const mrs_msgs::PathSrv::Request &req) {
        // Copy some parameters to remember the user preferences. Copy element-wise to not copy all the points too
        m_path_message_template.loop = false;
        m_path_message_template.fly_now = true;
        m_path_message_template.override_constraints = req.path.override_constraints;
        m_path_message_template.relax_heading = req.path.relax_heading;
        m_path_message_template.use_heading = req.path.use_heading;
        m_path_message_template.stop_at_waypoints = req.path.stop_at_waypoints;
        m_path_message_template.override_max_acceleration_horizontal = req.path.override_max_acceleration_horizontal;
        m_path_message_template.override_max_acceleration_vertical = req.path.override_max_acceleration_vertical;
        m_path_message_template.override_max_jerk_horizontal = req.path.override_max_jerk_horizontal;
        m_path_message_template.override_max_jerk_vertical = req.path.override_max_jerk_vertical;
        m_path_message_template.override_max_velocity_horizontal = req.path.override_max_velocity_horizontal;
        m_path_message_template.override_max_velocity_vertical = req.path.override_max_velocity_vertical;
    }


}
PLUGINLIB_EXPORT_CLASS(ns_wrapper::coverage_planning_wrapper_class, nodelet::Nodelet)
