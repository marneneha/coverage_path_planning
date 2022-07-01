#include<coverage_planning_wrapper.h>
#include <pluginlib/class_list_macros.h>
using namespace std;


namespace ns_wrapper{
void coverage_planning_wrapper_class::onInit(){
        cout<<"INSIDE ONINIT"<<endl;
		ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
		mrs_lib::ParamLoader param_loader(nh, "coverage_planning_wrapper");
		//param_loader.loadParam("simulation", _simulation_);
    std::string uav_name;
		param_loader.loadParam("uav_name", uav_name);
      ROS_INFO("[CoveragePathPlanner]: Initialized 2.");
		m_control_manager_stop_following_service_client = nh.serviceClient<std_srvs::Trigger>("stop_trajectory_tracking");
      ROS_INFO("[CoveragePathPlanner]: Initialized 3.");
        m_control_manager_start_following_service_client = nh.serviceClient<std_srvs::Trigger>("start_trajectory_tracking");
      ROS_INFO("[CoveragePathPlanner]: Initialized 4.");
	    m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>("path_out");
      ROS_INFO("[CoveragePathPlanner]: Initialized 5.");

	    m_transformer = mrs_lib::Transformer("coverage_planning_wrapper");
      m_transformer.setDefaultPrefix(uav_name);	

      ROS_INFO("[CoveragePathPlanner]: Initialized 6.");
	    m_service_server_follow_path = nh.advertiseService("path_to_follow", &coverage_planning_wrapper_class::callback_goal,this);

      ROS_INFO("[CoveragePathPlanner]: Initialized.");
      ros::spin();
	}

bool coverage_planning_wrapper_class::callback_goal(mrs_msgs::PathSrv::Request &req, mrs_msgs::PathSrv::Response &res){
        cout<<"callback_goal"<<endl;
		//send trajectory
		res.success = false;
        auto transform = m_transformer.getTransform(req.path.header.frame_id, "gps_origin");
		auto path_transformed = req.path.points;
        for(int i = 0; i<req.path.points.size(); i++){
            cout<<"x is"<<req.path.points[i].position.x<<"y is"<<req.path.points[i].position.y<<endl;
        }
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
void coverage_planning_wrapper_class::add_heading_to_path(mrs_msgs::Path &path) {
        bool reverse = false;
        double old_heading = 0;
        bool first_not_straight = true;
        //TODO: move this staff to path_generator and combine them somehow
        for (size_t i = 1; i + 2 < path.points.size(); ++i) {
            auto angle1 = atan2(path.points[i].position.y - path.points[i - 1].position.y,
                               path.points[i].position.x - path.points[i - 1].position.x);
            auto angle2 = atan2(path.points[i + 1].position.y - path.points[i].position.y,
                                     path.points[i + 1].position.x - path.points[i].position.x);

            auto angle3 = atan2(path.points[i + 2].position.y - path.points[i + 1].position.y,
                                    path.points[i + 2].position.x - path.points[i + 1].position.x);

            if (abs(angle2 - angle1) <= 1e-2) {
                old_heading = path.points[i].heading = angle1 + (reverse ? M_PI : 0);
                first_not_straight = true;
            } else if (abs(angle2 - angle3) <= 1e-2) {
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
