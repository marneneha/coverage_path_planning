#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <mrs_lib/transformer.h>

namespace ns_uav_state_publisher{
    class uav_state_publisher_node_class: public nodelet::Nodelet{
        public:
        virtual void onInit();
        UavStateCallback(msg)
    };
    void image_publisher_node_class::onInit(){
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
        std::cout<<"INSIDE UAV STATE PUBLISHER"<<std::endl;
        ros::Publisher uav_state_pub = it.advertise("uavState", 1);
    boundry_sub = nh.subscribe<sensor_msgs::Image>("/uav1/coverage_planning_image_publisher/boundry_topic", 10, &boundry_detection_node_class::potential_field_generator, this);
        ros::suscriber waypointCallback_sub =nh.subscribe<>("/uav1/coverage_planning_boundry_detection/boundry_topic")waypointCallback()
        ros::spin();
    }
    image_publisher_node_class::WayPointCallback(){
        nh.sleep(5);
        uav_state_pub.pub(msg);
    }
}
PLUGINLIB_EXPORT_CLASS(ns_uav_state_publisher::uav_state_publisher_node_class, nodelet::Nodelet);