#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <string>
#include <pluginlib/class_list_macros.h>

namespace ns_image_publisher{
    class image_publisher_node_class: public nodelet::Nodelet{
        public:
        virtual void onInit();
    };
    void image_publisher_node_class::onInit(){
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
        image_transport::ImageTransport it(nh);
        std::cout<<"INSIDE IMAGE PUBLISHER"<<std::endl;
        std::string image_path = "/home/mrs/boundry_image.png";
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR); 
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_transport::Publisher image_publisher = it.advertise("boundry_topic", 1);
        ros::Rate loop_rate(2);
        // cv::imshow("Image_publisher_window", image);
        // int k = cv::waitKey(0); // Wait for a keystroke in the window
        while(nh.ok()){
            image_publisher.publish(msg);
            loop_rate.sleep();
            // std::cout<<"inside while"<<std::endl;
        }
       ros::spin();
    }
}
PLUGINLIB_EXPORT_CLASS(ns_image_publisher::image_publisher_node_class, nodelet::Nodelet);