#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

namespace ns_image_publisher{
    class image_publisher_node_class: public nodelet::Nodelet{
        public:
        virtual void onInit();
        std::string _uav_name_, _image_path_;
        bool waypoint_reached = false;
    };
    void image_publisher_node_class::onInit(){
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
        std::cout<<"INSIDE IMAGE PUBLISHER"<<std::endl;
        image_transport::ImageTransport it(nh);
		mrs_lib::ParamLoader param_loader(nh, "coverage_planning_image_publisher");
        param_loader.loadParam("uav_name", _uav_name_);
        param_loader.loadParam("coverage_planning_image_path", _image_path_);
        cv::Mat image = cv::imread(_image_path_, cv::IMREAD_COLOR); 
        image_transport::Publisher image_publisher = it.advertise("boundry_topic", 1);
        // cv::imshow("Image_publisher_window", image);
        // int k = cv::waitKey(0); // Wait for a keystroke in the window
        while(nh.ok()){
            int height = image.size().height;
            int width = image.size().width;
            //divide image in 20parts
            int cropped_height = height/20;
            int y=0, x=0;
            if(waypoint_reached && y<height){
                y = y+cropped_height;
                waypoint_reached = false;
            }
            cv::Mat croppedImage = image(cv::Rect(x,y ,width, y+cropped_height));
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", croppedImage).toImageMsg();
            image_publisher.publish(msg);
            if(height-y < cropped_height){
                y = 0;
            }
            // ros::Rate loop_rate(1);
            // loop_rate.sleep();
            // std::cout<<"inside while"<<std::endl;
        }
       ros::spin();
    }
}
PLUGINLIB_EXPORT_CLASS(ns_image_publisher::image_publisher_node_class, nodelet::Nodelet);