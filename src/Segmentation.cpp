#include<segmentation.h>
namespace ns_segmentation{
    void segmentation_node_class::onInit(){
	ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::subscriber = image_sub = nh.subscribe("camera_feed_topic", &segmentation_node_class::imageCB, this);
    std::cout<<"INSIDE SEGMENTATION"<<std::endl;
    ros::spin();
    }
    void segmentation_node_class::imageCB(& msg){

        boundry.pub(boundry);
    }
}