/* Author: Jason Hughes
*  Date: May 2024
*  About: Nodelet logic for EO camera
*/
#include <pluginlib/class_list_macros.h>
#include "eeyoreROS/electro_optical_nodelet_wrapper.hpp"

namespace eeyore_ros
{
ElectroOpticalNodelet::~ElectroOpticalNodelet()
{
    camera_.closeDevice();
}

void ElectroOpticalNodelet::onInit()
{
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle private_nh = getPrivateNodeHandle();

    // get parameters
    int h,w;
    std::string trigger;
    TriggerType trig;

    nh.getParam("eo_node/frame_rate", frame_rate_);
    nh.getParam("eo_node/frame_id", frame_id_);
    nh.getParam("eo_node/height", h);
    nh.getParam("eo_node/width", w);
    nh.getParam("eo_node/trigger_type", trigger);

    if (trigger == "SOFTWARE") trig = SOFTWARE;

    // initialize parameters with camera
    camera_.setHeight(h);
    camera_.setWidth(w);
    camera_.setTrigger(trig);
    camera_.initCam();

    // initialize ROS stuff
    timer_ = nh.createTimer(ros::Duration(1.0 / frame_rate_), boost::bind(&ElectroOpticalNodelet::frameRateCallback, this, _1));
    
    image_transport::ImageTransport it(nh);
    cam_pub_ = it.advertise("camera/electro_optical", 1);
}

void ElectroOpticalNodelet::frameRateCallback(const ros::TimerEvent& event)
{
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv::Mat img = camera_.getFrame();

    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = event.current_real;
    cv_ptr->header.frame_id = frame_id_;
    cv_ptr->image = img;

    cam_pub_.publish(cv_ptr->toImageMsg());
}
}

PLUGINLIB_EXPORT_CLASS(eeyore_ros::ElectroOpticalNodelet, nodelet::Nodelet)
