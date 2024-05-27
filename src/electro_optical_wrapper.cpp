/* Author: Jason Hughes
*  Date: May 2024
*  About: ROS node for the spinnaker wrapper in eeyore
*/

#include "eeyoreROS/electro_optical_wrapper.hpp"

ElectroOpticalWrapper::ElectroOpticalWrapper(ros::NodeHandle *nh, int h, int w, std::string t)
    : camera_(h,w,t)
{
    image_transport::ImageTransport it(*nh);
    cam_pub_ = it.advertise("camera/electo_optical", 10);

    camera_.quickStart();

    std::string cal_path;
    nh -> getParam("eo_node/cal_path", cal_path);

    cv::Mat intrinsic = camera_.getParams(cal_path, "K");
    cv::Mat distance = camera_.getParams(cal_path, "D");

    camera_.setIntrinsicCoeffs(intrinsic);
    camera_.setDistanceCoeffs(distance);

    nh -> getParam("eo_node/frame_rate", frame_rate_);
    nh -> getParam("eo_node/frame_id", frame_id_);
}

ElectroOpticalWrapper::~ElectroOpticalWrapper()
{
    camera_.closeDevice();
}

void ElectroOpticalWrapper::main()
{
    ros::Rate rate(frame_rate_);

    while (ros::ok())
    {
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv::Mat img = camera_.getFrame();

        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = frame_id_;
        cv_ptr->image = img;

        this -> cam_pub_.publish(cv_ptr->toImageMsg());
        
        ros::spinOnce();
        rate.sleep();
    }
}
