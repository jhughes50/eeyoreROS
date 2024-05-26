/* Author: Jason Hughes
*  Date: May 2024
*  About: ROS node for the spinnaker wrapper in eeyore
*/

#include "eeyorROS/electro_optical_wrapper.hpp"

ElectroOpticalWrapper::ElectroOpticalWrapper(ros::NodeHandle *nh, int h, int w, std::string t)
{
    image_transport::ImageTransport it(*nh);
    cam_pub_ = it.advertise("camera/electo_optical", 10);

    cam_.quickStart();

    std::string cal_path;
    nh -> getParam("eeyore_ros/cal_path", cal_path);

    cv::Mat intrinsic = cam_.getParams(cal_path, "K");
    cv::Mat distance = cam_.getParams(cal_path, "D");

    cam_.setIntrinsicCoeffs(intrinsic);
    cam_.setDistanceCoeffs(distance);

    nh -> getParam("eeyore_ros/frame_rate", frame_rate_);
}

ElectroOpticalWrapper::~ElectroOpticalWrapper()
{
    cam_.closeDevice();
}

void ElectroOpticalWrapper::main()
{
    ros::Rate rate(frame_rate_);

    while (ros::ok())
    {
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv::Mat img = blackfly_cam_.getFrame();

        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = time_in.stamp;
        cv_ptr->header.frame_id = time_in.frame_id;
        cv_ptr->image = img;

        this -> cam_pub_.publish(cv_ptr->toImageMsg());
        
        ros::spinOnce();
        rate.sleep();
    }
}
