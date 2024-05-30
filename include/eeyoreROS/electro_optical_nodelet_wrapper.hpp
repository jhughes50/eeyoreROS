/* Author: Jason Hughes
*  Date: May 2024
*  About: Nodelet for Spinnaker camera
*/
#ifndef ELECTRO_OPTICAL_NODELET_WRAPPER_HPP
#define ELECTRO_OPTICAL_NODELET_WRAPPER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.h>

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "eeyore/electro_optical.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace eeyore_ros
{
    class ElectroOpticalNodelet : public nodelet::Nodelet
    {
        public:
            ~ElectroOpticalNodelet();

        private:
            virtual void onInit();
            void frameRateCallback(const ros::TimerEvent& event);
            
            int frame_rate_;
            std::string frame_id_;
            
            image_transport::Publisher cam_pub_;
            ros::Timer timer_;

            ElectroOpticalCam camera_;
    };
} // namespace eeyore_ros
#endif
