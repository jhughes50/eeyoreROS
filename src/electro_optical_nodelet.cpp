/* Author: Jason Hughes
*  Date: July 2023
*  About: file to start the eo node
*/

#include "eeyoreROS/electro_optical_nodelet_wrapper.hpp"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "electro_optical_nodelet");
    ros::NodeHandle nh;
    nodelet::Loader nodelet;

    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    
    std::string trigger_type;

    nh.getParam("eo_node/trigger_type", trigger_type);

    try
    {
        int h = 0;
        int w = 0;
        
        nodelet.load(nodelet_name, "eeyore_ros/ElectroOpticalNodelet", remap, nargv);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    }

    return 0;
}
