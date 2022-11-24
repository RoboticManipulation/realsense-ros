//#include <chrono>
//#include <functional>
//#include <memory>
//#include <inttypes.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"

#include <array>
#include <vector>

class RepublishCameraInfo
{
  private:
    ros::Subscriber sub;
    ros::Publisher pub;

  public:
    RepublishCameraInfo(ros::NodeHandle *nh)
    {
      sub = nh->subscribe("/camera/color/camera_info_factory", 1000,&RepublishCameraInfo::topic_callback, this);
      pub = nh->advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 10);
    }

    void topic_callback(const sensor_msgs::CameraInfo& camera_info_msgs)
    {
      message = sensor_msgs::CameraInfo();
      message = camera_info_msgs;

      //std::array<double, 5> d = {0.10119861778233893, -0.1954721576865799, 0.0012448486976424538, 0.00191865834729546, 0.0};
      double d_arr[5] = {0.10119861778233893, -0.1954721576865799, 0.0012448486976424538, 0.00191865834729546, 0.0};
      std::vector<double> d_vec(d_arr, d_arr+5);
      message.D = d_vec;

      std::array<double, 9> k_std_arr = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 1.0};
      //double k_arr[9] = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 1.0};
      //std::vector<double> k_vec(k_arr, k_arr+9);
      message.K = k_std_arr;

      std::array<double, 12> p_std_arr = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 0.0, 1.0, 0.0};
      //double p_arr[12] = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 0.0, 1.0, 0.0};
      //std::vector<double> p_vec(p_arr, p_arr+12);
      message.P = p_std_arr;

      //Publish the new message
      pub.publish(message);

    }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "republish_cam_info");
  ros::NodeHandle nh;
  RepublishCameraInfo repub_cam_info = RepublishCameraInfo(&nh);
  ros::spin();
  return 0;
}