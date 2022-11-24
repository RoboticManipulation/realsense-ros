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
      sensor_msgs::CameraInfo message = sensor_msgs::CameraInfo();
      message = camera_info_msgs;

      double d_arr[5] = {0.10119861778233893, -0.1954721576865799, 0.0012448486976424538, 0.00191865834729546, 0.0};
      std::vector<double> d_vec(d_arr, d_arr+5);
      message.D = d_vec;

      boost::array<double, 9> k_std_arr = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 1.0};
      message.K = k_std_arr;

      boost::array<double, 12> p_std_arr = {1383.6978295014662, 0.0, 975.384325483263, 0.0, 0.0, 1384.8496632036263, 574.7937430384186, 0.0, 0.0, 0.0, 1.0, 0.0};
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