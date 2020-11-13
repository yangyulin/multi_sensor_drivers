#include "ros/ros.h"
#include "ros_mscl/SetAccelBiasModel.h"
#include <cstdlib>


int main(int argc, char **argv){

  ros::init(argc, argv, "set_accel_bias_model_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ros_mscl::SetAccelBiasModel>("SetAccelBiasModel");
  ros_mscl::SetAccelBiasModel srv;

  srv.request.noise_vector.x = atoll(argv[1]);
  srv.request.noise_vector.y = atoll(argv[2]);
  srv.request.noise_vector.z = atoll(argv[3]);
  srv.request.beta_vector.x = atoll(argv[1]);
  srv.request.beta_vector.y = atoll(argv[2]);
  srv.request.beta_vector.z = atoll(argv[3]);

  if (client.call(srv))
  {
      if (srv.response.success)
      {
        ROS_INFO("success");
      }
  }
  else
  {
    ROS_INFO("Failed to call service");
  }
  return 0;
}
