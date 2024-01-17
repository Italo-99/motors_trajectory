// gripper_client_node.cpp

#include "ros/ros.h"
#include "your_package/YourGripperService.h"  // Replace 'your_package' and 'YourGripperService' with your actual package and service name

class GripperClient
{
public:
  GripperClient(ros::NodeHandle nh) : nh_(nh)
  {
    client_ = nh_.serviceClient<your_package::YourGripperService>("gripper_control_service");  // Replace 'your_package' and 'YourGripperService' with your actual package and service name
  }

  void sendGripperControlRequest(float position, float speed, float force)
  {
    your_package::YourGripperService srv;  // Replace 'your_package' and 'YourGripperService' with your actual package and service name

    // Fill the service request
    srv.request.position = position;
    srv.request.speed = speed;
    srv.request.force = force;

    // Send the request
    if (client_.call(srv))
    {
      ROS_INFO("Gripper Control Request Sent. Status: %d, Success: %s", srv.response.status, srv.response.success ? "true" : "false");
    }
    else
    {
      ROS_ERROR("Failed to call Gripper Control Service");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_client_node");
  ros::NodeHandle nh;

  GripperClient gripperClient(nh);

  // Example: Send Gripper Control Request
  gripperClient.sendGripperControlRequest(50.0, 75.0, 30.0);

  return 0;
}
