#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

void tf_kalman_publish(const geometry_msgs::Pose& pose) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformstamped;

  transformstamped.header.stamp            = ros::Time::now();
  transformstamped.header.frame_id         = "world";
  transformstamped.child_frame_id          = "quat_drone";
  transformstamped.transform.translation.x = 0.0;
  transformstamped.transform.translation.y = 0.0;
  transformstamped.transform.translation.z = pose.position.z;

  transformstamped.transform.rotation.w = pose.orientation.w;
  transformstamped.transform.rotation.x = pose.orientation.x;
  transformstamped.transform.rotation.y = pose.orientation.y;
  transformstamped.transform.rotation.z = pose.orientation.z;
  br.sendTransform(transformstamped);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quat_publisher_to_tf");
  ros::NodeHandle nh;
  ros::Subscriber RPY_kalman_sub = nh.subscribe("/pose", 1000, tf_kalman_publish);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}