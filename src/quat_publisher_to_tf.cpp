#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

void tf_pose_publish(const geometry_msgs::QuaternionStamped& pose_stamped) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformstamped;

  transformstamped.header.stamp            = pose_stamped.header.stamp;
  transformstamped.header.frame_id         = "world";
  transformstamped.child_frame_id          = "quat_drone";
  transformstamped.transform.translation.x = 0.0;
  transformstamped.transform.translation.y = 0.0;
  transformstamped.transform.translation.z = 1.0;

  transformstamped.transform.rotation.w = pose_stamped.quaternion.w;
  transformstamped.transform.rotation.x = pose_stamped.quaternion.x;
  transformstamped.transform.rotation.y = pose_stamped.quaternion.y;
  transformstamped.transform.rotation.z = pose_stamped.quaternion.z;
  br.sendTransform(transformstamped);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quat_publisher_to_tf");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("/pose", 1000, tf_pose_publish);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}