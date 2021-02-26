#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

void tf_publish(const geometry_msgs::Vector3Stamped& rpy) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformstamped;

  transformstamped.header.stamp            = ros::Time::now();
  transformstamped.header.frame_id         = "world";
  transformstamped.child_frame_id          = "drone";
  transformstamped.transform.translation.x = 0.0;
  transformstamped.transform.translation.y = 0.0;
  transformstamped.transform.translation.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
  transformstamped.transform.rotation.w = q.getW();
  transformstamped.transform.rotation.x = q.getX();
  transformstamped.transform.rotation.y = q.getY();
  transformstamped.transform.rotation.z = q.getZ();
  br.sendTransform(transformstamped);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quat_publisher_to_tf");
  ros::NodeHandle nh;
  ros::Subscriber RPY_kalman_sub = nh.subscribe("/pose", 1000, tf_publish);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}