#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


void tf_kalman_publish(const geometry_msgs::Vector3& rpy)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformstamped;

  transformstamped.header.stamp = ros::Time::now();
  transformstamped.header.frame_id = "world";
  transformstamped.child_frame_id = "drone_kalman";
  transformstamped.transform.translation.x = 0.0;
  transformstamped.transform.translation.y = 0.0;
  transformstamped.transform.translation.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(rpy.x, rpy.y, rpy.z);
  transformstamped.transform.rotation.w = q.getW();
  transformstamped.transform.rotation.x = q.getX();
  transformstamped.transform.rotation.y = q.getY();
  transformstamped.transform.rotation.z = q.getZ();
  br.sendTransform(transformstamped);
}

void tf_obse_publish(const geometry_msgs::Vector3& rpy)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformstamped;

  transformstamped.header.stamp = ros::Time::now();
  transformstamped.header.frame_id = "obse_base";
  transformstamped.child_frame_id = "drone_obse";
  transformstamped.transform.translation.x = 0.0;
  transformstamped.transform.translation.y = 0.0;
  transformstamped.transform.translation.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(rpy.x, rpy.y, rpy.z);
  transformstamped.transform.rotation.w = q.getW();
  transformstamped.transform.rotation.x = q.getX();
  transformstamped.transform.rotation.y = q.getY();
  transformstamped.transform.rotation.z = q.getZ();
  br.sendTransform(transformstamped);
}

void tf_pred_publish(const geometry_msgs::Vector3& rpy)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformstamped;

  transformstamped.header.stamp = ros::Time::now();
  transformstamped.header.frame_id = "pred_base";
  transformstamped.child_frame_id = "drone_pred";
  transformstamped.transform.translation.x = 0.0;
  transformstamped.transform.translation.y = 0.0;
  transformstamped.transform.translation.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(rpy.x, rpy.y, rpy.z);
  transformstamped.transform.rotation.w = q.getW();
  transformstamped.transform.rotation.x = q.getX();
  transformstamped.transform.rotation.y = q.getY();
  transformstamped.transform.rotation.z = q.getZ();
  br.sendTransform(transformstamped);
}

void static_tf2_broadcast()
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped_pred;
  geometry_msgs::TransformStamped static_transformStamped_obse;

  static_transformStamped_pred.header.stamp = ros::Time::now();
  static_transformStamped_pred.header.frame_id = "world";
  static_transformStamped_pred.child_frame_id = "pred_base";
  static_transformStamped_pred.transform.translation.x = 1.0;
  static_transformStamped_pred.transform.translation.y = 0.0;
  static_transformStamped_pred.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  static_transformStamped_pred.transform.rotation.w = q.getW();
  static_transformStamped_pred.transform.rotation.x = q.getX();
  static_transformStamped_pred.transform.rotation.y = q.getY();
  static_transformStamped_pred.transform.rotation.z = q.getZ();
  static_broadcaster.sendTransform(static_transformStamped_pred);

  static_transformStamped_obse.header.stamp = ros::Time::now();
  static_transformStamped_obse.header.frame_id = "world";
  static_transformStamped_obse.child_frame_id = "obse_base";
  static_transformStamped_obse.transform.translation.x = -1.0;
  static_transformStamped_obse.transform.translation.y = 0.0;
  static_transformStamped_obse.transform.translation.z = 0.0;
  // tf2::Quaternion q;
  // q.setRPY(0.0, 0.0, 0.0);
  static_transformStamped_obse.transform.rotation.w = q.getW();
  static_transformStamped_obse.transform.rotation.x = q.getX();
  static_transformStamped_obse.transform.rotation.y = q.getY();
  static_transformStamped_obse.transform.rotation.z = q.getZ();
  static_broadcaster.sendTransform(static_transformStamped_obse);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_publisher_to_tf");
  ros::NodeHandle nh;
  ros::Subscriber RPY_kalman_sub = nh.subscribe("/RPY_kalman_rad", 1000, tf_kalman_publish);
  ros::Subscriber RPY_obse_sub = nh.subscribe("/no_filter_obse", 1000, tf_obse_publish);
  ros::Subscriber RPY_pred_sub = nh.subscribe("/no_filter_pred", 1000, tf_pred_publish);
  static_tf2_broadcast();
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}