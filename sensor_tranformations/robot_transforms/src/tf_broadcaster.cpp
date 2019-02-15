#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>

bool depth_recorded = false;
double depth_avg = 0;

void depth_callback(const std_msgs::Float64MultiArray depth){

  for (int i=0;i<4;i++)
    depth_avg += depth.data[i];
  depth_avg = depth_avg/4;
  depth_recorded = true;
  printf("\n\n\n\n\n\ndepth :%f", depth_avg);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::Subscriber depth_sub = n.subscribe("DVL_depth", 1000, depth_callback);

  ros::Rate r(5);

  tf::TransformBroadcaster imu_broadcaster, dvl_broadcaster, pool_broadcaster;
/*--------base_link : robosub's frame
  --------base_imu : IMU's frame
  --------base_dvl : DVL's frame
*/

  tf::TransformBroadcaster odom_imu_broadcaster;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  while(n.ok()){
    imu_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(-3.14, 0,-3.14 ), tf::Vector3(0.0, 0.0, +0.3302)),
        ros::Time::now(),"base_link", "base_imu"));

    dvl_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, -3.14/2), tf::Vector3(0.0, 0.0, -0.127)),
        ros::Time::now(),"base_link", "base_dvl"));

    if(depth_recorded){

      // try{
      //   pool_broadcaster.sendTransform(
      //       tf::StampedTransform(
      //         tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(0.0, 0.0, depth_avg)),
      //         ros::Time::now(),"base_pool", "odom"));
      // }
      // catch (tf::TransformException e){
      //   printf("caught exception");
      //   ROS_ERROR("\n %s \n", e.what());
      // }


    }


    // try{
    //   listener.waitForTransform("/base_imu","/odom", ros::Time(0), ros::Duration(2));
    //   listener.lookupTransform("/base_imu","/odom", ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   printf("caught exception");
    //   ROS_ERROR("%s",ex.what());
    // }
    // odom_imu_broadcaster.sendTransform(transform);
    if(!depth_recorded)
      ros::spinOnce();
    r.sleep();
  }
}
