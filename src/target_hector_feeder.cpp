#include <ros/ros.h>

#include<Eigen/Core>
#include<Eigen/Dense>

#include <custom_msgs/Blobs.h>
#include <hector_worldmodel_msgs/PosePercept.h>

class TargetFeeder
{
public:
    ros::Publisher target_pub;
    
    TargetFeeder(){
        blob_sub   = nh_.subscribe("vision/Blobs", 100, &TargetFeeder::blobCb, this);
        target_pub = nh_.advertise<hector_worldmodel_msgs::PosePercept>("worldmodel/pose_percept", 20);
    }
    void blobCb(const custom_msgs::Blobs::ConstPtr& blob_msg){
        ros::Time now = ros::Time::now(); 
        pp.header.stamp = now;
        pp.header.frame_id = "base_link";
        pp.info.class_id = "target";
        pp.info.class_support = 1;

        if(blob_msg->blob_count > 0){
            for(int i = 0; i < blob_msg->blob_count; i++){
                pp.pose.pose.position.x = blob_msg->blobs[i].x;
                pp.pose.pose.position.y = blob_msg->blobs[i].y;
                pp.pose.pose.position.z = 0;

                pp.pose.covariance[0]  = 1; //setting covariance
                pp.pose.covariance[7]  = 1;
                pp.pose.covariance[14] = 1;
                pp.pose.covariance[21] = 0.00000000001;
                pp.pose.covariance[28] = 0.00000000001;
                pp.pose.covariance[35] = 0.00000000001;

                target_pub.publish(pp);               
            }
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber blob_sub;
    hector_worldmodel_msgs::PosePercept pp;
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  
  TargetFeeder target_feeder;

  ros::spin();

  return 0;
  
}
