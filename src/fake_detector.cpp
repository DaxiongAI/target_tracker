#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <custom_msgs/Blobs.h>
#include <custom_msgs/Blob.h>

using namespace std;


class Blobfinder
{
  ros::NodeHandle nh_;
  ros::Publisher blob_pub; 
   
public:
  Blobfinder()
  {
    blob_pub = nh_.advertise<custom_msgs::Blobs>("vision/Blobs",1);
    blob_msg.blob_count = 0;
  }


  void Update()
  {
    blob_msg.blob_count = 1;    
    blob_msg.blobs.resize(1);

    blob_msg.header.stamp = ros::Time::now();   
 
    blob_msg.blobs[0].x = 5;
    blob_msg.blobs[0].y = 5;
    blob_msg.blobs[0].z = 0; //the target is always on the ground;
        
    blob_pub.publish(blob_msg);
  }
  
private:

  custom_msgs::Blobs blob_msg;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_detector");
  Blobfinder fd;
  ros::Rate loop_rate(20);

  while(ros::ok()){
    fd.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

