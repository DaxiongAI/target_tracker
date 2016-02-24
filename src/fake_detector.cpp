#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <custom_msgs/Blobs.h>
#include <custom_msgs/Blob.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

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
    counter = 0;
  }


  void Update()
  {
    blob_msg.blob_count = 1;    
    blob_msg.blobs.resize(1);
    counter++;
    blob_msg.header.stamp = ros::Time::now();   
    if(counter > 40){
        boost::mt19937 rng_alt;
        boost::normal_distribution<> nd_alt(-5.0, 0.5);
        boost::variate_generator< boost::mt19937& , boost::normal_distribution<> > gaussian_d(rng_alt, nd_alt);

        blob_msg.blobs[0].x = gaussian_d();
        blob_msg.blobs[0].y = gaussian_d();
        blob_msg.blobs[0].z = 0; //the target is always on the ground;
            
        blob_pub.publish(blob_msg);
    }
    boost::mt19937 rng;
    boost::normal_distribution<> nd(5.0, 0.5);
    boost::variate_generator< boost::mt19937& , boost::normal_distribution<> > gaussian_d(rng, nd);

    blob_msg.blobs[0].x = gaussian_d();
    blob_msg.blobs[0].y = gaussian_d();
    blob_msg.blobs[0].z = 0; //the target is always on the ground;
        
    blob_pub.publish(blob_msg);
  }
  
private:
  int counter;
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

