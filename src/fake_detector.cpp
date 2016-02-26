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

    boost::normal_distribution<> nd_aux(0.0, 0.3);
    nd = nd_aux;
    boost::normal_distribution<> nd_aux2(0.0, 0.3);
    nd_ = nd_aux2;

    gaussian_ptr = new boost::variate_generator< boost::mt19937& , boost::normal_distribution<> >(rng, nd);
    gaussian_ptr_ = new boost::variate_generator< boost::mt19937& , boost::normal_distribution<> >(rng_, nd_);
  }


  void Update()
  {
    blob_msg.blob_count = 1;    
    blob_msg.blobs.resize(1);
    counter++;
    blob_msg.header.stamp = ros::Time::now();   
    if(counter > 50){
        blob_msg.blob_count = 2;    
        blob_msg.blobs.resize(2);
        blob_msg.blobs[1].x = -5 + (*gaussian_ptr_)();
        blob_msg.blobs[1].y = -5 + (*gaussian_ptr_)();
        blob_msg.blobs[1].z = 0; //the target is always on the ground;
            
        //blob_pub.publish(blob_msg);
    }
    blob_msg.blobs[0].x = 5 + (*gaussian_ptr)();
    blob_msg.blobs[0].y = 5 + (*gaussian_ptr)();
    blob_msg.blobs[0].z = 0; //the target is always on the ground;
        
    blob_pub.publish(blob_msg);
  }
  
private:
  int counter;
  custom_msgs::Blobs blob_msg;
  boost::normal_distribution<> nd;
  boost::normal_distribution<> nd_;
  boost::mt19937 rng;
  boost::mt19937 rng_;
  boost::variate_generator<boost::mt19937& , boost::normal_distribution<> >* gaussian_ptr;
  boost::variate_generator<boost::mt19937& , boost::normal_distribution<> >* gaussian_ptr_;
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

