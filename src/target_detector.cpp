#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <custom_msgs/Blobs.h>
#include <custom_msgs/Blob.h>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

cv::Point3d normalize(cv::Point3d v){
    double len = sqrt(norm(v));
    v.x /= len;
    v.y /= len;
    v.z /= len;

    return v;
}
class Blobfinder
{
  ros::NodeHandle nh_;
  ros::Publisher blob_pub; 
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;
   
public:
  Blobfinder()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribeCamera("/usb_cam/image_raw", 1, 
      &Blobfinder::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //cv::namedWindow(OPENCV_WINDOW);
    blob_pub = nh_.advertise<custom_msgs::Blobs>("vision/Blobs",1);
    iLowH  = 6;//12;
    iHighH = 26;//30;
    iLowS  = 64;//176;
    iHighS = 255;
    iLowV  = 208;
    iHighV = 255;
    blob_msg.blob_count = 0;
    frame_id = "imu";
  }

  ~Blobfinder()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //cvCreateTrackbar("LowH", "Image window", &iLowH, 179);
    //cvCreateTrackbar("HighH","Image window", &iHighH, 179);
       
    //cvCreateTrackbar("LowS", "Image window", &iLowS, 255);
    //cvCreateTrackbar("HighS", "Image window", &iHighS, 255);
 
    //cvCreateTrackbar("LowV", "Image window", &iLowV, 255);
    //cvCreateTrackbar("HighV", "Image window", &iHighV, 255);
    cv::Mat imgHSV;
    cvtColor(cv_ptr->image, imgHSV, CV_BGR2HSV);

    cv::Mat imgThresholded;
    inRange (imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);
      
    //morphological opening (removes small objects on the background)
    erode(imgThresholded, imgThresholded, getStructuringElement(2, cv::Size(5,5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(2, cv::Size(5,5)));
    //morpholocical closing (fill small holes in the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(2, cv::Size(5,5)));
    erode(imgThresholded, imgThresholded, getStructuringElement(2,cv::Size(5,5)));
    
    vector< vector<cv::Point> > contours; //pointer to contouri
    vector<cv::Vec4i> hierarchy;
    
    cv::findContours(imgThresholded, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
     
    vector<vector<cv::Point> > contours_poly( contours.size());
    vector<cv::Rect> boundRect( contours.size());
    vector<cv::Point2f>center( contours.size());
    vector<float>radius( contours.size());
    blob_msg.blob_count = contours.size();    
    blob_msg.blobs.resize(contours.size());
       
 
    cam_model_.fromCameraInfo(info_msg);

    tf::StampedTransform tCamtoBase, tBasetoCam;
    string camera_frame = cam_model_.tfFrame();
    string base_frame   = "/odom";
    try{
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 20);
        tf_listener_.waitForTransform(base_frame, camera_frame, acquisition_time, timeout);
        tf_listener_.lookupTransform( camera_frame, base_frame, acquisition_time, tCamtoBase);
        tf_listener_.lookupTransform( base_frame, camera_frame, acquisition_time, tBasetoCam);
    }catch(tf::TransformException& ex){
        ROS_WARN("Could'nt get the desired TFs! Exception:\n%s", ex.what());
        return;   
    }
    
    tf::Vector3 cameraOrigin = tBasetoCam.getOrigin();
    float cam_height = cameraOrigin.getZ();
    
    //  ------ DEGUB--------------------------------------------
    //printf("Camera height = %.2fm\n", cam_height);
    //printf("Camera origin = %.2f %.2f %.2f\n", cameraOrigin.getX(), cameraOrigin.getY(), cameraOrigin.getZ());
    //printf("Base origin = %.2f %.2f %.2f\n", tCamtoBase.getOrigin().getX(), tCamtoBase.getOrigin().getY(), tCamtoBase.getOrigin().getZ());
    //  ------ DEGUB--------------------------------------------

    //cam_ray.header.frame_id = "cam";
    //cam_ray.header.stamp    = ros::Time();  
    
    for(int i = 0; i < contours.size(); i++){
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
        minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
               
        
        cv::Point2d uv(center[i].x,center[i].y);
        cv::Point3d ray = cam_model_.projectPixelTo3dRay(uv); //ray in the camera frame
       
        printf("Camera ray (pre norm)= %.2f %.2f %.2f\n", ray.x, ray.y, ray.z);

        ray = normalize(ray);

        
        cam_ray.setX(ray.x);
        cam_ray.setY(ray.y);
        cam_ray.setZ(ray.z);

        
        /*
        try{
            tf_listener_.transformPoint("odom", cam_ray, base_ray);
        }catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform the camera ray to the \"odom\": %s", ex.what());
        } */
        base_ray = tBasetoCam(cam_ray);
        base_ray -= cameraOrigin;    
 
        if(base_ray.getZ() < 0){
            float s = cam_height / -base_ray.getZ();
            blob_temp.x = cameraOrigin.getX() + base_ray.getX() * s;
            blob_temp.y = cameraOrigin.getY() + base_ray.getY() * s;
            blob_temp.z = 0; //the target is always on the ground;
            blob_msg.blobs[i] = blob_temp;
        }

        
        //  ------ DEGUB--------------------------------------------
        printf("target position = %.2f %.2f\n", uv.x, uv.y);
        printf("Camera ray = %.2f %.2f %.2f\n", cam_ray.getX(), cam_ray.getY(), cam_ray.getZ());
        printf("Base ray = %.2f %.2f %.2f\n", base_ray.getX(), base_ray.getY(), base_ray.getZ());
        //  ------ DEGUB--------------------------------------------
        

        /*
        blob_temp.x = ray.x;
        blob_temp.y = ray.y;
        blob_temp.z = ray.z; // TODO Intersection with the ground plane (z=0)
        blob_msg.blobs[i] = blob_temp;
        */
    }
    for(int i = 0; i < contours.size(); i++){
        cv::Scalar color = cv::Scalar(0,0,255);
        cv::rectangle(cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
        cv::circle(cv_ptr->image, center[i], (int)radius[i], color, 2, 8, 0);
    }

    // Update GUI Window
    //cv::imshow("Image window",imgThresholded);
    //cv::imshow("", cv_ptr->image);
    //cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    blob_pub.publish(blob_msg);
  }
  
private:

  int iLowH;
  int iHighH;
  int iLowS;
  int iHighS;
  int iLowV;
  int iHighV;
  custom_msgs::Blobs blob_msg;
  custom_msgs::Blob blob_temp;
  std::string frame_id;
  //geometry_msgs::PointStamped cam_ray; //cam ray in the camera frame
  //geometry_msgs::PointStamped base_ray;//cam ray in the base_link frame
  tf::Vector3 cam_ray;
  tf::Vector3 base_ray;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Blobfinder ic;
  ros::spin();
  return 0;
}

