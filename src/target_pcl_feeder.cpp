#include <ros/ros.h>
#include <math.h>
#include<boost/random.hpp>
#include<boost/random/normal_distribution.hpp>

#include<Eigen/Core>
#include<Eigen/Dense>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <custom_msgs/Blobs.h>

#define CLOUDTHRESHOLD 2000  //maximum number of points in the cloud
#define DIST_THRESHOLD 2     //corresponds to distance in m

//class to define the node where each new target detected is storaged
class TargetNode
{
public:
    float x;
    float y;
    int samples;
    TargetNode *next;

    TargetNode(){
        samples = 0;
        next = NULL;
    }
    
    void addNode(TargetNode node){
        next = &node;
    }
    void updateNode(float nx, float ny, int count, Eigen::Matrix3f covariance){
        x = nx;
        y = ny;
        samples = count;
        cov_matrix = covariance;
    }
private:
    Eigen::Matrix3f cov_matrix;
};

//class that mantains a storage of all targets detected
class TargetHolder
{
public:
    TargetNode target_stg;
    int target_count;

    TargetHolder(){
        target_count = 0;
        targetStg_pub = nh_.advertise<custom_msgs::Blobs>("vision/Stored_Targets",1);
    }
    float distance(float vx, float vy, float ux, float uy){
        float distance = sqrt(pow(vx-ux,2)+pow(vy-uy,2));
        return distance;
    }
    void outputTargets(){
        if(target_count > 0){
            target_msg.blob_count = target_count;
            target_msg.blobs.resize(target_count);

            ros::Time tnow = ros::Time::now(); 
            target_msg.header.stamp = tnow; 

            TargetNode *aux = &target_stg;
            int j =  0;
            float end = false;
            while(!end){
                target_msg.blobs[j].x = aux->x;
                target_msg.blobs[j].y = aux->y;
                target_msg.blobs[j].z = aux->samples;
                if(aux->next != NULL){
                    aux = aux->next;
                }else{
                    end = true;
                }
                j++;
            }

            targetStg_pub.publish(target_msg);
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher targetStg_pub;
    custom_msgs::Blobs target_msg;
};


class TargetFeeder
{
public:
    int counter;

    ros::Publisher target_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //create PCL cloud structure

    TargetFeeder(){
        blob_sub   = nh_.subscribe("vision/Blobs", 100, &TargetFeeder::blobCb, this);
        target_pub = nh_.advertise<custom_msgs::Blobs>("vision/Targets",1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ ( new pcl::PointCloud<pcl::PointXYZ>); //create PCL cloud structure
        cloud = cloud_;
        counter = 0;
    }
    void blobCb(const custom_msgs::Blobs::ConstPtr& blob_msg){
        if(blob_msg->blob_count > 0){
            for(int i = 0; i < blob_msg->blob_count; i++){
                point_aux.x = blob_msg->blobs[i].x;
                point_aux.y = blob_msg->blobs[i].y;
                point_aux.z = 0;
                counter = counter + 1; 
                cloud->insert(cloud->end(), point_aux);

                if(cloud->points.size() > CLOUDTHRESHOLD){
                    cloud->erase(cloud->begin());
                    counter = counter - 1;
                }
            }
        }
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber blob_sub;
    pcl::PointXYZ point_aux;
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  //Creates output msg and class for target output
  custom_msgs::Blobs target_msg;
  TargetFeeder target_feeder;
  
  //creates instance of TargetHolder for storage of the targets
  TargetHolder target_holder;  

  // creating kdtree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> cluster_indices;
  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.5); //max distance between samples of the same cluster (?)
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);

  ros::Rate loop_rate(5);
  while(ros::ok()){ 
      ros::Time now = ros::Time::now(); 
      target_msg.header.stamp = now; 
      pcl::PointCloud<pcl::PointXYZ> cloud_aux;
      cloud_aux = *target_feeder.cloud; 
      if(cloud_aux.points.size() > 0){
          tree->setInputCloud(target_feeder.cloud);
          //creating storage for clusters
          //cluster extraction
          cluster_indices.clear();
          ec.setInputCloud(target_feeder.cloud);
          ec.extract(cluster_indices);

          target_msg.blob_count = cluster_indices.size();
          target_msg.blobs.resize(target_msg.blob_count);
          int j = 0;
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); ++it){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit!=it->indices.end(); ++pit){
              cloud_cluster->points.push_back(target_feeder.cloud->points[*pit]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            //calculate centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            //calculate the covariance
            Eigen::Matrix3f covariance_matrix;
            pcl::computeCovarianceMatrix (*cloud_cluster, centroid, covariance_matrix);
            
            float x_ = centroid(0);
            float y_ = centroid(1);
            
            
            bool flag = false;
            float dist;
            TargetNode* auxNode;
            auxNode = &(target_holder.target_stg);
            while(!flag){
                if(target_holder.target_count != 0){
                    dist = target_holder.distance(x_ , y_, auxNode->x, auxNode->y);
                    if(dist < DIST_THRESHOLD){
                        if(auxNode->samples < cloud_cluster->points.size()){
                            //
                            //std::cout << "Updating stored target" << std::endl; 
                            //
                            auxNode->updateNode(x_ , y_, cloud_cluster->points.size(), covariance_matrix);        
                        }
                        flag = true;
                    }else{
                        if(auxNode->next != NULL){
                            auxNode = auxNode->next;
                        }else{
                            break;
                        }
                    }
                }else{
                    //notice that a cluster is only created if there are at least 50 points to support it
                    //
                    //std::cout << "Adding first target to storage" << std::endl; 
                    //
                    target_holder.target_stg.updateNode( x_ , y_, cloud_cluster->points.size(), covariance_matrix);
                    target_holder.target_count++;
                    flag = true;
                }
            }
            if(auxNode->next == NULL && flag == false){
                //
                //std::cout << "Adding new target to target storage..." << std::endl; 
                //
                auxNode->next = new TargetNode();
                auxNode->next->updateNode( x_, y_, cloud_cluster->points.size(), covariance_matrix);
                target_holder.target_count++;
            }

            target_msg.blobs[j].x = centroid(0);
            target_msg.blobs[j].y = centroid(1);
            target_msg.blobs[j].z = cloud_cluster->points.size();


           // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points. Its centroid is:\n" << centroid << std::endl;
           // std::cout << "Its covariance matrix is:\n" << covariance_matrix << std::endl; 
            j++;
          }
          target_feeder.target_pub.publish(target_msg);
          target_holder.outputTargets();
      }
      ros::spinOnce();
      loop_rate.sleep();
  } 

  return 0;
  
}
