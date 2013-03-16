#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>
#include <algorithm>

//#include <iostream> // TODO: remove this dep

namespace point_cloud_lab
{

class PointCloudLab
{
private:

  ros::NodeHandle nh_;
  std::string action_name_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
  ros::Publisher plane_pub_; // points that were recognized as part of the table
  ros::Publisher block_pose_pub_; // publishes to the block logic server
  tf::TransformListener tf_listener_;

  //std::vector<geometry_msgs::Pose> block_poses_;
  geometry_msgs::PoseArray block_poses_;

  // Parameters of problem
  std::string arm_link;
  double block_size;
  double table_height;

  // Viewer
  pcl::visualization::CloudViewer viewer_;

public:

  PointCloudLab(const std::string name) :
    nh_("~"),
    action_name_(name),
    viewer_("Simple Cloud Viewer")
  {
    // Parameters
    arm_link = "/base_link";
    block_size = 0.04;
    table_height = 0.0;
    block_poses_.header.stamp = ros::Time::now();
    block_poses_.header.frame_id = arm_link;

    // Subscribe to point cloud
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudLab::cloudCallback, this);

    // Publish a point cloud of filtered data that was not part of table
    filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish a point cloud of data that was considered part of the plane
    plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);

    // Publish interactive markers for blocks
    block_pose_pub_ = nh_.advertise< geometry_msgs::PoseArray >("/", 1, true);

  }


  // Proccess the point clouds
  void cloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    ROS_INFO_STREAM("Recieved callback");

    block_poses_.poses.clear();
    // Basic point cloud conversions ---------------------------------------------------------------

    // Convert from ROS to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);



    // Make new point cloud that is in our working frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Transform to whatever frame we're working in, probably the arm's base frame, ie "base_link"
    tf_listener_.waitForTransform(std::string(arm_link), "/camera_link", //cloud->header.frame_id,
                                  cloud->header.stamp, ros::Duration(1.0));
    if(!pcl_ros::transformPointCloud(std::string(arm_link), *cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR("Error converting to desired frame");
      return;
    }


    viewer_.showCloud (cloud_transformed);

    /*
    // Limit to things we think are roughly at the table height ------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredZ(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(table_height - 0.05, table_height + block_size + 0.05);
    //pass.setFilterLimits(table_height - 0.01, table_height + block_size + 0.02); // DTC
    pass.filter(*cloud_filteredZ);


    // Limit to things in front of the robot ---------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.setInputCloud(cloud_filteredZ);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(.1,.5);
    pass.filter(*cloud_filtered);


    // Check if any points remain
    if( cloud_filtered->points.size() == 0 )
    {
      ROS_ERROR("0 points left");
      return;
    }
    else
    {
      ROS_INFO("[block detection] Filtered, %d points left", (int) cloud_filtered->points.size());
    }

    // Segment components --------------------------------------------------------------------------

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // robustness estimator - RANSAC is simple
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.005); // determines how close a point must be to the model in order to be considered an inlier

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());

    int nr_points = cloud_filtered->points.size();

    // Segment cloud until there are less than 30% of points left? not sure why this is necessary
    while(cloud_filtered->points.size() > 0.3 * nr_points)
    {

      // Segment the largest planar component from the remaining cloud (find the table)
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);

      if(inliers->indices.size() == 0)
      {
        ROS_ERROR("[block detection] Could not estimate a planar model for the given dataset.");
        return;
      }

      //std::cout << "Inliers: " << (inliers->indices.size()) << std::endl;

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Write the planar inliers to disk
      extract.filter(*cloud_plane);
      //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered);


      // Debug output - DTC
      // Show the contents of the inlier set, together with the estimated plane parameters, in ax+by+cz+d=0 form (general equation of a plane)
      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                << coefficients->values[1] << " "
                << coefficients->values[2] << " "
                << coefficients->values[3] << std::endl;

      /*std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        for (size_t i = 0; i < inliers->indices.size (); ++i)
        {
        std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
        << cloud.points[inliers->indices[i]].y << " "
        << cloud.points[inliers->indices[i]].z << std::endl;
        }* ///////////////////
    }

    // DTC: Removed to make compatible with PCL 1.5
    // Creating the KdTree object for the search method of the extraction
    //pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    //tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.005);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    //ec.setSearchMethod(tree);
    ec.setInputCloud( cloud_filtered);
    ec.extract(cluster_indices);

    // Publish point cloud data
    filtered_pub_.publish(cloud_filtered);
    plane_pub_.publish(cloud_plane);

    // for each cluster, see if it is a block
    for(size_t c = 0; c < cluster_indices.size(); ++c)
    {
      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0;
      float ymin = 0; float ymax = 0;
      float zmin = 0; float zmax = 0;
      for(size_t i = 0; i < cluster_indices[c].indices.size(); i++)
      {
        int j = cluster_indices[c].indices[i];
        float x = cloud_filtered->points[j].x;
        float y = cloud_filtered->points[j].y;
        float z = cloud_filtered->points[j].z;
        if(i == 0)
        {
          xmin = xmax = x;
          ymin = ymax = y;
          zmin = zmax = z;
        }
        else
        {
          xmin = std::min(xmin, x);
          xmax = std::max(xmax, x);
          ymin = std::min(ymin, y);
          ymax = std::max(ymax, y);
          zmin = std::min(zmin, z);
          zmax = std::max(zmax, z);
        }
      }

      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;
      float zside = zmax-zmin;

      const float tol = 0.01; // 1 cm error tolerance
      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      // z must be equal to or smaller than blocksize
      if(xside > block_size-tol &&
         xside < block_size*sqrt(2)+tol &&
                 yside > block_size-tol &&
         yside < block_size*sqrt(2)+tol &&
                 zside > tol && zside < block_size+tol)
      {
        // If so, then figure out the position and the orientation of the block
        float angle = atan(block_size/((xside+yside)/2));

        if(yside < block_size)
          angle = 0.0;

        ROS_INFO_STREAM("[block detection] xside: " << xside << " yside: " << yside << " zside " << zside << " angle: " << angle);
        // Then add it to our set
        addBlock( xmin+(xside)/2.0, ymin+(yside)/2.0, zmax - block_size/2.0, angle);
      }
  }

    if(block_poses_.poses.size() > 0)
    {
      block_pose_pub_.publish(block_poses_);
      ROS_INFO("[block detection] Finished");
    }
    else
    {
      ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
    }
*/
}

  void addBlock(float x, float y, float z, float angle)
  {
    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;

    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));

    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();

    // Discard noise
    if( block_pose.position.y > 10 || block_pose.position.y < -10 )
    {
      ROS_WARN_STREAM("Rejected block: " << block_pose );
    }

    ROS_INFO_STREAM("Added block: \n" << block_pose );

    block_poses_.poses.push_back(block_pose);
  }

};

};

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("Starting Point Cloud Lab node");

  ros::init(argc, argv, "point_cloud_lab");

  point_cloud_lab::PointCloudLab detector("pcl_lab");

  ros::spin();
  return 0;
}

