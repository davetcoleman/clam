/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Michael Ferguson, Helen Oleynikova, Dave Coleman
 */

/*
  DEV NOTES:
  Types of clouds:
  cloud
  cloud_transformed
  cloud_filtered
  cloud_plane

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <clam_msgs/BlockDetectionAction.h>

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

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>
#include <algorithm>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//OpenCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>


namespace clam_msgs
{

typedef cv::Mat_<cv::Vec3b> RGBImage;


class BlockDetectionServer
{
private:

  ros::NodeHandle nh_;

  // Actionlib
  actionlib::SimpleActionServer<clam_msgs::BlockDetectionAction> action_server_;
  std::string action_name_;

  // Actionlib messages
  clam_msgs::BlockDetectionFeedback feedback_;
  clam_msgs::BlockDetectionResult result_;
  clam_msgs::BlockDetectionGoalConstPtr goal_;

  // ROS Connections
  ros::Subscriber point_cloud_sub_;
  ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
  ros::Publisher plane_pub_; // points that were recognized as part of the table
  ros::Publisher block_pose_pub_; // publishes to the block logic server
  ros::Publisher block_marker_pub_; // shows markers in rviz
  tf::TransformListener tf_listener_;

  // Parameters from goal
  std::string arm_link;
  double block_size;
  double table_height;

  // Frequency of image processing
  static const unsigned int PROCESS_EVERY_NTH = 100;
  unsigned int process_count_;

  // OpenCV data structures
  cv::Mat cluster_image;
  cv::Mat cluster_image_gray;
  cv::Mat cluster_image_cropped;
  int opencv_threshhold;
  int max_opencv_threshhold;
  cv::RNG opencv_rand_gen;

public:

  BlockDetectionServer(const std::string name) :
    nh_("~"),
    action_server_(name, false),
    action_name_(name)
  {
    // Subscribe to point cloud
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &BlockDetectionServer::pointCloudCallback, this);

    // Publish a point cloud of filtered data that was not part of table
    filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish a point cloud of data that was considered part of the plane
    plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);

    // Publish interactive markers for blocks
    block_pose_pub_ = nh_.advertise< geometry_msgs::PoseArray >("block_orientation", 1, true);

    // Publish markers to highlight blocks
    block_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("block_marker", 1);

    // Setup OpenCV stuff
    opencv_rand_gen(12345); // ??
    opencv_threshhold = 100;
    max_opencv_threshhold = 255;

    // Initialize how often we process images
    process_count_ = PROCESS_EVERY_NTH;

    // TODO: move this, should be brought in from action goal. temporary!
    arm_link = "/base_link";
    block_size = 0.04;
    table_height = 0.001;

    // Register the goal and feeback callbacks.
    action_server_.registerGoalCallback(boost::bind(&BlockDetectionServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&BlockDetectionServer::preemptCB, this));

    action_server_.start();

    // Announce state
    ROS_INFO("[block detection] Server ready");


  }

  ~BlockDetectionServer()
  {
    cv::destroyAllWindows();
  }

  void goalCB()
  {
    ROS_INFO("[block detection] Starting detection");

    // Accept the new goal and save data
    goal_ = action_server_.acceptNewGoal();
    block_size = goal_->block_size;
    table_height = goal_->table_height;
    arm_link = goal_->frame;
  }

  // Cancel the detection
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());

    // set the action state to preempted
    action_server_.setPreempted();
  }

  // Decide if we should proccess the point cloud
  void pointCloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    // Only process every nth point cloud, unless we are working on a goal inwhich case process all of them
    ++process_count_;

    if( process_count_ > PROCESS_EVERY_NTH )
    {
      process_count_ = 0;
    }
    else
    {
      // Only do this if we're actually actively working on a goal.
      if(!action_server_.isActive())
        return;
    }
    processPointCloud( msg );
  }

  // Proccess the point clouds
  void processPointCloud( const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg )
  {
    ROS_INFO("\n\n\n");
    ROS_INFO("[block detection] Processing new point cloud");

    // ---------------------------------------------------------------------------------------------
    // Start making result
    result_.blocks.poses.clear();    // Clear last block detection result
    result_.blocks.header.stamp = pointcloud_msg->header.stamp;
    result_.blocks.header.frame_id = arm_link;

    // Basic point cloud conversions ---------------------------------------------------------------

    // Convert from ROS to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*pointcloud_msg, cloud);

    // Make new point cloud that is in our working frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Transform to whatever frame we're working in, probably the arm's base frame, ie "base_link"
    tf_listener_.waitForTransform(std::string(arm_link), cloud.header.frame_id,
                                  cloud.header.stamp, ros::Duration(2.0));
    if(!pcl_ros::transformPointCloud(std::string(arm_link), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR("Error converting to desired frame");

      // Do this to speed up the next process attempt:
      process_count_ = PROCESS_EVERY_NTH;

      return;
    }


    // Limit to things we think are roughly at the table height ------------------------------------
    //    pcl::PointIndices::Ptr filtered_indices(new pcl::PointIndices); // hold things at table height
    //    std::vector<int>
    boost::shared_ptr<std::vector<int> > filtered_indices(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(table_height - 0.05, table_height + block_size + 0.05);
    //pass.setFilterLimits(table_height - 0.01, table_height + block_size + 0.02); // DTC
    pass.filter(*filtered_indices);

    /*
    // Limit to things in front of the robot ---------------------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pass.setInputCloud(cloud_filteredZ);
    pass.setIndices(filtered_indices);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(.1,.5);
    pass.filter(*cloud_filtered);
    */

    /*
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
    */

    // Segment components --------------------------------------------------------------------------


    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // robustness estimator - RANSAC is simple
    seg.setMaxIterations(200); // the maximum number of iterations the sample consensus method will run
    seg.setDistanceThreshold(0.005); // determines how close a point must be to the model in order to be considered an inlier


    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());

    /*
      int nr_points = cloud_filtered->points.size();

      // Segment cloud until there are less than 30% of points left? not sure why this is necessary
      while(cloud_filtered->points.size() > 0.3 * nr_points)
      {

      // Segment the largest planar component from the remaining cloud (find the table)
      seg.setInputCloud(cloud_filtered);
      //      seg.setIndices();
      seg.segment(*inliers, *model_coefficients);

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

      // Copy the extracted component (the table) to a seperate point cloud
      extract.filter(*cloud_plane);
      //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered);  // remove table from cloud_filtered


      // Debug output - DTC
      // Show the contents of the inlier set, together with the estimated plane parameters, in ax+by+cz+d=0 form (general equation of a plane)
      ROS_INFO_STREAM( "[block detection] Model coefficients: " << model_coefficients->values[0] << " "
      << model_coefficients->values[1] << " "
      << model_coefficients->values[2] << " "
      << model_coefficients->values[3] ); // TODO: turn this into an rviz marker somehow?
    */
    // Show groups of recognized objects (the inliers)
    /*std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
      for (size_t i = 0; i < inliers->indices.size (); ++i)        {
      std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
      << cloud.points[inliers->indices[i]].y << " "
      << cloud.points[inliers->indices[i]].z << std::endl;
      }*/
    //    }


    // DTC: Removed to make compatible with PCL 1.5
    // Creating the KdTree object for the search method of the extraction
    //    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTreeFLANN<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    //    tree->setInputCloud(cloud_filtered);
    tree->setInputCloud(cloud_transformed, filtered_indices);

    // Find the clusters (objects) on the table
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extract;
    //cluster_extract.setClusterTolerance(0.005); // 5mm -  If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. So our recommendation is to just test and try out which value suits your dataset.
    cluster_extract.setClusterTolerance(0.02); // 2cm
    cluster_extract.setMinClusterSize(100);
    cluster_extract.setMaxClusterSize(25000);
    //    cluster_extract.setSearchMethod(tree);
    //    cluster_extract.setInputCloud(cloud_filtered);
    cluster_extract.setInputCloud(cloud_transformed);
    cluster_extract.setIndices(filtered_indices);
    ROS_INFO("Extracting...");
    cluster_extract.extract(cluster_indices);
    ROS_INFO("after cluster extract");

    // Publish point cloud data
    //    filtered_pub_.publish(cloud_filtered);
    //    plane_pub_.publish(cloud_plane);

    ROS_WARN_STREAM("Number indicies/clusters: " << cluster_indices.size() );

    //    processClusters( cluster_indices, pointcloud_msg, cloud_filtered );
    processClusters( cluster_indices, cloud_transformed, cloud_filtered, cloud );

    // ---------------------------------------------------------------------------------------------
    // Final results
    if(result_.blocks.poses.size() > 0)
    {
      // Change action state, if we the action is currently active
      if(action_server_.isActive())
      {
        action_server_.setSucceeded(result_);
      }
      // Publish block poses
      block_pose_pub_.publish(result_.blocks);

      // Publish rviz markers of the blocks
      publishBlockLocation();

      ROS_INFO("[block detection] Finished ---------------------------------------------- ");
    }
    else
    {
      ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
    }
  }

  // Processes the point cloud with OpenCV using the PCL cluster indices
  void processClusters( const std::vector<pcl::PointIndices> cluster_indices,
                        //                        const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg,
                        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr  cloud_transformed,
                        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_filtered,
                        const pcl::PointCloud<pcl::PointXYZRGB>&  cloud )
  {

    // -------------------------------------------------------------------------------------------------------
    // Convert image
    ROS_INFO("Converting image to OpenCV format");

    try
    {
      sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
      //      pcl::toROSMsg (*pointcloud_msg, *image_msg);
      pcl::toROSMsg (*cloud_transformed, *image_msg);
      cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(image_msg, "rgb8");
      cluster_image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }

    // -------------------------------------------------------------------------------------------------------
    // Process Image

    // Convert image to gray
    cv::cvtColor( cluster_image, cluster_image_gray, CV_BGR2GRAY );
    //cv::adaptiveThreshold( cluster_image, cluster_image_gray, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,5,10);

    // Blur image - reduce noise with a 3x3 kernel
    cv::blur( cluster_image_gray, cluster_image_gray, cv::Size(3,3) );

    ROS_INFO("Finished coverting");

    // -------------------------------------------------------------------------------------------------------
    // Check OpenCV and PCL image height for errors
    int image_width = cloud.width;
    int image_height = cloud.height;
    ROS_DEBUG_STREAM( "PCL Image height " << image_height << " -- width " << image_width << "\n");
    int image_width_cv = cluster_image_gray.size.p[1];
    int image_height_cv = cluster_image_gray.size.p[0];
    ROS_DEBUG_STREAM( "OpenCV Image height " << image_height_cv << " -- width " << image_width_cv << "\n");

    if( image_width != image_width_cv || image_height != image_height_cv )
    {
      ROS_ERROR("PCL and OpenCV image heights/widths do not match!");
      return;
    }

    // -------------------------------------------------------------------------------------------------------
    // GUI Stuff

    // First window
    const char* opencv_window = "Source";
    /*
      cv::namedWindow( opencv_window, CV_WINDOW_AUTOSIZE );
      cv::imshow( opencv_window, cluster_image_gray );
      cv::createTrackbar( " Canny thresh:", "Source", &opencv_threshhold, max_opencv_threshhold );
    */

    // -------------------------------------------------------------------------------------------------------
    // Start processing clusters
    ROS_INFO("Finding min/max in x/y axis");

    int top_image_overlay_x = 0; // tracks were to copyTo the mini images

    // for each cluster, see if it is a block
    for(size_t c = 0; c < cluster_indices.size(); ++c)
    {
      ROS_INFO("\n\n");
      ROS_INFO("On cluster %i", int(c));

      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0;
      float ymin = 0; float ymax = 0;

      // also remember each min & max's correponding other coordinate (not needed for z)
      float xminy = 0; float xmaxy = 0;
      float yminx = 0; float ymaxx = 0;

      // also remember their corresponding indice
      int xmini = 0; int xmaxi = 0;
      int ymini = 0; int ymaxi = 0;

      // loop through and find all min/max of x/y
      for(size_t i = 0; i < cluster_indices[c].indices.size(); i++)
      {
        int j = cluster_indices[c].indices[i];

        // Get RGB from point cloud
        pcl::PointXYZRGB p = cloud_transformed->points[j];

        float x = p.x;
        float y = p.y;

        if(i == 0) // initial values
        {
          xmin = xmax = x;
          ymin = ymax = y;
          xminy = xmaxy = y;
          yminx = ymaxx = x;
          xmini = xmaxi = ymini = ymaxi = j; // record the indice corresponding to the min/max
        }
        else
        {
          if( x < xmin )
          {
            xmin = x;
            xminy = y;
            xmini = j;
          }
          if( x > xmax )
          {
            xmax = x;
            xmaxy = y;
            xmaxi = j;
          }
          if( y < ymin )
          {
            ymin = y;
            yminx = x;
            ymini = j;
          }
          if( y > ymax )
          {
            ymax = y;
            ymaxx = x;
            ymaxi = j;
          }
        }
      }

      ROS_DEBUG_STREAM("Cluster size - xmin: " << xmin << " xmax: " << xmax << " ymin: " << ymin << " ymax: " << ymax);
      ROS_DEBUG_STREAM("Cluster size - xmini: " << xmini << " xmaxi: " << xmaxi << " ymini: " << ymini << " ymaxi: " << ymaxi);

      // ---------------------------------------------------------------------------------------------
      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;

      const float tol = 0.01; // 1 cm error tolerance

      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      if(xside > block_size-tol &&
         xside < block_size*sqrt(2)+tol &&
                 yside > block_size-tol &&
        yside < block_size*sqrt(2)+tol )
      {

        // -------------------------------------------------------------------------------------------------------
        // Convert to OpenCV Mat format
        /*
        //cv::Mat block_image = cv::Mat::zeros( image_height, image_width, CV_8UC3 );
        RGBImage block_image = RGBImage::zeros(image_height, image_width); //, CV_8UC3 );

        // Loop though all the indicies and copy to OpenCV
        for(size_t i = 0; i < cluster_indices[c].indices.size(); i++)
        {
        int j = cluster_indices[c].indices[i];

        // Get RGB from point cloud
        pcl::PointXYZRGB p = cloud.points[j];
        int row = 0;
        int col = 0;

        // Get the pixel coordinates of the index
        getXYCoordinates( j, image_height, image_width, col, row);

        // unpack rgb into r/g/b
        uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;
        //ROS_DEBUG_STREAM("RGB is " << int(r) << ", " << int(g) << ", " << int(b) << " at ROW,COL " << row << ", " << col);

        if( isnan(row) || isnan(col) )
        {
        ROS_ERROR("is nan");
        continue;
        }

        block_image.at<cv::Vec3b>(row,col)[0] = r;
        block_image.at<cv::Vec3b>(row,col)[1] = g;
        block_image.at<cv::Vec3b>(row,col)[2] = b;
        //ROS_DEBUG_STREAM("done assigning rgb");
        }


        ROS_DEBUG_STREAM("pre imshow");
        cv::imshow( "Copy conversion", block_image );
        ROS_DEBUG_STREAM("imshow");
        cv::waitKey(1000); // 50 milisec to allow gui to catch up
        */

        // -------------------------------------------------------------------------------------------------------
        // Get the four farthest corners of the block - use OpenCV only on the region identified by PCL

        // Get the pixel coordinates of the xmax and ymax indicies
        int px_xmax = 0; int py_xmax = 0;
        int px_ymax = 0; int py_ymax = 0;
        getXYCoordinates( xmaxi, image_height, image_width, px_xmax, py_xmax);
        getXYCoordinates( ymaxi, image_height, image_width, px_ymax, py_ymax);

        // Get the pixel coordinates of the xmin and ymin indicies
        int px_xmin = 0; int py_xmin = 0;
        int px_ymin = 0; int py_ymin = 0;
        getXYCoordinates( xmini, image_height, image_width, px_xmin, py_xmin);
        getXYCoordinates( ymini, image_height, image_width, px_ymin, py_ymin);

        ROS_DEBUG_STREAM("px_xmin " << px_xmin << " px_xmax: " << px_xmax << " py_ymin: " << py_ymin << " py_ymax: " << py_ymax );

        // -------------------------------------------------------------------------------------------------------
        // Change the frame of reference from the robot to the camera

        // Create an array of all the x value options
        const int x_values_a[] = {px_xmax, px_ymax, px_xmin, px_ymin};
        const int y_values_a[] = {py_xmax, py_ymax, py_xmin, py_ymin};
        // Turn it into a vector
        std::vector<int> x_values (x_values_a, x_values_a + sizeof(x_values_a) / sizeof(x_values_a[0]));
        std::vector<int> y_values (y_values_a, y_values_a + sizeof(y_values_a) / sizeof(y_values_a[0]));
        // Find the min
        int x1 = *std::min_element(x_values.begin(), x_values.end());
        int y1 = *std::min_element(y_values.begin(), y_values.end());
        // Find the max
        int x2 = *std::max_element(x_values.begin(), x_values.end());
        int y2 = *std::max_element(y_values.begin(), y_values.end());

        ROS_DEBUG_STREAM("x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);

        // -------------------------------------------------------------------------------------------------------
        // Expand the ROI by a fudge factor, if possible
        const int FUDGE_FACTOR = 5; // pixels
        if( x1 > FUDGE_FACTOR)
          x1 -= FUDGE_FACTOR;
        if( y1 > FUDGE_FACTOR )
          y1 -= FUDGE_FACTOR;
        if( x2 < image_width - FUDGE_FACTOR )
          x2 += FUDGE_FACTOR;
        if( y2 < image_height - FUDGE_FACTOR )
          y2 += FUDGE_FACTOR;

        ROS_DEBUG_STREAM("After Fudge Factor - x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);

        // -------------------------------------------------------------------------------------------------------
        // Create ROI parameters
        //        (x1,y1)----------------------
        //       |                            |
        //       |            ROI             |
        //       |                            |
        //       |_____________________(x2,y2)|

        // Create Region of Interest
        int roi_width = x2 - x1;
        int roi_height = y2 - y1;
        cv::Rect region_of_interest = cv::Rect( x1, y1, roi_width, roi_height );
        ROS_DEBUG_STREAM("ROI: x " << x1 << " -- y " << y1 << " -- height " << roi_height << " -- width " << roi_width );

        // -------------------------------------------------------------------------------------------------------
        // Create a sub image of just the block
        cv::Point a1 = cv::Point(x1, y1);
        cv::Point a2 = cv::Point(x2, y2);
        cv::rectangle( cluster_image, a1, a2, cv::Scalar(0, 255, 255), 1, 8);

        // Crop image (doesn't actually copy the data)
        cluster_image_cropped = cluster_image_gray(region_of_interest);

        // -------------------------------------------------------------------------------------------------------
        // Detect edges using canny
        ROS_INFO_STREAM("Detecting edges using canny");

        cv::Mat canny_output;
        cv::Canny( cluster_image_cropped, canny_output, opencv_threshhold, opencv_threshhold*2, 3 );

        // Find contours
        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        ROS_INFO_STREAM("Contours");

        // Draw contours
        cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
        ROS_INFO_STREAM("Drawing contours");

        // Find the largest contour for getting the angle
        double max_contour_length = 0;
        int max_contour_length_i;
        for( size_t i = 0; i< contours.size(); i++ )
        {
          double contour_length = cv::arcLength( contours[i], false );
          if( contour_length > max_contour_length )
          {
            max_contour_length = contour_length;
            max_contour_length_i = i;
          }
          //ROS_DEBUG_STREAM("Contour length = " << contour_length << " of index " << max_contour_length_i);


          cv::Scalar color = cv::Scalar( opencv_rand_gen.uniform(0, 255),
                                         opencv_rand_gen.uniform(0,255),
                                         opencv_rand_gen.uniform(0,255) );
          cv::drawContours( drawing, contours, (int)i, color, 1, 8, hierarchy, 0, cv::Point() );
          //drawContours( image, contours, contourIdx, color, thickness, lineType, hierarchy, maxLevel, offset )

        }

        // -------------------------------------------------------------------------------------------------------
        // Copy largest contour to main image
        cv::Scalar color = cv::Scalar( 0, 255, 0 );
        cv::drawContours( cluster_image, contours, (int)max_contour_length_i, color, 1, 8, hierarchy, 0, a1 );
        //drawContours( image, contours, contourIdx, color, thickness, lineType, hierarchy, maxLevel, offset )

        // -------------------------------------------------------------------------------------------------------
        // Copy largest contour to seperate image
        cv::Mat hough_input = cv::Mat::zeros( canny_output.size(), CV_8UC1 );
        cv::Mat hough_input_color;
        cv::Scalar hough_color = cv::Scalar( 200 );
        cv::drawContours( hough_input, contours, (int)max_contour_length_i, hough_color, 1, 8, hierarchy, 0 );
        cv::cvtColor(hough_input, hough_input_color, CV_GRAY2BGR);

        // -------------------------------------------------------------------------------------------------------
        // Hough Transform
        cv::Mat hough_drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
        std::vector<cv::Vec4i> lines;

        const double hough_rho = 1; // Distance resolution of the accumulator in pixels.
        const double hough_theta = CV_PI/180; // Angle resolution of the accumulator in radians.
        const int hough_threshold = 5; // Accumulator threshold parameter. Only those lines are returned that get enough votes
        const double hough_minLineLength = 10; // Minimum line length. Line segments shorter than that are rejected.
        const double hough_maxLineGap = 10; // Maximum allowed gap between points on the same line to link them.
        cv::HoughLinesP(hough_input, lines, hough_rho, hough_theta, hough_threshold, hough_minLineLength, hough_maxLineGap);

        ROS_WARN_STREAM("Found " << lines.size() << " lines");

        std::vector<double> line_angles;

        // Copy detected lines to the drawing image
        for( size_t i = 0; i < lines.size(); i++ )
        {
          cv::Vec4i line = lines[i];
          ROS_WARN_STREAM("Hough Lines: " << line );
          cv::line( hough_drawing, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                    cv::Scalar(255,255,255), 1, CV_AA);

          // Find angle
          double angle = atan2(line[3] - line[1], line[2] - line[0]); //in radian, degrees: * 180.0 / CV_PI;
          line_angles.push_back(angle);
          ROS_WARN_STREAM("Hough Line angle: " << angle );
        }

        std::vector<int> base_angle;
        std::vector<int> perpendicular_angle;

        // Everything is based on the first angle
        base_angle.push_back(line_angles[0]);

        const double angle_tolerance = .45 * CV_PI; // Just less than 90 degrees

        // Group angles
        for( size_t i = 1; i < line_angles.size(); i++ )
        {
          // Determine if this angle is perpendicular or parallel to the first angle
          if( line_angles[i] < base_angle[0] + angle_tolerance && 
              line_angles[i] > base_angle[0] - angle_tolerance )
          {
            // Qualified for base group
            base_angle.push_back(line_angles[i]);
          }
          else
          {
            // In perpendicular group
            perpendicular_angle.push_back(line_angles[i]);
          }
        }

        // TODO: 
        // make sure there is at least one angle before selecting the base angle!!
        // average the two groups
        // rotate by 90 the second group
        // avarage the two averages
        // done
        // also, consider removing outliers
        // consider doing this several times with random intial angles until the perp and choose the one with
        //   perp and base closest
        // move this into a seperate function
        //
        // also
        // find the center point of the cube
        // create a vector using the overall angle
        // better image processing for different color cubes
        // fix color of stuff
        // determine if copying directly from point cloud is faster

        // -------------------------------------------------------------------------------------------------------
        // GUI Stuff

        // Copy the cluster image to the main image in the top left corner
        if( top_image_overlay_x + canny_output.size.p[1] < image_width )
        {
          const int common_height = 42;
          cv::Rect small_roi_row0 = cv::Rect(top_image_overlay_x, common_height, canny_output.size.p[1],
                                             canny_output.size.p[0]);
          cv::Rect small_roi_row1 = cv::Rect(top_image_overlay_x, common_height,
                                             canny_output.size.p[1], canny_output.size.p[0]);
          cv::Rect small_roi_row2 = cv::Rect(top_image_overlay_x, common_height,
                                             canny_output.size.p[1], canny_output.size.p[0]);
          cv::Rect small_roi_row3 = cv::Rect(top_image_overlay_x, common_height,
                                             canny_output.size.p[1], canny_output.size.p[0]);

          /*
          // Copy cropped image to avoid CopyTo bug ?
          //          cv::Mat small_image = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
          cv::Mat small_image(cluster_image_cropped.rows,cluster_image_cropped.cols,cluster_image_cropped.type());
          ROS_INFO("before copy");
          //          cluster_image_cropped.copyTo(small_image(cv::Rect(0,0,cluster_image_cropped.rows,
          //                                                            cluster_image_cropped.cols)));
          cluster_image_cropped.copyTo(small_image);

          cv::imshow( "ss", small_image );
          ROS_INFO_STREAM("imshow waitkey...");
          cv::waitKey(10000); // 1 sec to allow gui to catch up
          ROS_INFO("after copy");

          small_image.copyTo(          cluster_image(small_roi_row0) );
          */

          drawing.copyTo(              cluster_image(small_roi_row0) );
          hough_input_color.copyTo(    cluster_image(small_roi_row1) );
          hough_drawing.copyTo(        cluster_image(small_roi_row2) );

          top_image_overlay_x += canny_output.size.p[1];
        }

        //        cv::imshow( "Cropped Image", cluster_image_cropped );
        cv::imshow( opencv_window, cluster_image );
        //        cv::imshow( "Contours", drawing );

        ROS_INFO_STREAM("imshow waitkey...");
        cv::waitKey(10); // 1 sec to allow gui to catch up

        /*
        // figure out the position and the orientation of the block
        //float angle = atan(block_size/((xside+yside)/2));
        //float angle = atan( (xmaxy - ymin) / (xmax - yminx) );
        float angle = atan( (xmaxy - xminy) / (xmax - xmin ) );

        //if(yside < block_size)
        //          angle = 0.0;


        ROS_INFO_STREAM("[block detection] FOUND -> xside: " << xside << " yside: " << yside << " angle: " << angle);
        // Then add it to our set
        //addBlock( xmin+(xside)/2.0, ymin+(yside)/2.0, zmax - block_size/2.0, angle);

        // Find the block's center point
        float x_origin = xmin+(xside)/2.0;
        float y_origin = ymin+(yside)/2.0;
        float z_origin = table_height + block_size / 2;

        addBlock( x_origin, y_origin, z_origin, angle);
        */

      }
      else
      {
        ROS_ERROR_STREAM("[block detection] REJECT -> xside: " << xside << " yside: " << yside );
      } // end if size of block

    } // end for each cluser

    ROS_INFO_STREAM("final imshow waitkey...");
    cv::waitKey(10000); // 1 sec to allow gui to catch up
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

    /*
    // Discard noise
    if( block_pose.position.y > 10 || block_pose.position.y < -10 )
    {
    ROS_WARN_STREAM("Rejected block: " << block_pose );
    }
    */

    //ROS_INFO_STREAM("Added block: \n" << block_pose );

    result_.blocks.poses.push_back(block_pose);

  }

  void publishBlockLocation()
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Block"; //_" + boost::lexical_cast<std::string>(block_index);

    // Set the marker type.
    marker.type = visualization_msgs::Marker::CUBE;

    // Set marker size
    marker.scale.x = block_size;
    marker.scale.y = block_size;
    marker.scale.z = block_size - 0.01;

    // Set marker color
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;

    // Keep track of the max number of blocks ever published so that we can delete them if some are lost
    static int max_blocks_published = 0;

    if( result_.blocks.poses.size() > max_blocks_published )
      max_blocks_published = result_.blocks.poses.size();

    ROS_WARN_STREAM("max_blocks_published " << max_blocks_published << " this time we have " << result_.blocks.poses.size() );

    // Loop through all blocks ever and add/delete
    for(unsigned int i = 0; i < max_blocks_published; ++i)
    {
      marker.id = i;

      // Check if we are adding or deleting this block
      if( i < result_.blocks.poses.size() )
      {
        // Set the marker action.  Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose
        marker.pose = result_.blocks.poses[i];
      }
      else
      {
        // We don't have as many blocks as we did at one point
        marker.action = visualization_msgs::Marker::DELETE;
      }

      block_marker_pub_.publish( marker );
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }

  void getXYCoordinates(const int index, const int height, const int width, int& x, int& y)
  {
    //    y = (int)(index / width);
    //    x = index - (y * width);

    x = index % width;
    y = (index - x) / width;
    //    ROS_WARN_STREAM("Converting point " << index << " to x=" << x << " and y=" << y );
  }


  void markPoint(int row, int col, int r, int g, int b)
  {
    cluster_image.at<cv::Vec3b>(row,col)[0] = r;
    cluster_image.at<cv::Vec3b>(row,col)[1] = g;
    cluster_image.at<cv::Vec3b>(row,col)[2] = b;
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_detection_action_server");

  clam_msgs::BlockDetectionServer server("block_detection");
  ros::spin();

  return 0;
}


