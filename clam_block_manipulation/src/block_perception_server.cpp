/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Detects Cublet blocks on a plane using PCL and OpenCV combined
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseArray.h>
#include <clam_msgs/BlockPerceptionAction.h>

#include <tf/transform_listener.h>

#include <pcl/conversions.h>
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


namespace clam_block_manipulation
{

typedef cv::Mat_<cv::Vec3b> RGBImage;


class BlockPerceptionServer
{
private:

  ros::NodeHandle nh_;

  // Actionlib
  actionlib::SimpleActionServer<clam_msgs::BlockPerceptionAction> action_server_;
  std::string action_name_;

  // Actionlib messages
  clam_msgs::BlockPerceptionFeedback feedback_;
  clam_msgs::BlockPerceptionResult result_;
  clam_msgs::BlockPerceptionGoalConstPtr goal_;

  // ROS Connections
  ros::Subscriber point_cloud_sub_;
  ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
  ros::Publisher plane_pub_; // points that were recognized as part of the table
  ros::Publisher block_pose_pub_; // publishes to the block logic server
  ros::Publisher block_marker_pub_; // shows markers in rviz
  tf::TransformListener tf_listener_;

  // Parameters from goal
  std::string camera_link;
  std::string base_link;
  double block_size;
  double table_height;

  // Frequency of image processing
  static const unsigned int PROCESS_EVERY_NTH = 100;
  unsigned int process_count_;

  // OpenCV data structures
  cv::Mat full_input_image;
  cv::Mat full_input_image_gray;
  cv::Mat output_image;
  cv::Mat cropped_image;

  // OpenCV settings
  int canny_threshold;

  int hough_rho; // Distance resolution of the accumulator in pixels.
  int hough_theta; // Angle resolution of the accumulator in radians.
  int hough_threshold; // Accumulator threshold parameter. Only those lines are returned that get enough votes
  int hough_minLineLength; // Minimum line length. Line segments shorter than that are rejected.
  int hough_maxLineGap; // Maximum allowed gap between points on the same line to link them.

public:

  BlockPerceptionServer(const std::string name) :
    nh_("~"),
    action_server_(name, false),
    action_name_(name)
  {
    // Publish a point cloud of filtered data that was not part of table
    filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish a point cloud of data that was considered part of the plane
    plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);

    // Publish interactive markers for blocks
    block_pose_pub_ = nh_.advertise< geometry_msgs::PoseArray >("block_orientation", 1, true);

    // Publish markers to highlight blocks
    block_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("block_marker", 1);

    // Setup OpenCV stuff
    canny_threshold = 100;

    hough_rho = 2; // Distance resolution of the accumulator in pixels.
    hough_theta = 1; // Angle resolution of the accumulator in fraction of degress (so 1/theta degrees). to be converted to radians
    hough_threshold = 14; // Accumulator threshold parameter. Only those lines are returned that get enough votes
    hough_minLineLength = 13; //10; // Minimum line length. Line segments shorter than that are rejected.
    hough_maxLineGap = 16; // Maximum allowed gap between points on the same line to link them.

    // Initialize how often we process images
    process_count_ = PROCESS_EVERY_NTH;

    // TODO: move this, should be brought in from action goal. temporary!
    base_link = "/base_link";
    camera_link = "/camera_rgb_frame";
    //    camera_link = "/camera_rgb_optical_frame";
    block_size = 0.04;
    //    table_height = 0.001;
    table_height = 0.0;

    // Subscribe to point cloud
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &BlockPerceptionServer::pointCloudCallback, this);

    // Register the goal and feeback callbacks.
    action_server_.registerGoalCallback(boost::bind(&BlockPerceptionServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&BlockPerceptionServer::preemptCB, this));

    action_server_.start();

    // Announce state
    ROS_INFO_STREAM_NAMED("perception", "Server ready.");
    ROS_INFO_STREAM_NAMED("perception", "Waiting for point clouds...");
  }

  ~BlockPerceptionServer()
  {
    cv::destroyAllWindows();
  }

  void goalCB()
  {
    ROS_INFO_STREAM_NAMED("perception","Current scene requested");

    // Accept the new goal and save data
    goal_        = action_server_.acceptNewGoal();
    block_size   = goal_->block_size;
    table_height = goal_->table_height;
    base_link    = goal_->frame;
  }

  // Cancel the perception
  void preemptCB()
  {
    ROS_INFO_NAMED("perception","Preempted");

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
    ROS_INFO_NAMED("perception","\n\n\n");
    ROS_INFO_STREAM_NAMED("perception","Processing new point cloud");

    // ---------------------------------------------------------------------------------------------
    // Start making result
    result_.blocks.poses.clear();    // Clear last block perception result
    result_.blocks.header.stamp = pointcloud_msg->header.stamp;
    result_.blocks.header.frame_id = base_link;

    // Basic point cloud conversions ---------------------------------------------------------------

    // Convert from ROS to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*pointcloud_msg, cloud);

    // Make new point cloud that is in our working frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Transform to whatever frame we're working in, probably the arm's base frame, ie "base_link"
    ROS_INFO_STREAM_NAMED("perception","Waiting for transform...");
    ros::spinOnce();
    tf_listener_.waitForTransform(base_link, cloud.header.frame_id, pointcloud_msg->header.stamp, ros::Duration(2.0));

    if(!pcl_ros::transformPointCloud(base_link, cloud, *cloud_transformed, tf_listener_))
    {
      if( process_count_ > 1 ) // the first time we can ignore it
        ROS_ERROR_STREAM_NAMED("perception","Error converting to desired frame");

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
    ROS_ERROR_STREAM_NAMED("perception","0 points left");
    return;
    }
    else
    {
    ROS_INFO_STREAM_NAMED("perception","Filtered, %d points left", (int) cloud_filtered->points.size());
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
      ROS_ERROR_STREAM_NAMED("perception","Could not estimate a planar model for the given dataset.");
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
      ROS_INFO_STREAM_NAMED("perception", "Model coefficients: " << model_coefficients->values[0] << " "
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
    ROS_INFO_STREAM_NAMED("perception","Extracting...");
    cluster_extract.extract(cluster_indices);
    ROS_INFO_STREAM_NAMED("perception","after cluster extract");

    // Publish point cloud data
    //    filtered_pub_.publish(cloud_filtered);
    //    plane_pub_.publish(cloud_plane);

    ROS_WARN_STREAM_NAMED("perception","Number indicies/clusters: " << cluster_indices.size() );

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

      ROS_INFO_STREAM_NAMED("perception","Finished ---------------------------------------------- ");
    }
    else
    {
      ROS_INFO_STREAM_NAMED("perception","Couldn't find any blocks this iteration!");
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
    ROS_INFO_STREAM_NAMED("perception","Converting image to OpenCV format");

    try
    {
      sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
      //const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr  cloud_transformed,
      sensor_msgs::PointCloud2 temp_cloud;
      pcl::toROSMsg(*cloud_transformed, temp_cloud);
      pcl::toROSMsg (temp_cloud, *image_msg);
      cv_bridge::CvImagePtr input_bridge = cv_bridge::toCvCopy(image_msg, "rgb8");
      full_input_image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR_STREAM_NAMED("perception","[calibrate] Failed to convert image");
      return;
    }

    // -------------------------------------------------------------------------------------------------------
    // Process Image

    // Convert image to gray
    cv::cvtColor( full_input_image, full_input_image_gray, CV_BGR2GRAY );
    //cv::adaptiveThreshold( full_input_image, full_input_image_gray, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,5,10);

    // Blur image - reduce noise with a 3x3 kernel
    cv::blur( full_input_image_gray, full_input_image_gray, cv::Size(3,3) );

    ROS_INFO_STREAM_NAMED("perception","Finished coverting");

    // -------------------------------------------------------------------------------------------------------
    // Check OpenCV and PCL image height for errors
    int image_width = cloud.width;
    int image_height = cloud.height;
    ROS_DEBUG_STREAM( "PCL Image height " << image_height << " -- width " << image_width << "\n");
    int image_width_cv = full_input_image.size.p[1];
    int image_height_cv = full_input_image.size.p[0];
    ROS_DEBUG_STREAM( "OpenCV Image height " << image_height_cv << " -- width " << image_width_cv << "\n");

    if( image_width != image_width_cv || image_height != image_height_cv )
    {
      ROS_ERROR_STREAM_NAMED("perception","PCL and OpenCV image heights/widths do not match!");
      return;
    }

    // -------------------------------------------------------------------------------------------------------
    // GUI Stuff

    // First window
    const char* opencv_window = "Source";
    /*
      cv::namedWindow( opencv_window, CV_WINDOW_AUTOSIZE );
      cv::imshow( opencv_window, full_input_image_gray );
    */


    //    while(true)  // use this when we want to tweak the image
    {

      output_image = full_input_image.clone();

      // -------------------------------------------------------------------------------------------------------
      // Start processing clusters
      ROS_INFO_STREAM_NAMED("perception","Finding min/max in x/y axis");

      int top_image_overlay_x = 0; // tracks were to copyTo the mini images

      // for each cluster, see if it is a block
      for(size_t c = 0; c < cluster_indices.size(); ++c)
      {
        ROS_INFO_STREAM_NAMED("perception","\n\n");
        ROS_INFO_STREAM_NAMED("perception","On cluster " << c);

        // find the outer dimensions of the cluster
        double xmin = 0; double xmax = 0;
        double ymin = 0; double ymax = 0;

        // also remember each min & max's correponding other coordinate (not needed for z)
        double xminy = 0; double xmaxy = 0;
        double yminx = 0; double ymaxx = 0;

        // also remember their corresponding indice
        int xmini = 0; int xmaxi = 0;
        int ymini = 0; int ymaxi = 0;

        // loop through and find all min/max of x/y
        for(size_t i = 0; i < cluster_indices[c].indices.size(); i++)
        {
          int j = cluster_indices[c].indices[i];

          // Get RGB from point cloud
          pcl::PointXYZRGB p = cloud_transformed->points[j];

          double x = p.x;
          double y = p.y;

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

        ROS_DEBUG_STREAM_NAMED("perception","Cluster size - xmin: " << xmin << " xmax: " << xmax << " ymin: " << ymin << " ymax: " << ymax);
        ROS_DEBUG_STREAM_NAMED("perception","Cluster size - xmini: " << xmini << " xmaxi: " << xmaxi << " ymini: " << ymini << " ymaxi: " << ymaxi);

        // ---------------------------------------------------------------------------------------------
        // Check if these dimensions make sense for the block size specified
        double xside = xmax-xmin;
        double yside = ymax-ymin;

        const double tol = 0.01; // 1 cm error tolerance

        // In order to be part of the block, xside and yside must be between
        // blocksize and blocksize*sqrt(2)
        if(xside > block_size-tol &&
           xside < block_size*sqrt(2)+tol &&
                   yside > block_size-tol &&
          yside < block_size*sqrt(2)+tol )
        {

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

          ROS_DEBUG_STREAM_NAMED("perception","px_xmin " << px_xmin << " px_xmax: " << px_xmax << " py_ymin: " << py_ymin << " py_ymax: " << py_ymax );

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

          ROS_DEBUG_STREAM_NAMED("perception","x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);

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

          ROS_DEBUG_STREAM_NAMED("perception","After Fudge Factor - x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);

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
          ROS_DEBUG_STREAM_NAMED("perception","ROI: x " << x1 << " -- y " << y1 << " -- height " << roi_height << " -- width " << roi_width );

          // -------------------------------------------------------------------------------------------------------
          // Find paramters of the block in pixel coordiantes
          int block_center_x = x1 + 0.5*roi_width;
          int block_center_y = y1 + 0.5*roi_height;
          int block_center_z = block_size / 2; // TODO: make this better
          const cv::Point block_center = cv::Point( block_center_x, block_center_y );

          // -------------------------------------------------------------------------------------------------------
          // Create a sub image of just the block
          cv::Point a1 = cv::Point(x1, y1);
          cv::Point a2 = cv::Point(x2, y2);
          cv::rectangle( output_image, a1, a2, cv::Scalar(0, 255, 255), 1, 8);

          // Crop image (doesn't actually copy the data)
          cropped_image = full_input_image_gray(region_of_interest);

          // -------------------------------------------------------------------------------------------------------
          // Detect edges using canny
          ROS_INFO_STREAM_NAMED("perception","Detecting edges using canny");

          // Find edges
          cv::Mat canny_output;
          cv::Canny( cropped_image, canny_output, canny_threshold, canny_threshold*2, 3 );

          // Get mini window stats
          const int mini_width = canny_output.size.p[1];
          const int mini_height = canny_output.size.p[0];
          const cv::Size mini_size = canny_output.size();
          const cv::Point mini_center = cv::Point( mini_width/2, mini_height/2 );

          // Find contours
          std::vector<std::vector<cv::Point> > contours;
          std::vector<cv::Vec4i> hierarchy;
          cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
          ROS_INFO_STREAM_NAMED("perception","Contours");

          // Draw contours
          cv::Mat drawing = cv::Mat::zeros( mini_size, CV_8UC3 );
          ROS_INFO_STREAM_NAMED("perception","Drawing contours");

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
            //ROS_DEBUG_STREAM_NAMED("perception","Contour length = " << contour_length << " of index " << max_contour_length_i);


            cv::Scalar color = cv::Scalar( (30 + i*10) % 255, (30 + i*10) % 255, (30 + i*10) % 255);
            cv::drawContours( drawing, contours, (int)i, color, 1, 8, hierarchy, 0, cv::Point() );
            //drawContours( image, contours, contourIdx, color, thickness, lineType, hierarchy, maxLevel, offset )

          }

          // -------------------------------------------------------------------------------------------------------
          // Copy largest contour to main image
          cv::Scalar color = cv::Scalar( 0, 255, 0 );
          cv::drawContours( output_image, contours, (int)max_contour_length_i, color, 1, 8, hierarchy, 0, a1 );
          //drawContours( image, contours, contourIdx, color, thickness, lineType, hierarchy, maxLevel, offset )

          // -------------------------------------------------------------------------------------------------------
          // Copy largest contour to seperate image
          cv::Mat hough_input = cv::Mat::zeros( mini_size, CV_8UC1 );
          cv::Mat hough_input_color;
          cv::Scalar hough_color = cv::Scalar( 200 );
          cv::drawContours( hough_input, contours, (int)max_contour_length_i, hough_color, 1, 8, hierarchy, 0 );
          cv::cvtColor(hough_input, hough_input_color, CV_GRAY2BGR);

          // -------------------------------------------------------------------------------------------------------
          // Hough Transform
          cv::Mat hough_drawing = cv::Mat::zeros( mini_size, CV_8UC3 );
          std::vector<cv::Vec4i> lines;


          ROS_DEBUG_STREAM_NAMED("perception","hough_rho " << hough_rho << " hough_theta " << hough_theta <<
                                 " theta_converted " << (1/hough_theta)*CV_PI/180 << " hough_threshold " <<
                                 hough_threshold << " hough_minLineLength " << hough_minLineLength <<
                                 " hough_maxLineGap " << hough_maxLineGap );

          cv::HoughLinesP(hough_input, lines, hough_rho, (1/hough_theta)*CV_PI/180, hough_threshold, hough_minLineLength, hough_maxLineGap);

          ROS_WARN_STREAM_NAMED("perception","Found " << lines.size() << " lines");

          std::vector<double> line_angles;

          // Copy detected lines to the drawing image
          for( size_t i = 0; i < lines.size(); i++ )
          {
            cv::Vec4i line = lines[i];
            cv::line( hough_drawing, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                      cv::Scalar(255,255,255), 1, CV_AA);

            // Error check
            if(line[3] - line[1] == 0 && line[2] - line[0] == 0)
            {
              ROS_ERROR_STREAM_NAMED("perception","Line is actually two points at the origin, unable to calculate. TODO: handle better?");
              continue;
            }

            // Find angle
            double line_angle = atan2(line[3] - line[1], line[2] - line[0]); //in radian, degrees: * 180.0 / CV_PI;
            // Reverse angle direction if negative
            if( line_angle < 0 )
            {
              line_angle += CV_PI;
            }
            line_angles.push_back(line_angle);
            ROS_DEBUG_STREAM_NAMED("perception","Hough Line angle: " << line_angle * 180.0 / CV_PI;);
          }

          double block_angle = 0; // the overall result of the block's angle

          // Everything is based on the first angle
          if( line_angles.size() == 0 ) // make sure we have at least 1 angle
          {
            ROS_ERROR_STREAM_NAMED("perception","No lines were found for this cluster, unable to calculate block angle");
          }
          else
          {
            calculateBlockAngle( line_angles, block_angle );
          }

          // -------------------------------------------------------------------------------------------------------
          // Draw chosen angle
          ROS_INFO_STREAM_NAMED("perception","Using block angle " << block_angle*180.0/CV_PI);

          // Draw chosen angle on mini image
          cv::Mat angle_drawing = cv::Mat::zeros( mini_size, CV_8UC3 );
          int line_length = 0.5*double(mini_width); // have the line go 1/4 across the screen
          int new_x = mini_center.x + line_length*cos( block_angle );
          int new_y = mini_center.y + line_length*sin( block_angle );
          ROS_INFO_STREAM("Origin (" << mini_center.x << "," << mini_center.y << ") New (" << new_x << "," << new_y <<
                          ") length " << line_length << " angle " << block_angle <<
                          " mini width " << mini_width << " mini height " << mini_height);
          cv::Point angle_point = cv::Point(new_x, new_y);
          cv::line( angle_drawing, mini_center, angle_point, cv::Scalar(255,255,255), 1, CV_AA);

          // Draw chosen angle on contours image
          cv::line( hough_drawing, mini_center, angle_point, cv::Scalar(255,0, 255), 1, CV_AA);

          // Draw chosen angle on main image
          line_length = 0.75 * double(mini_width); // have the line go 1/2 across the box
          new_x = block_center_x + line_length*cos( block_angle );
          new_y = block_center_y + line_length*sin( block_angle );
          ROS_INFO_STREAM_NAMED("perception",block_center_x << ", " << block_center_y << ", " << new_x << ", " << new_y);
          angle_point = cv::Point(new_x, new_y);
          cv::line( output_image, block_center, angle_point, cv::Scalar(255,0,255), 2, CV_AA);


          // -------------------------------------------------------------------------------------------------------
          // Get world coordinates

          // Find the block's center point
          double world_x1 = xmin+(xside)/2.0;
          double world_y1 = ymin+(yside)/2.0;
          double world_z1 = table_height + block_size / 2;

          // Convert pixel coordiantes back to world coordinates
          double world_x2 = cloud_transformed->at(new_x, new_y).x;
          double world_y2 = cloud_transformed->at(new_x, new_y).y;
          double world_z2 = world_z1; // is this even necessary?

          // Get angle from two world coordinates...
          double world_theta = abs( atan2(world_y2 - world_y1, world_x2 - world_x1) );

          // Attempt to make all angles point in same direction
          makeAnglesUniform( world_theta );

          // -------------------------------------------------------------------------------------------------------
          // GUI Stuff

          // Copy the cluster image to the main image in the top left corner
          if( top_image_overlay_x + mini_width < image_width )
          {
            const int common_height = 42;
            cv::Rect small_roi_row0 = cv::Rect(top_image_overlay_x, common_height*0, mini_width, mini_height);
            cv::Rect small_roi_row1 = cv::Rect(top_image_overlay_x, common_height*1, mini_width, mini_height);
            cv::Rect small_roi_row2 = cv::Rect(top_image_overlay_x, common_height*2, mini_width, mini_height);
            cv::Rect small_roi_row3 = cv::Rect(top_image_overlay_x, common_height*3, mini_width, mini_height);

            drawing.copyTo(              output_image(small_roi_row0) );
            hough_input_color.copyTo(    output_image(small_roi_row1) );
            hough_drawing.copyTo(        output_image(small_roi_row2) );
            angle_drawing.copyTo(        output_image(small_roi_row3) );

            top_image_overlay_x += mini_width;
          }

          // figure out the position and the orientation of the block
          //double angle = atan(block_size/((xside+yside)/2));
          //double angle = atan( (xmaxy - xminy) / (xmax - xmin ) );
          // Then add it to our set
          //addBlock( xmin+(xside)/2.0, ymin+(yside)/2.0, zmax - block_size/2.0, angle);
          //ROS_INFO_STREAM_NAMED("perception","FOUND -> xside: " << xside << " yside: " << yside << " angle: " << block_angle);


          addBlock( world_x1, world_y1, world_z1, world_theta );
          //addBlock( block_center_x, block_center_y, block_center_z, block_angle);
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("perception","REJECT -> xside: " << xside << " yside: " << yside );
        } // end if size of block

      } // end for each cluser

      cv::imshow( opencv_window, output_image );
      cv::createTrackbar( "Canny thresh:", opencv_window, &canny_threshold, 255 );
      cv::createTrackbar( "Resolution:", opencv_window, &hough_rho, 10 );
      cv::createTrackbar( "Theta:", opencv_window, &hough_theta, 10 );
      cv::createTrackbar( "Threshold:", opencv_window, &hough_threshold, 100 );
      cv::createTrackbar( "minLineLength:", opencv_window, &hough_minLineLength, 100 );
      cv::createTrackbar( "maxLineGap:", opencv_window, &hough_maxLineGap, 100 );

      ROS_INFO_STREAM_NAMED("perception","final imshow waitkey...");
      cv::waitKey(1000); // 1 sec to allow gui to catch up
    }
  }

  void addBlock(double x, double y, double z, double angle)
  {
    ROS_INFO_STREAM_NAMED("perception","Adding block in world coordinates (" << x << "," << y << "," << z << ") and angle " << angle );

    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;

    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(float(angle), Eigen::Vector3f(0,0,1)));

    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();

    /*
    // Discard noise
    if( block_pose.position.y > 10 || block_pose.position.y < -10 )
    {
    ROS_WARN_STREAM_NAMED("perception","Rejected block: " << block_pose );
    }
    */

    ROS_INFO_STREAM_NAMED("perception","Added block: \n" << block_pose );

    result_.blocks.poses.push_back(block_pose);
  }

  void publishBlockLocation()
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "Block"; //_" + boost::lexical_cast<std::string>(block_index);

    // Set the marker type.
    marker.type = visualization_msgs::Marker::CUBE;

    // Set marker size
    marker.scale.x = block_size - 0.001;
    marker.scale.y = block_size - 0.001;
    marker.scale.z = block_size - 0.001;

    // Set marker color
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    // Keep track of the max number of blocks ever published so that we can delete them if some are lost
    static int max_blocks_published = 0;

    if( result_.blocks.poses.size() > max_blocks_published )
      max_blocks_published = result_.blocks.poses.size();

    ROS_WARN_STREAM_NAMED("perception","max_blocks_published " << max_blocks_published << " this time we have " << result_.blocks.poses.size() );

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
    //    ROS_WARN_STREAM_NAMED("perception","Converting point " << index << " to x=" << x << " and y=" << y );
  }


  // Find a line perpendicular to the block in the x/y plane
  // @param lines - the list of contours that makeup the outline of the block
  // @param block_angle - the resulting angle of the block
  void calculateBlockAngle( std::vector<double> line_angles, double &block_angle )
  {
    std::vector<std::pair<double,double> > vector_score_angle;

    double score;
    double angle;

    // Group the angles repeatidly, using different base angles each time
    for( size_t i = 0; i < line_angles.size(); i++ )
    {
      groupAngles( line_angles, i, score, angle );
      vector_score_angle.push_back(std::pair<double,double>(score,angle));
    }

    double min_score = 2*CV_PI; // really big angle
    double best_angle = 0;

    // Find the angle with the best (lowest)
    for( size_t i = 0; i < vector_score_angle.size(); ++i)
    {
      if( vector_score_angle[i].first < min_score )
      {
        min_score = vector_score_angle[i].first;
        best_angle = vector_score_angle[i].second;
      }
    }

    ROS_INFO_STREAM_NAMED("perception","Chose angle " << best_angle*180.0/CV_PI << " with score " << min_score);

    // return the best angle found
    block_angle = best_angle;
  }

  void groupAngles( std::vector<double> line_angles, int base_angle_id, double &score, double &angle )
  {
    std::vector<double> parallel_angles;
    std::vector<double> perpendicular_angles;

    // Choose base angle
    double base_angle = line_angles[base_angle_id];
    parallel_angles.push_back(base_angle);

    // Use the positive perpendicular angle
    double perpendicular_angle;
    if( base_angle > 0.5*CV_PI )
      perpendicular_angle = base_angle - 0.5 * CV_PI; // rotate 90 degrees
    else
      perpendicular_angle = base_angle + 0.5 * CV_PI; // rotate 90 degrees

    ROS_WARN_STREAM_NAMED("perception","Base Angle: " << base_angle * 180.0 / CV_PI;);
    ROS_WARN_STREAM_NAMED("perception","Perp Angle: " << perpendicular_angle * 180.0 / CV_PI;);

    // Must be within 30 degrees of an angle to be considered not an outlier
    //    const double angle_tolerance = 0.166666 * CV_PI;
    const double angle_tolerance =  3*CV_PI/180; // radians = 5 degrees

    // Group angles
    for( size_t i = 1; i < line_angles.size(); i++ )
    {
      // Skip the base_angle_id because it has already been categorized
      if( i == base_angle_id )
        continue;

      // Determine if this angle is perpendicular or parallel to the first angle
      if( line_angles[i] < base_angle + angle_tolerance &&
          line_angles[i] > base_angle - angle_tolerance )
      {
        // Qualified for base group
        parallel_angles.push_back(line_angles[i]);
        ROS_WARN_STREAM_NAMED("perception","In parallel group " << line_angles[i]* 180.0 / CV_PI;);
      }
      else if( line_angles[i] < perpendicular_angle + angle_tolerance &&
               line_angles[i] > perpendicular_angle - angle_tolerance )
      {
        // In perpendicular group
        perpendicular_angles.push_back(line_angles[i]);
        ROS_WARN_STREAM_NAMED("perception","In perpendicular group " << line_angles[i]* 180.0 / CV_PI;);
      }
      else // rejected
      {
        ROS_ERROR_STREAM_NAMED("perception","Angle rejected for being out of range " << line_angles[i]* 180.0 / CV_PI;);
      }
    }

    // Average both groups
    double parallel_avg = average_vector( parallel_angles );
    double perpendicular_avg = average_vector( perpendicular_angles );

    ROS_INFO_STREAM_NAMED("perception","parallel avg " << parallel_avg*180.0/CV_PI << " perp avg " << perpendicular_avg*180.0/CV_PI << " num perp " << perpendicular_angles.size());

    if( perpendicular_angles.empty() ) // nothing was grouped into second group...
    {
      score = CV_PI; // a bad score
      angle = parallel_avg; // just use one group of angles
    }
    else
    {
      // Score this angle based on how perpendicular the two averages are

      double perpendicular_avg_rotated;

      // Score by taking the difference of the smallest from biggest
      if( parallel_avg > perpendicular_avg_rotated )
      {
        perpendicular_avg_rotated = perpendicular_avg + .5 * CV_PI; // rotate 90 degrees
        score = std::abs(parallel_avg - perpendicular_avg_rotated);
      }
      else
      {
        perpendicular_avg_rotated = perpendicular_avg - .5 * CV_PI; // rotate 90 degrees
        score = std::abs(perpendicular_avg_rotated - parallel_avg);
      }

      ROS_DEBUG_STREAM_NAMED("perception","Parallel avg = " << parallel_avg*180.0/CV_PI << " and transformed perp avg " << perpendicular_avg_rotated*180.0/CV_PI);

      // Calculate the mean of the parallel and perpendicular
      angle = (parallel_avg + perpendicular_avg + .5 * CV_PI) / 2;
    }

    ROS_DEBUG_STREAM_NAMED("perception","Averaged angle " << angle*180.0/CV_PI << " with score " << score );
  }


  // Find a line perpendicular to the block in the x/y plane
  // @param lines - the list of contours that makeup the outline of the block
  // @param block_angle - the resulting angle of the block
  void calculateBlockAngleSimple( std::vector<double> line_angles, double &block_angle )
  {
    const double angle_tolerance =  45*CV_PI/180; // radians = 45 degrees
    std::vector<double> parallel_angles;

    // Get base angle
    double base_angle = line_angles[0];
    parallel_angles.push_back(base_angle);
    ROS_DEBUG_STREAM_NAMED("perception","Base angle " << base_angle*180.0/CV_PI;);

    for( size_t i = 1; i < line_angles.size(); i++ )
    {
      // Determine if this angle is perpendicular or parallel to the first angle
      if( line_angles[i] < base_angle + angle_tolerance &&
          line_angles[i] > base_angle - angle_tolerance )
      {
        parallel_angles.push_back(line_angles[i]);
        ROS_DEBUG_STREAM_NAMED("perception","Not flipped " << line_angles[i]* 180.0 / CV_PI;);
      }
      else
      {
        double angle_converted;
        if( base_angle < line_angles[i] )
        {
          angle_converted = line_angles[i] - 0.5*CV_PI;
        }
        else
        {
          angle_converted = line_angles[i] + 0.5*CV_PI;
        }
        parallel_angles.push_back(angle_converted);
        ROS_DEBUG_STREAM_NAMED("perception","Flipped " << angle_converted*180.0/CV_PI;);
      }
    }

    // Average all the angles
    block_angle = average_vector( parallel_angles );
    ROS_INFO_STREAM_NAMED("perception","Average angle: " << block_angle*180.0/CV_PI);
  }

  double average_vector(std::vector<double>& input)
  {
    ROS_DEBUG_STREAM_NAMED("perception","Averaging angles...");
    double sum = 0.0;
    for(std::vector<double>::const_iterator num_it = input.begin(); num_it < input.end(); ++num_it)
    {
      ROS_DEBUG_STREAM_NAMED("perception","  Adding angle " << *num_it*180.0/CV_PI);
      sum += *num_it;
    }
    ROS_DEBUG_STREAM_NAMED("perception","  Angle sum " << sum*180.0/CV_PI << " divided by " << input.size() );
    return sum / input.size();
  }

  // Just a temp code placeholder
  void convertPCLtoOpenCVImage()
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
    //ROS_DEBUG_STREAM_NAMED("perception","RGB is " << int(r) << ", " << int(g) << ", " << int(b) << " at ROW,COL " << row << ", " << col);

    if( isnan(row) || isnan(col) )
    {
    ROS_ERROR_STREAM_NAMED("perception","is nan");
    continue;
    }

    block_image.at<cv::Vec3b>(row,col)[0] = r;
    block_image.at<cv::Vec3b>(row,col)[1] = g;
    block_image.at<cv::Vec3b>(row,col)[2] = b;
    //ROS_DEBUG_STREAM_NAMED("perception","done assigning rgb");
    }


    ROS_DEBUG_STREAM_NAMED("perception","pre imshow");
    cv::imshow( "Copy conversion", block_image );
    ROS_DEBUG_STREAM_NAMED("perception","imshow");
    cv::waitKey(1000); // 50 milisec to allow gui to catch up
    */

  }

  void makeAnglesUniform( double& world_theta )
  {
    // Filter angle to be in one direction
    static const double GOAL_ANGLE = CV_PI;

    ROS_INFO_STREAM("Goal angle " << GOAL_ANGLE*180.0/CV_PI);
    ROS_INFO_STREAM("Orig angle " << world_theta*180.0/CV_PI);

    std::vector<double> new_angles;
    new_angles.push_back( world_theta + CV_PI / 2 ); // increase by 90d
    new_angles.push_back( world_theta + CV_PI     ); // increase by 180d
    new_angles.push_back( world_theta - CV_PI / 2 ); // decrease by 90d
    new_angles.push_back( world_theta - CV_PI     ); // decrease by 180d
    new_angles.push_back( world_theta             ); // keep as is

    double best_difference = CV_PI*2; // really big number
    double best_angle;
    
    for( std::vector<double>::const_iterator angle_it = new_angles.begin(); 
         angle_it < new_angles.end(); ++angle_it)
    {
      if( abs( GOAL_ANGLE - *angle_it ) < best_difference )
      {
        best_difference = abs( GOAL_ANGLE - *angle_it );
        best_angle = *angle_it;
      }
    }

    world_theta = best_angle;

    ROS_INFO_STREAM("New angle " << world_theta*180.0/CV_PI);
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_perception_server");

  clam_block_manipulation::BlockPerceptionServer server("block_perception");
  ros::spin();

  return 0;
}



