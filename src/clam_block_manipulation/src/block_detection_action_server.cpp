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
    ROS_INFO("here");
    boost::shared_ptr<std::vector<int> > filtered_indices(new std::vector<int>);
    ROS_INFO("here");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed);    
    pass.setFilterFieldName("z");
    pass.setFilterLimits(table_height - 0.05, table_height + block_size + 0.05);
    //pass.setFilterLimits(table_height - 0.01, table_height + block_size + 0.02); // DTC
    ROS_INFO("before filtered indicies");
    pass.filter(*filtered_indices);
    ROS_INFO("after filter");

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
    ROS_INFO("kdTree");
    tree->setInputCloud(cloud_transformed, filtered_indices);
    ROS_INFO("after set input cloud");

    // Find the clusters (objects) on the table
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extract;
    //cluster_extract.setClusterTolerance(0.005); // 5mm -  If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. So our recommendation is to just test and try out which value suits your dataset.
    cluster_extract.setClusterTolerance(0.02); // 2cm
    cluster_extract.setMinClusterSize(100);
    cluster_extract.setMaxClusterSize(25000);
    cluster_extract.setSearchMethod(tree);
    //    cluster_extract.setInputCloud(cloud_filtered);
    cluster_extract.setInputCloud(cloud_transformed);
    ROS_INFO("cluster_Extract");
    cluster_extract.setIndices(filtered_indices);
    ROS_INFO("after cluster extract");
    cluster_extract.extract(cluster_indices);

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

    /*
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

    // Blur image - reduce noise with a 3x3 kernel
    cv::blur( cluster_image_gray, cluster_image_gray, cv::Size(3,3) );

    // -------------------------------------------------------------------------------------------------------
    // GUI Stuff

    // First window
    const char* source_window = "Source";
    cv::namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    cv::imshow( source_window, cluster_image_gray );
    cv::createTrackbar( " Canny thresh:", "Source", &opencv_threshhold, max_opencv_threshhold );

    // Second window
    cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    cv::waitKey(10000); // 1 sec to allow gui to catch up
    */
    // -------------------------------------------------------------------------------------------------------
    // Start processing clusters

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

      ROS_INFO_STREAM("Cluster size - xmin: " << xmin << " xmax: " << xmax << " ymin: " << ymin << " ymax: " << ymax << "\n");

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
          int image_width = 640;
          int image_height = 480;
        */
        int image_width = cloud.width;
        int image_height = cloud.height;
        ROS_WARN_STREAM( "PCL Image height " << image_height << " -- width " << image_width << "\n");

        //cv::Mat block_image = cv::Mat::zeros( image_height, image_width, CV_8UC3 );
        //typedef cv::Mat_<cv::Vec3b> block_image = cv::Mat::zeros( image_height, image_width, CV_8UC3 ); 
        RGBImage block_image = RGBImage::zeros(image_height, image_width); 


        //ROS_INFO_STREAM("cloud points should be 307200 but it is " << cloud.points.size() );

        for(size_t i = 0; i < cluster_indices[c].indices.size(); i++)
        {

          int j = cluster_indices[c].indices[i];

          /*
            for(size_t j = 0; j < cloud.points.size(); ++j)
            {
          */

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
          //          ROS_INFO_STREAM("RGB is " << int(r) << ", " << int(g) << ", " << int(b) << 
          //                          " at ROW,COL " << row << ", " << col);

          if( isnan(row) || isnan(col) )
          {
            ROS_ERROR("is nan");
            continue;
          }
          //        block_image.data[j][0] = r;
          //       block_image.data[j][1] = g;
          //        block_image.data[j][2] = b;

          block_image.at<cv::Vec3b>(row,col)[0] = r;
          block_image.at<cv::Vec3b>(row,col)[1] = g;
          block_image.at<cv::Vec3b>(row,col)[2] = b;
          /*
            block_image.at<cv::Vec3b>(row,col)[0] = 255;
            block_image.at<cv::Vec3b>(row,col)[1] = 255;
            block_image.at<cv::Vec3b>(row,col)[2] = 0;
          */
          //          ROS_INFO_STREAM("done assigning rgb");
        }


        ROS_INFO_STREAM("pre imshow");
        cv::imshow( "Copy conversion", block_image );
        ROS_INFO_STREAM("imshow");
        cv::waitKey(10000); // 50 milisec to allow gui to catch up


        // -------------------------------------------------------------------------------------------------------
        // Find Contours
        /*      
        // Use OpenCV only on the region identified by PCL
        int image_width = cluster_image_gray.size.p[1];
        int image_height = cluster_image_gray.size.p[0];
        ROS_WARN_STREAM( "OpenCV Image height " << image_height << " -- width " << image_width << "\n");

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
        
        // Decide which - min,min or max,max is the closest to the 0,0 in pixel coordinates
        */
        /*        (x1,y1)----------------------------------
         *       |                                        |
         *       |                                        |
         *       |                   ROI                  |
         *       |                                        |
         *       |                                        |
         *       |                                        |
         *       |_________________________________(x2,y2)|
         */
        /*
          ROS_WARN_STREAM("");
          ROS_WARN_STREAM("px_xmin " << px_xmin << " px_xmax: " << px_xmax << " py_ymin: " << py_ymin << " py_ymax: " << py_ymax );

          // Change the frame of reference from the robot to the camera
          int x1 = std::min( px_xmin, px_xmax );
          int y1 = std::min( py_ymin, py_ymax );
          int x2 = std::max( px_xmin, px_xmax );
          int y2 = std::max( py_ymin, py_ymax );          

          ROS_WARN_STREAM("x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);

          // Create ROI parameters
          int roi_width = x2 - x1 + 10;
          int roi_height = y2 - y1 + 10;
          int x_offset = x1;
          int y_offset = x2;

          ROS_WARN_STREAM("ROI: x " << x_offset << " -- y " << y_offset << " -- height " << roi_height << " -- width " << roi_width );

          if( roi_width + x_offset > image_width )
          {
          ROS_ERROR("skipped because width too much");
          continue;
          }
          if( roi_height + y_offset > image_height )
          {
          ROS_ERROR("skipped because height too much");
          continue;
          }

          cv::Rect region_of_interest( x_offset, y_offset, roi_height, roi_width );
          //        cv::Rect region_of_interest(10, 10, 100, 100);

          // Crop image (doesn't actually copy the data)
          cluster_image_cropped = cluster_image_gray(region_of_interest);
          ROS_INFO_STREAM("Cropped");


          // GUI Stuff
          ROS_INFO_STREAM("pre imshow");
          cv::imshow( "Contours", cluster_image_cropped );
          ROS_INFO_STREAM("imshow");
          cv::waitKey(5000); // 50 milisec to allow gui to catch up

          // Detect edges using canny
          cv::Mat canny_output;
          cv::Canny( cluster_image_cropped, canny_output, opencv_threshhold, opencv_threshhold*2, 3 );
          ROS_INFO_STREAM("Canny");

          // Find contours
          vector<vector<cv::Point> > contours;
          vector<cv::Vec4i> hierarchy;
          cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
          ROS_INFO_STREAM("Contours");

          // Draw contours
          cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
          ROS_INFO_STREAM("drawing");

          for( size_t i = 0; i< contours.size(); i++ )
          {
          cv::Scalar color = cv::Scalar( opencv_rand_gen.uniform(0, 255),
          opencv_rand_gen.uniform(0,255),
          opencv_rand_gen.uniform(0,255) );
          cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
          }

          // -------------------------------------------------------------------------------------------------------
          // GUI Stuff
          cv::imshow( "Contours", drawing );
          ROS_INFO_STREAM("imshow");
          cv::waitKey(5000); // 1 sec to allow gui to catch up


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
      }
    }

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

  void getXYCoordinates(const int index, const int height, const int width,
                        int& x, int& y)
  {
    //    y = (int)(index / width);
    //    x = index - (y * width);

    x = index % width;
    y = (index - x) / width;
    //    ROS_WARN_STREAM("Converting point " << index << " to x=" << x << " and y=" << y );
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


