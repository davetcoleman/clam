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
 * Author: Michael Ferguson, Helen Oleynikova
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

//#include <iostream> // TODO: remove this dep

namespace clam_msgs
{

class BlockDetectionServer
{
private:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<clam_msgs::BlockDetectionAction> as_;
    std::string action_name_;
    clam_msgs::BlockDetectionFeedback feedback_;
    clam_msgs::BlockDetectionResult result_;
    clam_msgs::BlockDetectionGoalConstPtr goal_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
    ros::Publisher plane_pub_; // points that were recognized as part of the table
    ros::Publisher block_pose_pub_; // publishes to the interactive_manipulation_action_server
    tf::TransformListener tf_listener_;

    // Parameters from goal
    std::string arm_link;
    double block_size;
    double table_height;

public:
    BlockDetectionServer(const std::string name) :
        nh_("~"), as_(name, false), action_name_(name)
    {
        // Load parameters from the server.

        // Register the goal and feeback callbacks.
        as_.registerGoalCallback(boost::bind(&BlockDetectionServer::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&BlockDetectionServer::preemptCB, this));

        as_.start();

        // Subscribe to point cloud
        point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &BlockDetectionServer::cloudCb, this);

        // Publish a point cloud of filtered data that was not part of table
        filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

        // Publish a point cloud of data that was considered part of the plane
        plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);

        // Publish interactive markers for blocks
        block_pose_pub_ = nh_.advertise< geometry_msgs::PoseArray >("/clam_blocks", 1, true);
    }

    void goalCB()
    {
        ROS_INFO("[block detection] Starting detection");

        // Clear last block detection result
        result_.blocks.poses.clear();

        // Accept the new goal and save data
        goal_ = as_.acceptNewGoal();
        block_size = goal_->block_size;
        table_height = goal_->table_height;
        arm_link = goal_->frame;

        // Start making result
        result_.blocks.header.frame_id = arm_link;
    }

    // Cancel the detection
    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());

        // set the action state to preempted
        as_.setPreempted();
    }

    // Proccess the point clouds
    void cloudCb( const sensor_msgs::PointCloud2ConstPtr& msg )
    {
        // Only do this if we're actually actively working on a goal.
        if(!as_.isActive())
            return;

        result_.blocks.header.stamp = msg->header.stamp;

        // Basic point cloud conversions ---------------------------------------------------------------

        // Convert from ROS to PCL
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Make new point cloud that is in our working frame
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Transform to whatever frame we're working in, probably the arm's base frame, ie "base_link"
        tf_listener_.waitForTransform(std::string(arm_link), cloud.header.frame_id,
                                      cloud.header.stamp, ros::Duration(1.0));
        if(!pcl_ros::transformPointCloud(std::string(arm_link), cloud, *cloud_transformed, tf_listener_))
        {
            ROS_ERROR("Error converting to desired frame");
            return;
        }

        // Limit to things we think are roughly at the table height ------------------------------------
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud_transformed);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(table_height - 0.05, table_height + block_size + 0.05);
        //pass.setFilterLimits(table_height - 0.01, table_height + block_size + 0.02); // DTC
        pass.filter(*cloud_filtered);

        // Check if more points remain
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
              }*/
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

        if(result_.blocks.poses.size() > 0)
        {
            as_.setSucceeded(result_);
            block_pose_pub_.publish(result_.blocks);
            ROS_INFO("[block detection] Finished");
        }
        else
        {
            ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
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

        // Discard noise
        if( block_pose.position.y > 10 || block_pose.position.y < -10 )
        {
            ROS_WARN_STREAM("Rejected block: " << block_pose );
        }

        ROS_INFO_STREAM("Added block: " << block_pose );

        result_.blocks.poses.push_back(block_pose);
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

