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
   Desc:   
*/

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>

#include <clam_msgs/BlockLogicAction.h>
#include <geometry_msgs/PoseArray.h>

using namespace visualization_msgs;

namespace clam_msgs
{

class BlockLogicServer
{
private:
  ros::NodeHandle nh_;

  interactive_markers::InteractiveMarkerServer interactive_m_server_;

  actionlib::SimpleActionServer<clam_msgs::BlockLogicAction> action_server_;
  std::string action_name_;

  clam_msgs::BlockLogicFeedback     feedback_;
  clam_msgs::BlockLogicResult       action_result_;
  clam_msgs::BlockLogicGoalConstPtr goal_;

  // Publishers/Subscribers
  ros::Publisher marker_pub_; // for showing the blocks in rviz
  ros::Subscriber block_sub_;
  ros::Publisher pick_place_pub_;

  geometry_msgs::Pose old_pose_;

  geometry_msgs::PoseArrayConstPtr msg_;
  bool initialized_;

  // Parameters from goal
  std::string arm_link;
  double      block_size;

  // Parameters from server
  double bump_size;

  static const double CUBE_Z_HEIGHT = 0.2;

public:

  BlockLogicServer(const std::string name) :
    nh_("~"), 
    interactive_m_server_("block_controls"), 
    action_server_(name, false), 
    action_name_(name), 
    initialized_(false), 
    block_size(0.4)
  {
    // Load parameters from the server.
    nh_.param<double>("bump_size", bump_size, 0.0); // original 0.005

    // ---------------------------------------------------------------------------------------------
    // Read in all seen blocks
    block_sub_ = nh_.subscribe("/cube_block_poses", 1, &BlockLogicServer::addBlocks, this);

    // ---------------------------------------------------------------------------------------------
    // Publish location of blocks??
    pick_place_pub_ = nh_.advertise< geometry_msgs::PoseArray >("/pick_place", 1, true);

    // ---------------------------------------------------------------------------------------------
    // Rviz Visualizations
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // ---------------------------------------------------------------------------------------------
    // Register the goal and feeback callbacks for action server
    action_server_.registerGoalCallback(boost::bind(&BlockLogicServer::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&BlockLogicServer::preemptCB, this));
    action_server_.start();
  }

  void goalCB()
  {
    // ---------------------------------------------------------------------------------------------
    // Accept the new goal
    goal_ = action_server_.acceptNewGoal();

    ROS_INFO("[block logic] Received goal! %f, %s", goal_->block_size, goal_->frame.c_str());

    block_size = goal_->block_size;
    arm_link = goal_->frame;

    if (initialized_)
    {
      addBlocks(msg_);
    }

    // --------------------------------------------------------------------------------------------
    // Start pose - choose one that is preferrably not in the goal region
    geometry_msgs::Pose start_pose;
    bool found_pose = false;

    // Check if there is only 1 block detected
    if( !msg_->poses.size() )
    {
      // no blocks, what to do?
      ROS_WARN("[block logic] No blocks found");
    }
    else if( msg_->poses.size() == 1 )
    {
      start_pose = msg_->poses[0];
      found_pose = true;
      ROS_INFO("[block logic] Only 1 block, using it");
    }
    else
    {
      // Search for block that meets our criteria
      for(int i = 0; i < msg_->poses.size(); ++i)
      {
        if( msg_->poses[i].position.y > -0.12 && msg_->poses[i].position.y < 0.2 ) // start of goal region
        {
          start_pose = msg_->poses[i];
          found_pose = true;
          ROS_INFO_STREAM("[block logic] Chose this block:\n" << start_pose);
          break;
        }
      }
      if( !found_pose )
      {
        ROS_INFO("[block logic] Did not find a good block, default to first");
        start_pose = msg_->poses[0];
        found_pose = true;
      }
    }

    // --------------------------------------------------------------------------------------------
    // End pose is just chosen place on board
    geometry_msgs::Pose end_pose;
    end_pose.orientation = msg_->poses[0].orientation; // keep the same orientation
    end_pose.position.x = 0.225;
    end_pose.position.y = 0.18;
    end_pose.position.z = msg_->poses[0].position.z;

    // --------------------------------------------------------------------------------------------
    // Move that block
    if( found_pose )
    {
      moveBlock(start_pose, end_pose);
    }
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    action_server_.setPreempted();
  }

  void addBlocks(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    interactive_m_server_.clear();
    interactive_m_server_.applyChanges();

    ROS_INFO("[block logic] Got block detection callback. Adding blocks.");
    geometry_msgs::Pose block;
    bool active = action_server_.isActive();

    for (unsigned int i=0; i < msg->poses.size(); i++)
    {
      block = msg->poses[i];
      addBlock(block, i, active, msg->header.frame_id);
    }
    ROS_INFO("[block logic] Added %d blocks to Rviz", int(msg->poses.size()));

    interactive_m_server_.applyChanges();

    msg_ = msg;
    initialized_ = true;
  }

  // Move the real block!
  void feedbackCb( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    if (!action_server_.isActive())
    {
      ROS_INFO("[block logic] Got feedback but not active!");
      return;
    }
    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM("[block logic] Staging " << feedback->marker_name);
      old_pose_ = feedback->pose;
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM("[block logic] Now moving " << feedback->marker_name);
      moveBlock(old_pose_, feedback->pose);
      break;
    }

    interactive_m_server_.applyChanges();
  }

  void moveBlock(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    geometry_msgs::Pose start_pose_bumped, end_pose_bumped;
    start_pose_bumped = start_pose;
    start_pose_bumped.position.y -= bump_size;
    //start_pose_bumped.position.z -= block_size/2.0 - bump_size;
    start_pose_bumped.position.z = CUBE_Z_HEIGHT;
    action_result_.pickup_pose = start_pose_bumped;

    end_pose_bumped = end_pose;
    //end_pose_bumped.position.z -= block_size/2.0 - bump_size;
    end_pose_bumped.position.z = CUBE_Z_HEIGHT;
    action_result_.place_pose = end_pose_bumped;

    geometry_msgs::PoseArray msg;
    msg.header.frame_id = arm_link;
    msg.header.stamp = ros::Time::now();
    msg.poses.push_back(start_pose_bumped);
    msg.poses.push_back(end_pose_bumped);

    pick_place_pub_.publish(msg);

    action_server_.setSucceeded(action_result_);

    // DTC server_.clear();
    // DTC server_.applyChanges();
  }

  // Make a box
  Marker makeBox( InteractiveMarker &msg, float r, float g, float b )
  {
    Marker m;

    m.type = Marker::CUBE;
    m.scale.x = msg.scale;
    m.scale.y = msg.scale;
    m.scale.z = msg.scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    return m;
  }

  // Add a new block
  void addBlock( const geometry_msgs::Pose pose, int n, bool active, std::string link)
  {
    InteractiveMarker marker;
    marker.header.frame_id = link;
    marker.pose = pose;
    marker.scale = block_size;

    std::stringstream conv;
    conv << n;
    conv.str();

    marker.name = "block" + conv.str();

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

    if (active)
      marker.controls.push_back( control );

    control.markers.push_back( makeBox(marker, .5, .5, .5) );
    control.always_visible = true;
    marker.controls.push_back( control );


    interactive_m_server_.insert( marker );
    interactive_m_server_.setCallback( marker.name, boost::bind( &BlockLogicServer::feedbackCb, this, _1 ));
  }

};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_logic_action_server");

  clam_msgs::BlockLogicServer block_logic("block_logic");

  ros::spin();
}

