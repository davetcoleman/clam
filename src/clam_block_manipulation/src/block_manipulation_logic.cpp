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
   Desc:   This script combines the three components of block manipulation: block perception, block logic,
   and pick & place into one coherent program using actionlib action servers.
*/

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <clam_msgs/BlockPerceptionAction.h>
#include <clam_msgs/PickPlaceAction.h>
#include <clam_msgs/ClamArmAction.h>

#include <string>
#include <sstream>


const std::string pick_place_topic = "/pick_place";

namespace clam_block_manipulation
{

class BlockManipulationLogic
{
private:
  ros::NodeHandle nh_;

  // Actions
  actionlib::SimpleActionClient<clam_msgs::BlockPerceptionAction> block_perception_action_;
  actionlib::SimpleActionClient<clam_msgs::PickPlaceAction> pick_place_action_;
  actionlib::SimpleActionClient<clam_msgs::ClamArmAction> clam_arm_client_;

  clam_msgs::BlockPerceptionGoal block_perception_goal_;
  clam_msgs::PickPlaceGoal pick_place_goal_;
  clam_msgs::ClamArmGoal clam_arm_goal_;

  // Parameters
  std::string base_link_;
  double z_up_;
  double z_down_;
  double block_size_;
  bool once_;

public:

  BlockManipulationLogic() :
    block_perception_action_("block_perception", true),
    pick_place_action_("pick_place", true),
    clam_arm_client_("clam_arm", true)
  {
    // Load parameters -------------------------------------------------------------------

    nh_.param<std::string>("/block_manipulation_action_demo/base_link", base_link_, "/base_link");
    nh_.param<double>("/block_manipulation_action_demo/z_up", z_up_, 0.12);
    nh_.param<double>("/block_manipulation_action_demo/table_height", z_down_, 0.01);
    nh_.param<double>("/block_manipulation_action_demo/block_size", block_size_, 0.03);
    nh_.param<bool>("once", once_, true);

    //ROS_INFO_STREAM_NAMED("logic","Block size %f", block_size_);
    //ROS_INFO_STREAM_NAMED("logic","Table height %f", z_down_);

    // -----------------------------------------------------------------------------------------------
    // Initialize goals

    // Block Perception
    block_perception_goal_.frame = base_link_;
    block_perception_goal_.table_height = z_down_;
    block_perception_goal_.block_size = block_size_;

    // Pick and Place
    pick_place_goal_.frame = base_link_;
    pick_place_goal_.z_up = z_up_;
    pick_place_goal_.topic = pick_place_topic;

    // -----------------------------------------------------------------------------------------------
    // Connect to the needed action servers
    connectToServers();

    // -----------------------------------------------------------------------------------------------
    // Begin pipeline
    resetArm();
  }

  // Shutdown the arm correctly... doesn't work
  ~BlockManipulationLogic()
  {
    // -----------------------------------------------------------------------------------------------
    // Go to sleep
    ROS_INFO_STREAM_NAMED("logic","Sending arm to shutdown position");
    clam_arm_goal_.command = clam_msgs::ClamArmGoal::SHUTDOWN;
    clam_arm_client_.sendGoal(clam_arm_goal_);
    clam_arm_client_.waitForResult(ros::Duration(20.0));
    if( !clam_arm_client_.getState().isDone() || !clam_arm_client_.getResult()->success )
    {
      ROS_ERROR_STREAM_NAMED("logic","Timeout: Unable to move to shutdown position");
    }
  }

  void connectToServers()
  {
    // Wait for servers -------------------------------------------------------------------
    ROS_INFO_STREAM_NAMED("logic","Finished initializing, waiting for servers:");

    ROS_INFO_STREAM_NAMED("logic","- Waiting for block perception server.");
    block_perception_action_.waitForServer();

    ROS_INFO_STREAM_NAMED("logic","- Waiting for pick and place server.");
    pick_place_action_.waitForServer();

    ROS_INFO_STREAM_NAMED("logic","- Waiting for clam arm server.");
    clam_arm_client_.waitForServer();

    ROS_INFO_STREAM_NAMED("logic","Startup complete.");
  }

  // Send arm to home position
  void resetArm()
  {
    ROS_INFO_STREAM_NAMED("logic","1. Sending arm to home position (reseting)");

    clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;

    clam_arm_client_.sendGoal(clam_arm_goal_,
                              boost::bind( &BlockManipulationLogic::detectBlocks, this));
  }

  void detectBlocks()
  {
    ROS_INFO_STREAM_NAMED("logic","2. Requesting current block positions");

    block_perception_action_.sendGoal(block_perception_goal_,
                                      boost::bind( &BlockManipulationLogic::receivedBlockPoses, this, _1, _2));
  }

  void receivedBlockPoses(const actionlib::SimpleClientGoalState& state,
                          const clam_msgs::BlockPerceptionResultConstPtr& result)
  {
    // Check that the action succeeded with no erors
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM_NAMED("logic","3. Received detected blocks, deciding what to do next.");
    }
    else
    {
      // Try to recover from error
      if( state == actionlib::SimpleClientGoalState::LOST )
      {
        ROS_ERROR_STREAM_NAMED("logic","3. Lost action server. Attempting to reconnect...");
        // Try to reconnect
        connectToServers();
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("logic","3. Failed to get detected blocks: " << state.toString());
        ros::shutdown();
      }
    }


    // --------------------------------------------------------------------------------------------
    // Start pose - choose one that is preferrably not in the goal region
    geometry_msgs::Pose start_pose;
    bool found_pose = false;

    // Check if there is only 1 block detected
    if( !result->blocks.poses.size() )
    {
      // no blocks, what to do?
      ROS_WARN_STREAM_NAMED("logic","No blocks found");
    }
    else if( result->blocks.poses.size() == 1 )
    {
      start_pose = result->blocks.poses[0];
      found_pose = true;
      ROS_INFO_NAMED("logic","Only 1 block, using it");
    }
    else
    {
      // Search for block that meets our criteria
      for(int i = 0; i < result->blocks.poses.size(); ++i)
      {
        if( result->blocks.poses[i].position.y > -0.12 && result->blocks.poses[i].position.y < 0.2 ) // start of goal region
        {
          start_pose = result->blocks.poses[i];
          found_pose = true;
          ROS_INFO_STREAM_NAMED("logic","Chose this block:\n" << start_pose);
          break;
        }
      }
      if( !found_pose )
      {
        ROS_INFO_NAMED("logic","Did not find a good block, default to first");
        start_pose = result->blocks.poses[0];
        found_pose = true;
      }
    }

    // --------------------------------------------------------------------------------------------
    // End pose is just chosen place on board TODO
    geometry_msgs::Pose end_pose;
    end_pose.orientation = result->blocks.poses[0].orientation; // keep the same orientation
    end_pose.position.x = 0.225;
    end_pose.position.y = 0.18;
    end_pose.position.z = result->blocks.poses[0].position.z;

    // --------------------------------------------------------------------------------------------
    // Move that block
    if( found_pose )
    {
      moveBlock(start_pose, end_pose);
    }
    else
    {
      ROS_WARN_STREAM_NAMED("logic","No pose found... ?");
    }

  }

  void moveBlock(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    double bump_size = 0;

    // Copy the start and end poses to 'bumped' versions
    geometry_msgs::Pose start_pose_bumped = start_pose;
    start_pose_bumped.position.y -= bump_size;
    start_pose_bumped.position.z -= block_size_/2.0 - bump_size;

    geometry_msgs::Pose end_pose_bumped = end_pose;
    end_pose_bumped.position.z -= block_size_/2.0 - bump_size;

    // Copy the bumped poses to the pick place goal
    pick_place_goal_.pickup_pose = start_pose_bumped;
    pick_place_goal_.place_pose = end_pose_bumped;

    // Send the goal
    ROS_INFO_NAMED("logic","4. Sending pick place goal to action server");
    pick_place_action_.sendGoal(pick_place_goal_,
                                boost::bind( &BlockManipulationLogic::pickPlaceComplete, this, _1, _2), // Done callback
                                boost::bind( &BlockManipulationLogic::pickPlaceActive, this),   // Active callback
                                boost::bind( &BlockManipulationLogic::pickPlaceFeedback, this, _1)  // Feedback callback
                                );
  }

  void pickPlaceActive()
  {
    ROS_INFO_NAMED("logic","4b. Pick place goal just went active");
  }

  void pickPlaceFeedback(const clam_msgs::PickPlaceFeedbackConstPtr& feedback)
  {
    ROS_INFO_STREAM_NAMED("logic","4c. Pick place feedback: " << feedback->status); // TODO ->status?
  }

  void pickPlaceComplete(const actionlib::SimpleClientGoalState& state,
              const clam_msgs::PickPlaceResultConstPtr& result)
  {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      if(result->success == true)
      {
        ROS_INFO_STREAM_NAMED("logic","5. Pick and place commands successfull");
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("logic","5. Pick and place did not succeed");
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("logic","5. Pick and place action failed: " <<  state.toString().c_str());
    }

    if (once_)
    {
      clam_arm_goal_.command = clam_msgs::ClamArmGoal::RESET;
      clam_arm_client_.sendGoal(clam_arm_goal_);
      ros::Duration(5.0).sleep();

      ROS_INFO_STREAM_NAMED("logic","Shutting down");
      ros::shutdown();
    }
    else
    {
      ROS_INFO_STREAM_NAMED("logic"," ");
      ROS_INFO_STREAM_NAMED("logic","Restarting Logic --------------------------------------------- ");

      resetArm();
    }
  }
};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");

  clam_block_manipulation::BlockManipulationLogic demo;

  // everything is done in cloud callback, just spin
  ros::spin();
}

