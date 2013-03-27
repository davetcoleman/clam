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
   Desc:   Runs tests on the grasp cache
*/

#include <moveit/kdlc_kinematics_plugin/simple_cache.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h> // rand
#include <stdio.h> // remove
#include <time.h>

namespace simple_cache_test
{

static const int NUM_JOINTS = 7;
static const std::string CACHE_LOCATION = "/home/dave/.ros/kdlc_test_cache.dat";

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void getRandomPose(geometry_msgs::Pose& pose, double pose_hi, double pose_low)
{
  // Positions:
  pose.position.x = fRand(pose_low,pose_hi);
  pose.position.y = fRand(pose_low,pose_hi);
  pose.position.z = fRand(pose_low,pose_hi);
  // Orientations:
  pose.orientation.x = fRand(pose_low,pose_hi);
  pose.orientation.y = fRand(pose_low,pose_hi);
  pose.orientation.z = fRand(pose_low,pose_hi);
  pose.orientation.w = fRand(pose_low,pose_hi);
}

void getRandomJoints(std::vector<double>& joints, double joint_hi, double joint_low)
{
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    joints.push_back(fRand(joint_low,joint_hi));
  }
}

bool randTrueFalse(int probability)
{
  if ((rand() % 100) < probability)
    return true;
  else
    return false;
}

void runTests(int num_tests)
{
  // Print output to screen
  bool verbose = false;
  bool live_write = true;

  // Statistics
  int num_insertions = 0;
  int num_duplicates = 0;
  int num_insert_errors = 0;
  int num_get_errors = 0;
  // Stats for getting
  double max_error = 0;
  double total_error  = 0;
  double total_measurements = 0;
  double total_missed = 0;

  // ---------------------------------------------------------------------------------------------------------
  // Create random value ranges
  double joint_hi = simple_cache_test::fRand(-5,10);
  double joint_low = simple_cache_test::fRand(-10,joint_hi);
  double pose_hi = simple_cache_test::fRand(-5,10);
  double pose_low = simple_cache_test::fRand(-10,pose_hi);

  // Create instance of simple cache
  simple_cache::SimpleCache cache(simple_cache_test::NUM_JOINTS, verbose, joint_hi, joint_low, pose_hi, pose_low);

  // Debug limits
  if(verbose)
    cache.printLimits();

  // Delete previous cache
  remove(CACHE_LOCATION.c_str());

  // Read in previous cache
  cache.readFile(CACHE_LOCATION);

  // Begin to append new data to file
  if( live_write )
    cache.startAppend(CACHE_LOCATION);

  // ---------------------------------------------------------------------------------------------------------
  // Create sets of fake data
  std::vector< std::pair<geometry_msgs::Pose,std::vector<double> > > test_data;

  for (int i = 0; i < num_tests; ++i)
  {
    // Create new data pair
    std::pair<geometry_msgs::Pose,std::vector<double> > test_pair;
    simple_cache_test::getRandomPose(test_pair.first, pose_hi, pose_low);
    simple_cache_test::getRandomJoints(test_pair.second, joint_hi, joint_low);
    test_data.push_back(test_pair);

    // Insert 10% of test data a second time
    /*
      if( simple_cache_test::randTrueFalse(10) )
      {
      test_data.push_back(test_pair);
      }
    */
  }

  // ---------------------------------------------------------------------------------------------------------
  // Make some of the data out of bounds
  /*
    for (int i = 0; i < test_data.size(); ++i)
    {
    // Corrupt 1% of the data
    if( simple_cache_test::randTrueFalse(1) )
    {
    // Corrupt pose
    test_data[i].first.position.x = simple_cache_test::fRand(-10,10);
    test_data[i].first.position.y = simple_cache_test::fRand(-10,10);

    // Corrupt joint values
    test_data[i].second[0] = fRand(-10,10);
    test_data[i].second[NUM_JOINTS-1] = fRand(-10,10);
    }
    }
  */

  // ---------------------------------------------------------------------------------------------------------
  // Add all test data into cache
  for (int i = 0; i < test_data.size(); ++i)
  {
    // Debug
    if(verbose)
    {
      ROS_INFO_STREAM_NAMED("","Adding ik solution: -------------------------------------------------------");
      std::copy(test_data[i].second.begin(),test_data[i].second.end(), std::ostream_iterator<double>(std::cout, "\n"));
      ROS_INFO_STREAM_NAMED("","For pose:\n" << test_data[i].first);
    }

    // Insert 90% of test data
    if( true ) //simple_cache_test::randTrueFalse(90) )
    {
      // Insert
      simple_cache::results_t result = cache.insert(test_data[i].first, test_data[i].second);
      if( result == simple_cache::FAILURE)
      {
        ++num_insert_errors;
      }
      else if( result == simple_cache::DUPLICATE )
      {
        ++num_duplicates;
      }
      else if( result != simple_cache::SUCCESS )
      {
        ROS_BREAK(); // this should not happen
      }
      ++num_insertions;

    }
  }

  // ---------------------------------------------------------------------------------------------------------
  // View map
  //cache.printMap();

  // ---------------------------------------------------------------------------------------------------------
  // Retrieve test data and check if still the same

  if(verbose)
    ROS_INFO_STREAM_NAMED("","Retrieving data for comparision: -----------------------------------------------");

  for (int i = 0; i < test_data.size(); ++i)
  {
    //ROS_WARN_STREAM_NAMED("","retrieve data # " << i << " using pose \n" << test_data[i].first);
    std::vector<double> joint_value_seed;

    simple_cache::results_t result = cache.get(test_data[i].first, joint_value_seed);
    if( result == simple_cache::FAILURE)
    {
      ++num_get_errors;
      continue; // we can't convert this posea
    }
    else if( result == simple_cache::NOTFOUND )
    {
      ++total_missed;
      continue; // we can't convert this posea
    }
    else if( result != simple_cache::SUCCESS )
    {
      ROS_BREAK(); // this should not happen
    }

    if(verbose)
    {
      ROS_WARN_STREAM_NAMED("","joint value seed size is = " << joint_value_seed.size());
      ROS_DEBUG_STREAM_NAMED("","Showing differences between cache and actual:");
    }

    double error;
    // Compare similarity of the seed value and the correct value
    for (int j = 0; j < joint_value_seed.size(); ++j)
    {
      if(verbose)
        std::cout << "orig: " << test_data[i].second[j] << " new: " << joint_value_seed[j] << std::endl;

      error = fabs(test_data[i].second[j] - joint_value_seed[j]);
      total_error += error;
      ++total_measurements;

      if( error > max_error )
        max_error = error;
    }

  }

  // Write cache to disk if we haven't already been doing it live
  if( !live_write )
    cache.writeFile(CACHE_LOCATION);


  ROS_INFO_STREAM_NAMED("","Tests Complete ---------------------------------------------------------------");
  ROS_INFO_STREAM_NAMED("","Num of tests: " << num_tests);
  ROS_INFO_STREAM_NAMED("","Size of cache: " << cache.getSize());
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("","Num of duplicates: " << num_duplicates );
  ROS_INFO_STREAM_NAMED("","Percent duplicates: " << double(num_duplicates) / test_data.size() * 100.0 << " %");
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("","Num of get errors: " << num_get_errors );
  ROS_INFO_STREAM_NAMED("","Percent get errors: " << double(num_get_errors) / test_data.size() * 100.0 << " %");
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("","Num of insert errors: " << num_insert_errors );
  ROS_INFO_STREAM_NAMED("","Percent insert errors: " << double(num_insert_errors) / test_data.size() * 100.0 << " %");
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("","Num missing values: " << total_missed);
  ROS_INFO_STREAM_NAMED("","Percent missing values: " << total_missed / test_data.size() * 100.0 << " %");
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("","Joint range " << joint_low << " to " << joint_hi);
  ROS_INFO_STREAM_NAMED("","Max joint error: " << max_error);
  ROS_INFO_STREAM_NAMED("","Avg joint error: " << total_error / total_measurements);


}

} // end namespace

int main(int argc, char *argv[])
{
  int num_tests = 1000000;
  //int num_tests = 10;

  // initialize ros time without using node handle
  ros::Time::init();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();
  {
    // Run Tests
    simple_cache_test::runTests(num_tests);
  }
  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  return 0;
}

