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

#include "simple_cache.h"
#include <geometry_msgs/Pose.h>
#include <stdlib.h> // rand
#include <time.h>

namespace simple_cache_test
{

static const int NUM_JOINTS = 7;

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void getRandomPose(geometry_msgs::Pose& pose)
{
  // Positions: 1 < x < -1
  pose.position.x = fRand(-1,1);
  pose.position.y = fRand(-1,1);
  pose.position.z = fRand(-1,1);
  // Orientations: 1 < x < -1 TODO: is this correct?
  pose.orientation.x = fRand(-1,1);
  pose.orientation.y = fRand(-1,1);
  pose.orientation.z = fRand(-1,1);
  pose.orientation.w = fRand(-1,1);

  /*for (int i = 0; i < 1000000000; ++i)
    {
    //ROS_WARN_STREAM_NAMED("","Check that num is between 1 and -1: " << num);
    if( num >= 1 || num <= -1 )
    {
    ROS_ERROR_STREAM_NAMED("","num out of bound " << num);
    break;
    }
    } */
  //std::cout << pose << std::endl;
}

void getRandomJoints(std::vector<double>& joints)
{
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    joints.push_back(fRand(-1,1));
  }

  // debug
  //ROS_WARN_STREAM_NAMED("","FAKE JOINTS:");
  //std::copy(joints.begin(),joints.end(), std::ostream_iterator<double>(std::cout, "\n"));
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

  // Create instance of simple cache
  simple_cache::SimpleCache cache(simple_cache_test::NUM_JOINTS, verbose);

  // ---------------------------------------------------------------------------------------------------------
  // Create sets of fake data
  std::vector< std::pair<geometry_msgs::Pose,std::vector<double> > > test_data;

  for (int i = 0; i < num_tests; ++i)
  {
    // Create new data pair
    std::pair<geometry_msgs::Pose,std::vector<double> > test_pair;
    simple_cache_test::getRandomPose(test_pair.first);
    simple_cache_test::getRandomJoints(test_pair.second);
    test_data.push_back(test_pair);

    // Insert 10% of test data a second time
    if( simple_cache_test::randTrueFalse(10) )
    {
      test_data.push_back(test_pair);
    }
  }

  // ---------------------------------------------------------------------------------------------------------
  // Make some of the data out of bounds
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
    if( simple_cache_test::randTrueFalse(90) )
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

      if( error > 0.02 )
      {
        ROS_ERROR_STREAM_NAMED("","Found error " << error << " above expected amount 0.02");
        break;
      }
    }

  }


  ROS_INFO_STREAM_NAMED("","Tests Complete ---------------------------------------------------------------");
  ROS_INFO_STREAM_NAMED("","Num of tests: " << num_tests);
  ROS_INFO_STREAM_NAMED("","Num of key-value pairs: " << cache.getSize());
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
  ROS_INFO_STREAM_NAMED("","Max joint error: " << max_error);
  ROS_INFO_STREAM_NAMED("","Avg joint error: " << total_error / total_measurements);


}

} // end namespace

int main(int argc, char *argv[])
{
  // initialize ros time without using node handle
  ros::Time::init();

  // Seed random
  srand(ros::Time::now().toSec());

  // Run tests
  simple_cache_test::runTests(1000000);

  return 0;
}

