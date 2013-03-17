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
   Desc:   Simple 7 number cacher using 64bit ints
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// Boost
#include <boost/foreach.hpp>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace simple_cache
{

enum results_t {SUCCESS, FAILURE, DUPLICATE, NOTFOUND};

// Class
class SimpleCache
{
private:

  std::map<int64_t,int64_t> cache_;

  // Size of ik solutions
  int num_joints_;

  // Output to console
  bool verbose_;

public:

  /**
   * @brief Constructor
   * @param num_joints size of ik solutions
   * @param verbose whether to print debug output to screen
   */
  SimpleCache(int num_joints, bool verbose) :
    num_joints_(num_joints),
    verbose_(verbose)
  {
  }

  /**
   * @brief Add an IK solution to cache
   * @param ik_pose the input key
   * @param joint_values the input value
   * @return results_t an enum of different status
   */
  results_t insert(const geometry_msgs::Pose& ik_pose, const std::vector<double>& joint_values)
  {
    // Error check
    if( joint_values.size() != num_joints_ )
    {
      ROS_ERROR_STREAM_NAMED("cache","Mismatched solution size for joint values. Recieved " << joint_values.size()
                             << " expected " << num_joints_);
      return FAILURE;
    }

    int64_t key = 0;
    int64_t value = 0;
    if(!poseToKey(ik_pose,key))
    {
      return FAILURE;
    }
    if(!jointsToKey(joint_values,value))
      return FAILURE;

    // Check map for key
    if(cache_.count( key ) == 1)
    {
      if(verbose_)
        ROS_ERROR_STREAM_NAMED("cache","Key already in map! Prev: " << cache_[key] << " New: " << value);
      return DUPLICATE;
    }

    // Insert into cache
    cache_[key] = value;

    return SUCCESS;
  }

  /**
   * @brief Get an IK solution from cache
   * @param ik_pose the input key
   * @param joint_values the returned ik seed
   * @return results_t an enum of different status
   */
  results_t get(const geometry_msgs::Pose& ik_pose, std::vector<double>& joint_values)
  {
    // Convert ik_pose to key
    int64_t key = 0;
    if(!poseToKey(ik_pose,key))
      return FAILURE;

    // Check map for key
    if(cache_.count( key ) == 0)
    {
      if(verbose_)
        ROS_ERROR_STREAM_NAMED("cache","No value found for key " << key);
      return NOTFOUND;
    }

    if(verbose_)
      ROS_INFO_STREAM_NAMED("cache","Getting value with key " << key);

    // Convert result to vector
    int64_t result = cache_[key];
    if(!keyToJoints(result, joint_values))
      return FAILURE;

    return SUCCESS;
  }

  /**
   * @brief get size of cache (map)
   * @return size of cache
   */
  size_t getSize()
  {
    return cache_.size();
  }

  /**
   * @brief output cache to console for debug
   */
  void printMap()
  {
    ROS_INFO_STREAM_NAMED("cache","Printing key value pairs in map: ---------------------------------------");
    std::pair<int64_t,int64_t> keyvalue; // what a map<int, int> is made of
    BOOST_FOREACH(keyvalue, cache_) {
      std::cout << keyvalue.first << " " << keyvalue.second << "\n";
    }

  }

private:

  /**
   * @brief Convert joint_values to key value
   * @param joint_values input to be converted
   * @param key output
   * @return false if either inputs are out of specified range - could not cache
   */
  bool jointsToKey( const std::vector<double>& joint_values, int64_t& key )
  {
    if(verbose_)
    {
      ROS_INFO_STREAM_NAMED("cache","Converting joints to key using values:");
      std::copy(joint_values.begin(), joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));
    }

    int joint_size = joint_values.size();
    double doubles[joint_size];
    std::copy( joint_values.begin(), joint_values.begin()+joint_size, doubles);

    if( !arrayToKey(doubles, joint_size, key) )
      return false;

    return true;
  }

  /**
   * @brief Convert joint_values to array
   * @param key input to be converted
   * @param joint_values result
   * @return false if error occure
   */
  bool keyToJoints( const int64_t& key, std::vector<double>& joint_values )
  {
    double doubles[num_joints_];

    // Convert key to array
    if( !keyToArray(key, num_joints_, doubles) )
    {
      // Failed to convert
      ROS_WARN_STREAM_NAMED("cache","Failed to convert value to array");
      return false;
    }
    
    // Clear previous joint_values
    joint_values.clear();

    // Convert array to joints
    for (int i = 0; i < num_joints_; ++i)
    {
      joint_values.push_back(doubles[i]);
    }

    return true;
  }

  /**
   * @brief Convert ik_pose to key value
   * @param ik_pose input to be converted
   * @param key output value
   * @return false if either inputs are out of specified range - could not cache
   */
  bool poseToKey(const geometry_msgs::Pose& ik_pose, int64_t& key)
  {
    //ROS_INFO_STREAM_NAMED("cache","POSE:\n" << ik_pose );

    static const int POSE_SIZE = 7;
    double doubles[] = {ik_pose.position.x, ik_pose.position.y, ik_pose.position.z, ik_pose.orientation.x,
                        ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w};
    if( !arrayToKey(doubles, POSE_SIZE, key) )
      return false;

    return true;
  }

  /**
   * @brief Convert array of doubles to key value
   * @param doubles input to be converted
   * @param key output value
   * @return false if either inputs are out of specified range - could not cache
   */
  bool arrayToKey(const double doubles[], const size_t n, int64_t& key)
  {
    if(verbose_)
      ROS_INFO_STREAM_NAMED("cache","Converting data array to key -----------------------------");

    int converted;
    int exp = 0;
    // fill ints with converted doubles
    for (int j = 0; j < n; ++j)
    {
      if( !doubleToInt(doubles[j], converted) )
      {
        // rounding failed because value outside range
        if(verbose_)
          ROS_WARN_STREAM_NAMED("cache","Caching failed on index " << j);

        return false;
      }
      else
      {
        // Now add to key
        if(exp > 0)
          key += (int64_t)(converted * pow(10,exp));
        else
          key += (int64_t)(converted); // the first one isn't multiplied by a exponent of 10

        //ROS_DEBUG_STREAM_NAMED("cache","key is " << key << " and 10 is " << pow(10,exp));
        //ROS_DEBUG_STREAM_NAMED("cache",doubles[j] << " converted to " << converted);

        exp = exp + 2; // increment next exponent
      }
    }

    // Debug output
    /*for (int j = 0; j < n; ++j)
      {
      std::cout << doubles[j] << " to " << ints[j] << "\n";
      }
    */

    if(verbose_)
      ROS_INFO_STREAM_NAMED("cache","Final key is = " << key);

    return true;
  }

  /**
   * @brief Convert key to doubles
   * @param key input to be converted
   * @param n the number of doubles in the key to be pulled out
   * @param doubles output value
   * @return false if a number is out of range
   */
  bool keyToArray(int64_t key, const int n, double doubles[])
  {
    if(verbose_)
      ROS_INFO_STREAM_NAMED("cache","Converting value " << key << " back to array ----------------------------");
    int exp = 2 * (n-1);
    //int ints[n];
    double segment;
    int rounded_segment;
    double converted;

    // Pull out n numbers
    for (int i = n-1; i >= 0; --i)
    {
      // Get the next number out of the key
      segment = key / pow(10,exp);
      //ROS_DEBUG_STREAM_NAMED("cache","segment = " << segment << " and 10 is " << pow(10,exp) );

      // Cut off decimals
      rounded_segment = int(segment);

      // Subtract that number from the key
      key = key - rounded_segment * pow(10,exp);
      //ROS_DEBUG_STREAM_NAMED("cache","key is now " << key);

      // Convert that number to an int
      if( !intToDouble(rounded_segment, converted) ) 
      {
        return false;
      }
      else
      {
        // Converted successfully
        doubles[i] = converted;
      }

      //ROS_WARN_STREAM_NAMED("","int #" << i << " resulted in " << doubles[i] << " key is now: " << key);

      // decrease exponent
      exp = exp - 2;
    }

    return true;
  }

  /**
   * @brief Converts a double to an int using 2 decimals of precision and the specefied range
   * @param x input to be converted
   * @param result output value
   * @return false if outside range - unable to cache
   */
  bool doubleToInt(double x, int& result)
  {
    // Assumes -1 < x < 1
    //ROS_INFO_STREAM_NAMED("cache","converting " << x );
    // Check that x within range
    if( x >= 1 || x <= -1 )
    {
      ROS_WARN_STREAM_ONCE_NAMED("cache","Input number " << x << " is out of required range. This message will only print once.");
      return false; // this does not fit in cache
    }

    // Scale x to be > 0
    x = x + 1.0;

    // Use the first decimal of x only
    x = (100.0*x)/2.0;
    //ROS_INFO_STREAM_NAMED("cache","   now " << x );
    // Convert to int
    result = int(x);
    //ROS_INFO_STREAM_NAMED("cache","   now " << result );
    return true;
  }

  /**
   * @brief convert int to double - reverse of doubleToInt function
   * @param x the input int
   * @param result the output double
   * @return false if out of range - but not really implemented
   */
  bool intToDouble(int x, double& result)
  {
    // Convert to double
    result = x;

    // Scale back
    result = (result * 2.0) / 100.0;

    // Move back to negative
    result = result - 1.0;

    if(verbose_)
      ROS_DEBUG_STREAM_NAMED("cache","Converted " << x << " to " << result);

    return true;
  }

}; // end of class

typedef boost::shared_ptr<SimpleCache> SimpleCachePtr;
typedef boost::shared_ptr<const SimpleCache> SimpleCacheConstPtr;

} // namespace
