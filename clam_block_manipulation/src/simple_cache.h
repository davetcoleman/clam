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

// C++
#include <math.h>
#define _USE_MATH_DEFINES

namespace clam_block_manipulation
{

// Class
class SimpleCache
{
private:

  std::map<int64_t,int64_t> cache_;

public:

  // Constructor
  SimpleCache()
  {
  }


  bool insertToCache(geometry_msgs::Pose* ik_pose, const std::vector<double>& joint_values)                     
  {
    int64_t key;
    int64_t value;
    convertPose(ik_pose,key);
    convertJointValues(joint_values,value);
      
    // -------------------------------------------------------------------------------------------------------
    // Insert into cache
    cache_[key] = value;
  }

  // Convert joint_values to array
  bool convertJointValues( const std::vector<double> joint_values, int64_t& key )
  {
    ROS_INFO_STREAM_NAMED("cache","Joint Values:\n");
    std::copy(joint_values.begin(), joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));   
   
    int joint_size = joint_values.size();
    double doubles[joint_size];
    std::copy( joint_values.begin(), joint_values.begin()+joint_size, doubles);

    convertArray(doubles2, joint_size, key);
  }

  // Convert pose to array
  bool convertPose( geometry_msgs::Pose* ik_pose, int64_t& key )
  {
    ROS_INFO_STREAM_NAMED("cache","POSE:\n" << (*ik_pose) );

    static const int POSE_SIZE = 7;
    double doubles[] = {ik_pose->position.x, ik_pose->position.y, ik_pose->position.z, ik_pose->orientation.x,
                         ik_pose->orientation.y, ik_pose->orientation.z, ik_pose->orientation.w};
    convertArray(doubles, POSE_SIZE, key);
  }

  bool convertArray(double doubles[], size_t n, int64_t& key)
  {
    int converted;
    int exp = 0;
    int ints[n];
    // fill ints with converted doubles
    for (int j = 0; j < n; ++j)
    {
      if( !roundPosition(doubles[j], converted) )
      {
        // rounding failed because value outside range
        ROS_WARN_STREAM_NAMED("cache","Caching failed on index " << j);
        return true; // TODO: change this
      }
      else
      {
        ints[j] = converted;

        // Now add to key
        if(exp > 0)
          key += (int64_t)(ints[j] * pow(10,exp));
        else
          key += (int64_t)(ints[j]); // the first one isn't multiplied by a exponent of 10

        ROS_DEBUG_STREAM_NAMED("cache","key is " << key << " and 10 is " << pow(10,exp));

        exp = exp + 2; // increment next exponent
      }
    }

    // Debug output
    /*for (int j = 0; j < n; ++j)
      {
      std::cout << doubles[j] << " to " << ints[j] << "\n";
      }
    */
    ROS_INFO_STREAM_NAMED("cache","key = " << key);
  }

  // Converts a double to an int
  bool roundPosition(double x, int& converted)
  {
    // Assumes -1 < x < 1
    //ROS_INFO_STREAM_NAMED("cache","converting " << x );
    // Check that x within range
    if( x >= 1 || x <= -1 )
      return false; // this does not fit in cache

    // Scale x to be > 0
    x = x + 1.0;

    // Use the first decimal of x only
    x = (100.0*x)/2.0;
    //ROS_INFO_STREAM_NAMED("cache","   now " << x );
    // Convert to int
    converted = int(x);
    //ROS_INFO_STREAM_NAMED("cache","   now " << converted );
    return true;
  }


}; // end of class

} // namespace
