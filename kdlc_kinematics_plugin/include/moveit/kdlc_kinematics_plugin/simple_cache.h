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
#include <boost/lexical_cast.hpp>

// C++
#include <iostream>
#include <fstream>
#include <math.h>
#include <climits>
#define _USE_MATH_DEFINES

namespace simple_cache
{

enum results_t {SUCCESS, FAILURE, DUPLICATE, NOTFOUND, NOSOLUTION};

// Class
class SimpleCache
{
private:

  std::map<int64_t,int64_t> cache_;

  // Size of ik solutions
  int num_joints_;

  // Output to console
  bool verbose_;

  // Whether to keep the file open and write to disk as insertions are made
  bool live_write_;
  //FILE *append_file_;
  std::ofstream append_file_;

  // Ranges of inputs
  double joint_hi_;
  double joint_low_;
  double pose_hi_;
  double pose_low_;

  // Stats
  unsigned int num_matches_;
  unsigned int num_inserts_;
  unsigned int num_duplicate_inserts_;
  unsigned int num_nosolutions_inserts_;
  unsigned int num_nosolutions_gets_;
  unsigned int num_errors_;

public:

  /**
   * @brief Constructor
   * @param num_joints size of ik solutions
   * @param verbose whether to print debug output to screen
   * @param joint_hi  -- Limits of max and min values
   * @param joint_low
   * @param pose_hi
   * @param pose_low
   */
  SimpleCache(int num_joints, bool verbose,
              double joint_hi, double joint_low, double pose_hi,  double pose_low) :
    num_joints_(num_joints),
    verbose_(verbose),
    joint_hi_(joint_hi),
    joint_low_(joint_low),
    pose_hi_(pose_hi),
    pose_low_(pose_low),
    live_write_(false),
    num_matches_(0),
    num_inserts_(0),
    num_duplicate_inserts_(0),
    num_nosolutions_inserts_(0),
    num_nosolutions_gets_(0),
    num_errors_(0)
  {
  }

  /**
   * @brief Deconstructor - close append file if still open
   */
  ~SimpleCache()
  {
    // Only close file if we are in append mode
    if( live_write_ )
    {
      ROS_INFO_STREAM_NAMED("cache","Closing append file...");
      append_file_.close();
    }
  }

  /**
   * @brief Write a cache to file
   * @param path location of file
   * @return true if write was successful
   */
  bool writeFile(std::string path)
  {
    ROS_INFO_STREAM_NAMED("cache","Writing to file");
    ros::Duration(3.0).sleep();

    int num_insertions = 0;
    if (cache_.empty())
    {
      if(verbose_)
        ROS_WARN_STREAM_NAMED("cache","Did not write to file because cache is empty");
      return false;
    }

    //FILE *file = fopen(path.c_str(), "w");
    std::ofstream file;
    file.open(path.c_str(), std::ios_base::out);

    if (!file.is_open() || !file.good())
    {
      ROS_ERROR_STREAM_NAMED("cache","Error writing KDLC cache to disk.");
      return false;
    }

    // Write to file
    for(std::map<int64_t, int64_t>::iterator it = cache_.begin(); it != cache_.end(); it++)
    {
      //fprintf(file, "%ld=%ld\n", it->first, it->second);
      file << it->first << " " << it->second << std::endl;
      num_insertions++;
    }

    //fclose(file);
    file.close();

    ROS_INFO_STREAM_NAMED("cache","Wrote " << num_insertions << " key value pairs to file");

    // Sucess
    return true;
  }

  /**
   * @brief open file for being appended to
   * @param path location of file
   */
  void startAppend(std::string path)
  {
    append_file_.open(path.c_str(), std::ios_base::app);

    if (!append_file_.is_open() || !append_file_.good())
    {
      ROS_ERROR_STREAM_NAMED("cache","Error opening file for appending: " << path);
      live_write_ = false;
      return;
    }

    live_write_ = true;
  }

  /**
   * @brief Load a cache from file
   * @param path location of file
   * @return true if read was successful
   */
  bool readFile(std::string path)
  {
    if (access(path.c_str(), R_OK) < 0)
    {
      ROS_WARN_STREAM_NAMED("cache","File not found: " << path);
      return false;
    }

    std::ifstream file(path.c_str());
    //FILE *file = fopen(path.c_str(), "r");
    if (!file.is_open())
    {
      ROS_ERROR_STREAM_NAMED("cache","Error opening file " << path);
      return false;
    }

    cache_.clear();

    int num_insertions = 0;
    int64_t key;
    int64_t value;

    while( file.good() )
    {
      file >> key >> value;

      //ROS_INFO_STREAM_NAMED("cache","Read in " << key << "," << value);
      // Add to cache
      cache_[key] = value;

      ++num_insertions;
    }

    ROS_INFO_STREAM_NAMED("cache","Read " << num_insertions << " key value pairs into cache");

    // Sucess
    return true;
  }

  /**
   * @brief Add an IK solution to cache
   * @param ik_pose the input key
   * @param joint_values the input value
   * @return results_t an enum of different status
   */
  results_t insert(const geometry_msgs::Pose& ik_pose, const std::vector<double>& joint_values, bool no_solution = false)
  {
    // Error check
    if( joint_values.size() != num_joints_ && !no_solution)
    {
      ROS_ERROR_STREAM_NAMED("cache","Mismatched solution size for joint values. Recieved " << joint_values.size()
                             << " expected " << num_joints_);
      ++num_errors_;
      return FAILURE;
    }

    int64_t key = 0;
    int64_t value = 0;
    if(!poseToKey(ik_pose,key))
    {
      ++num_errors_;
      return FAILURE;
    }

    // if no solution, set value to MAX VALUE
    if(no_solution)
    {
      value = LLONG_MAX;
      ++num_nosolutions_inserts_;
    }
    else if(!jointsToKey(joint_values,value))
    {
      ++num_errors_;
      return FAILURE;
    }

    // Check map for key
    if(cache_.count( key ) == 1)
    {
      if(verbose_)
        ROS_ERROR_STREAM_NAMED("cache","Key already in map! Prev: " << cache_[key] << " New: " << value);

      // Check if previous one had a solution, out of curiosity
      if( no_solution && cache_[key] != LLONG_MAX)
      {
        ROS_ERROR_STREAM_NAMED("cache","Current solution is 'NOSOLUTION' but previous one had valid solution. Curious.");
      }

      ++num_duplicate_inserts_;
      return DUPLICATE;
    }

    // Insert into cache
    cache_[key] = value;
    ++num_inserts_;

    // Save to file if necessary
    if( live_write_ )
      fileAppend(key,value);

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
    {
      ++num_errors_;
      return FAILURE;
    }

    // Check map for key
    if(cache_.count( key ) == 0)
    {
      if(verbose_)
        ROS_WARN_STREAM_NAMED("cache","get: No value found for key " << key);

      return NOTFOUND;
    }

    // Get value in map
    int64_t value = cache_[key];
    ++num_matches_;

    if(verbose_)
      ROS_INFO_STREAM_NAMED("cache","get: found value " << value << " max is " << LLONG_MAX);

    // Check if key is at maximum value, if it is that means no solution was found
    if(value == LLONG_MAX)
    {
      ++num_nosolutions_gets_;
      return NOSOLUTION;
    }

    // Convert value to vector
    if(!keyToJoints(value, joint_values))
    {
      ++num_errors_;
      return FAILURE;
    }

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
   * @brief output cache to console for debugging
   */
  void printMap()
  {
    ROS_INFO_STREAM_NAMED("cache","Printing key value pairs in map: ---------------------------------------");
    std::pair<int64_t,int64_t> keyvalue; // what a map<int, int> is made of
    BOOST_FOREACH(keyvalue, cache_) {
      std::cout << keyvalue.first << " " << keyvalue.second << "\n";
    }
  }

  /**
   * @brief output the number limits to consolte for debugging
   */
  void printLimits()
  {
    ROS_INFO_STREAM_NAMED("cache","Limits of numbers:");
    std::cout << "joint_hi: " << joint_hi_ << std::endl;
    std::cout << "joint_low: " << joint_low_ << std::endl;
    std::cout << "pose_hi: " << pose_hi_ << std::endl;
    std::cout << "pose_low: " << pose_low_ << std::endl;
  }

  /**
   * @brief print out stats
   */
  void printStats()
  {
    ROS_INFO_STREAM_NAMED("cache","Stats");
    std::cout << "num matches: \t\t\t" << num_matches_ << std::endl;
    std::cout << "num inserts: \t\t\t" << num_inserts_ << std::endl;    
    std::cout << "num duplicate inserts: \t\t" << num_duplicate_inserts_ << std::endl;
    std::cout << "num nosolution inserts: \t" << num_nosolutions_inserts_ << std::endl;
    std::cout << "num nosolution gets: \t\t" << num_nosolutions_gets_ << std::endl;
    std::cout << "num errors: \t\t\t" << num_errors_ << std::endl;
    std::cout << "size of cache: \t\t\t" << cache_.size() << std::endl;
  }

private:

  /**
   * @brief save an insertion to disk
   * @param key input
   * @param value input
   */
  void fileAppend(int64_t key, int64_t value)
  {
    // Make sure file is open first!
    append_file_ << key << " " << value << std::endl;
  }

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
      ROS_INFO_STREAM_NAMED("cache","Converting to a 64bit int using joint values:");
      std::copy(joint_values.begin(), joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));
    }

    int joint_size = joint_values.size();
    double doubles[joint_size];
    std::copy( joint_values.begin(), joint_values.begin()+joint_size, doubles);

    if( !arrayToKey(doubles, joint_size, key, joint_low_, joint_hi_) )
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
    if( !keyToArray(key, num_joints_, doubles, joint_low_, joint_hi_) )
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
    if(verbose_)
      ROS_DEBUG_STREAM_NAMED("cache","Converting to 64 bit from pose:\n" << ik_pose );

    static const int POSE_SIZE = 7;
    double doubles[] = {ik_pose.position.x, ik_pose.position.y, ik_pose.position.z, ik_pose.orientation.x,
                        ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w};
    if( !arrayToKey(doubles, POSE_SIZE, key, pose_low_, pose_hi_) )
      return false;

    return true;
  }

  /**
   * @brief Convert array of doubles to key value
   * @param doubles input to be converted
   * @param key output value
   * @return false if either inputs are out of specified range - could not cache
   */
  bool arrayToKey(const double doubles[], const size_t n, int64_t& key, double low, double hi)
  {
    //ROS_INFO_STREAM_NAMED("cache","Converting data array to key -----------------------------");

    int converted;
    int exp = 0;
    // fill ints with converted doubles
    for (int j = 0; j < n; ++j)
    {
      if( !doubleToInt(doubles[j], converted, low, hi) )
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
      ROS_DEBUG_STREAM_NAMED("cache","Converted data array to key " << key);

    return true;
  }

  /**
   * @brief Convert key to doubles
   * @param key input to be converted
   * @param n the number of doubles in the key to be pulled out
   * @param doubles output value
   * @return false if a number is out of range
   */
  bool keyToArray(int64_t key, const int n, double doubles[], double low, double hi)
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
      if( !intToDouble(rounded_segment, converted, low, hi) )
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
  bool doubleToInt(double x, int& result, double low, double high)
  {
    // Assumes -1 < x < 1
    //ROS_INFO_STREAM_NAMED("cache","converting " << x );
    // Check that x within range
    if( x >= high || x <= low )
    {
      if(verbose_)
        ROS_WARN_STREAM_NAMED(     "cache","Num out of range: " << low << " <= " << x << " <= " << high <<
                                   " is not true.");
      else
        ROS_WARN_STREAM_ONCE_NAMED("cache","Num out of range: " << low << " <= " << x << " <= " << high <<
                                   " is not true. This message will only print once.");
      return false; // this does not fit in cache
    }

    // Translate x to be > 0
    x = x + fabs(low);

    // Use the first decimal of x only
    x = (100.0*x)/fabs(high-low);
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
  bool intToDouble(int x, double& result, double low, double high)
  {
    // Convert to double
    result = x;

    // Scale back
    result = (result * fabs(high-low)) / 100.0;

    // translate back to negative if necessary
    result = result - fabs(low);

    if(verbose_)
      ROS_DEBUG_STREAM_NAMED("cache","Converted " << x << " to " << result);

    return true;
  }

}; // end of class

typedef boost::shared_ptr<SimpleCache> SimpleCachePtr;
typedef boost::shared_ptr<const SimpleCache> SimpleCacheConstPtr;

} // namespace
