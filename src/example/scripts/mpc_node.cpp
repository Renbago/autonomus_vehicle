/*
* MIT License
* 
* Copyright (c) 2024 Mehmet Baha Dursun
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*
 * Author: Mehmet Baha Dursun
 */

#include "mekatronom/MpcNode.hpp"
#include <thread>
int main(int argc, char** argv) {
  ros::init(argc, argv, "MpcNode", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");
  
  try {
    ROS_INFO("[MPC Node]: Initializing node");
    unsigned int cpu_cores = std::thread::hardware_concurrency();
    ROS_INFO_STREAM("[MPC Node]: Number of CPU cores: " << cpu_cores);
    MpcNode mpcNode(nh, nh_local);
    ros::AsyncSpinner spinner(0); // ROS will use a thread for each CPU core
    spinner.start();
    ros::waitForShutdown();
  }
  catch (const char* s) {
    ROS_FATAL_STREAM("[MPC Node]: " << s);
  }
  catch (...) {
    ROS_FATAL_STREAM("[MPC Node]: Unexpected error");
  }

  return 0;
}