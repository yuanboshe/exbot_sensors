/*!
 * Author: Yuanbo She yuanboshe@126.com
 * Group: ExBot http://blog.exbot.net
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, ExBot & RoboPeak.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/LaserScan.h>
#include "rplidar.h"

using namespace rp::standalone::rplidar;

// global vars
ros::Publisher scanPub;
RPlidarDriver* drv;

void shutdown(int sig)
{
  RPlidarDriver::DisposeDriver(drv);
  ROS_INFO("RPlidar ended safely!");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // consts
  const float PI = 3.1415;
  const float RPLIDAR_SAMPLE_DURATION = 0.0004903; //us

  // ros init
  ros::init(argc, argv, "exbot_rplidar");
  ros::NodeHandle node;
  scanPub = node.advertise<sensor_msgs::LaserScan>("scan", 50);
  signal(SIGINT, shutdown);

  // params
  std::string com_path; // com path, default "/dev/ttyUSB0"
  node.param<std::string>("com_path", com_path, "/dev/ttyUSB0");
  int com_baudrate; // com baudrate, default 115200
  node.param("com_baudrate", com_baudrate, 115200);
  std::string frame_id; // frame_id for display in ros
  node.param<std::string>("frame_id", frame_id, "laser_frame");
  double range_min; // the data value < range_min will be dropped
  node.param("range_min", range_min, 0.0);
  double range_max; // the data value > range_max will be dropped
  node.param("range_max", range_max, 13.0);

  // init rplidar
  u_result op_result;
  RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
  if (!drv)
  {
    ROS_ERROR("Create RPlidar instance failed, please check the connection!");
    exit(-2);
  }

  // make connection...
  if (IS_FAIL(drv->connect(com_path.c_str(), com_baudrate)))
  {
    ROS_ERROR("Bind to the specified serial port %s failed, please check the connection!", com_path.c_str());
    shutdown(0);
  }

  // check rplidar health...
  u_result health_result;
  rplidar_response_device_health_t health_info;
  health_result = drv->getHealth(health_info);
  if (IS_OK(op_result))
  {
    ROS_INFO("RPlidar health status: [%d]", health_info.status);
    if (health_info.status == RPLIDAR_STATUS_ERROR)
    {
      ROS_ERROR("RPlidar internal error detected. Please reboot the device to retry.");
      shutdown(0);
    }
  }
  else
  {
    ROS_ERROR("RPlidar cannot retrieve the lidar health code: [%x], Please reboot the device to retry.", op_result);
    shutdown(0);
  }

  // start scan...
  drv->startScan();
  ROS_INFO("RPlidar start...");
  while (ros::ok())
  {
    rplidar_response_measurement_node_t nodes[360 * 2];
    size_t count = (int)(sizeof(nodes) / sizeof(nodes[0]));
    op_result = drv->grabScanData(nodes, count);

    if (IS_OK(op_result))
    {
      // generate LaserScan message
      sensor_msgs::LaserScan scan_msg;
      scan_msg.header.stamp = ros::Time().now();
      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min = 0;
      scan_msg.angle_max = 2 * PI;
      scan_msg.angle_increment = -2 * PI / count;
      scan_msg.time_increment = RPLIDAR_SAMPLE_DURATION;
      scan_msg.range_min = range_min;
      scan_msg.range_max = range_max;
      scan_msg.ranges.resize(count);
      scan_msg.intensities.resize(count);
      for (int pos = 0; pos < (int)count; ++pos)
      {
        scan_msg.ranges[pos] = nodes[pos].distance_q2 / 4000.0f;
        scan_msg.intensities[pos] = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
      }

      scanPub.publish(scan_msg);
      ROS_INFO("Pub scan data count: %d", count);
    }
  }

  ROS_INFO("You can not reach hear...");
  return 0;
}
