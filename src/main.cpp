/**
 *
 *  \file
 *  \brief      Main entry point for UM6 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */
#include <string>

#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "um7/comms.h"
#include "um7/registers.h"
#include "um7/Reset.h"
#include <ros/console.h>

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = UM7_DREG_TEMPERATURE;

/**
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in UM7 registers.
 */
template<typename RegT>
void configureVector3(um7::Comms* sensor, const um7::Accessor<RegT>& reg,
                      std::string param, std::string human_name)
{
  if (reg.length != 3)
  {
    throw std::logic_error("configureVector3 may only be used with 3-field registers!");
  }

  if (ros::param::has(param))
  {
    double x, y, z;
    ros::param::get(param + "/x", x);
    ros::param::get(param + "/y", y);
    ros::param::get(param + "/z", z);
    ROS_INFO_STREAM("Configuring " << human_name << " to ("
                    << x << ", " << y << ", " << z << ")");
    reg.set_scaled(0, x);
    reg.set_scaled(1, y);
    reg.set_scaled(2, z);
    if (sensor->sendWaitAck(reg))
    {
      throw std::runtime_error("Unable to configure vector.");
    }
  }
}

/**
 * Function generalizes the process of commanding the UM7 via one of its command
 * registers.
 */
template<typename RegT>
void sendCommand(um7::Comms* sensor, const um7::Accessor<RegT>& reg, std::string human_name)
{
  ROS_INFO_STREAM("Sending command: " << human_name);
  if (!sensor->sendWaitAck(reg))
  {
    throw std::runtime_error("Command to device failed.");
  }
}


/**
 * Send configuration messages to the UM7, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(um7::Comms* sensor)
{
  um7::Registers r;

  // Set the baud rate
  const uint8_t UM7_BAUD_115200 = 0x5;
  uint32_t comm_reg = UM7_BAUD_115200 << UM7_BAUD_START_BIT;
  r.communication.set(0, comm_reg);
  if (!sensor->sendWaitAck(r.communication))
  {
    throw std::runtime_error("Unable to set communication register.");
  }
  else
  {
    ROS_INFO("Communication register set.");
    ROS_INFO_STREAM(comm_reg);
  }
  
  // Set the broadcast rate of the raw data
  /*
  const uint8_t UM7_RAW_MAG_RATE = 0x0;
  const uint8_t UM7_RAW_GYRO_RATE = 0x0;
  const uint8_t UM7_RAW_ACCEL_RATE = 0x0;
  //uint32_t raw_broad_reg = UM7_RAW_MAG_RATE << UM7_RAW_MAG_RATE_START_BIT || UM7_RAW_GYRO_RATE << UM7_RAW_GYRO_RATE_START_BIT || UM7_RAW_ACCEL_RATE << UM7_RAW_ACCEL_RATE_START_BIT;
  uint32_t raw_broad_reg = 0b00000000000000000000000000000000;
  r.raw_broadcast.set(0, raw_broad_reg);
  if (!sensor->sendWaitAck(r.proc_broadcast))
  {
    throw std::runtime_error("Unable to set the 1st broadcast register.");
  }
  else
  {
    ROS_INFO("1st broadcast register set.");
    ROS_INFO_STREAM(raw_broad_reg);
  }
  */
  
  // Set the broadcast rate of the raw data
  const uint8_t UM7_ALL_RAW_BROADCAST_RATE = 0x0;
  const uint8_t UM7_TEMP_BROADCAST_RATE = 0x32;
  uint32_t all_raw_reg = UM7_TEMP_BROADCAST_RATE << UM7_TEMP_RATE_START_BIT;
  r.all_raw_broadcast.set(0, all_raw_reg);
  if (!sensor->sendWaitAck(r.all_raw_broadcast))
  {
    throw std::runtime_error("Unable to set the 2nd broadcast register.");
  }
  else
  {
    ROS_INFO("2nd broadcast register set.");
    ROS_INFO_STREAM(all_raw_reg);
  }
  
  /*
  // Set the broadcast rate of the processed data
  const uint8_t UM7_PROC_ACCEL_RATE = 0x64;
  const uint8_t UM7_PROC_GYRO_RATE = 0x64;
  const uint8_t UM7_PROC_MAG_RATE = 0x64;
  uint32_t broad_reg = UM7_PROC_MAG_RATE << UM7_PROC_MAG_RATE_START_BIT | UM7_PROC_GYRO_RATE << UM7_PROC_GYRO_RATE_START_BIT | UM7_PROC_ACCEL_RATE << UM7_PROC_ACCEL_RATE_START_BIT;
  r.proc_broadcast.set(0, broad_reg);
  if (!sensor->sendWaitAck(r.proc_broadcast))
  {
    throw std::runtime_error("Unable to set the 3rd broadcast register.");
  }
  else
  {
    ROS_INFO("3rd broadcast register set.");
    ROS_INFO_STREAM(broad_reg);
  }
  */
    
  // Set the broadcast rate of the processed data
  const uint8_t UM7_ALL_PROC_BROADCAST_RATE = 0x64;
  uint32_t all_proc_broad_reg = UM7_ALL_PROC_BROADCAST_RATE << UM7_ALL_PROC_RATE_START_BIT;
  r.all_proc_broadcast.set(0, all_proc_broad_reg);
  if (!sensor->sendWaitAck(r.all_proc_broadcast))
  {
    throw std::runtime_error("Unable to set the 4th broadcast register.");
  }
  else
  {
    ROS_INFO("4th broadcast register set.");
    ROS_INFO_STREAM(all_proc_broad_reg);
  }
  
  // Set the broadcast rate of the orientation
  const uint8_t UM7_QUAT_BROADCAST_RATE = 0x64;
  const uint8_t UM7_EULER_BROADCAST_RATE = 0x64;
  uint32_t orient_reg = UM7_QUAT_BROADCAST_RATE << UM7_QUAT_RATE_START_BIT | UM7_EULER_BROADCAST_RATE << UM7_EULER_RATE_START_BIT;
  r.orient_broadcast.set(0, orient_reg);
  if (!sensor->sendWaitAck(r.orient_broadcast))
  {
    throw std::runtime_error("Unable to set the 5th broadcast register.");
  }
  else
  {
    ROS_INFO("5th broadcast register set.");
    ROS_INFO_STREAM(orient_reg);
  }

  // Optional miscellaneous settings
  bool measure_gyro_bias, mag_updates;
  ros::param::param<bool>("measure_gyro_bias", measure_gyro_bias, true);
  ros::param::param<bool>("mag_updates", mag_updates, true);
  uint32_t misc_config_reg = UM7_Q_ENABLED;
  if (mag_updates)
  {
    misc_config_reg |= UM7_MAG_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding magnetometer updates from EKF.");
  }
  if (measure_gyro_bias)
  {
    misc_config_reg |= UM7_ZG_ENABLED;
  }
  else
  {
    ROS_WARN("Not measuring rate gyro bias.");
  }
  r.misc_config.set(0, misc_config_reg);
  if (!sensor->sendWaitAck(r.misc_config))
  {
    throw std::runtime_error("Unable to set misc config register.");
  }
  else
  {
    ROS_INFO("Misc config set.");
    ROS_INFO_STREAM(misc_config_reg);
  }

  // Optionally disable the gyro reset on startup. A user might choose to do this
  // if there's an external process which can ascertain when the vehicle is stationary
  // and periodically call the /reset service.
  bool zero_gyros;
  ros::param::param<bool>("zero_gyros", zero_gyros, true);
  if (zero_gyros) 
  {
    sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  }

  // Configurable vectors.
  //None of these are configurable on the UM7
  //configureVector3(sensor, r.cmd_set_mag_ref, "mag_ref", "magnetic reference vector");
  //configureVector3(sensor, r.accel_ref, "~accel_ref", "accelerometer reference vector"); doesn't exist on UM7
  //configureVector3(sensor, r.mag_bias, "~mag_bias", "magnetic bias vector"); set using CHR Serial Interface
  //configureVector3(sensor, r.accel_bias, "~accel_bias", "accelerometer bias vector"); doesn't exist on UM7
  //configureVector3(sensor, r.gyro_bias, "~gyro_bias", "gyroscope bias vector"); set by zeroing the gyros
}


bool handleResetService(um7::Comms* sensor,
                        const um7::Reset::Request& req, const um7::Reset::Response& resp)
{
  um7::Registers r;
  if (req.zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  if (req.reset_ekf) sendCommand(sensor, r.cmd_reset_ekf, "reset EKF");
  if (req.set_mag_ref) sendCommand(sensor, r.cmd_set_mag_ref, "set magnetometer reference");
  return true;
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(um7::Registers& r, ros::NodeHandle* n, const std_msgs::Header& header)
{
  static ros::Publisher imu_pub = n->advertise<sensor_msgs::Imu>("imu/data", 1, false); 
  static ros::Publisher mag_pub = n->advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1, false);
  static ros::Publisher rpy_pub = n->advertise<geometry_msgs::Vector3Stamped>("imu/rpy", 1, false);

  if (imu_pub.getNumSubscribers() > 0)
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = header;

    // IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
    imu_msg.orientation.x = r.quat.get_scaled(2);
    imu_msg.orientation.y = r.quat.get_scaled(1);
    imu_msg.orientation.z = -r.quat.get_scaled(3);
    imu_msg.orientation.w = r.quat.get_scaled(0);

    // IMU reports a 4x4 wxyz covariance, ROS requires only 3x3 xyz.
    // NED -> ENU conversion req'd?
    imu_msg.orientation_covariance[0] = 1;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 1;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 1;

    // NED -> ENU conversion.
    imu_msg.angular_velocity.x = r.gyro_x.get_scaled(1);
    imu_msg.angular_velocity.y = r.gyro_y.get_scaled(0);
    imu_msg.angular_velocity.z = -r.gyro_z.get_scaled(2);

    // NED -> ENU conversion.
    imu_msg.linear_acceleration.x = r.accel_x.get_scaled(1);
    imu_msg.linear_acceleration.y = r.accel_y.get_scaled(0);
    imu_msg.linear_acceleration.z = -r.accel_z.get_scaled(2);

    imu_pub.publish(imu_msg);
  }

  if (mag_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header = header;
    mag_msg.vector.x = r.mag_x.get_scaled(1);
    mag_msg.vector.y = r.mag_y.get_scaled(0);
    mag_msg.vector.z = -r.mag_z.get_scaled(2);
    mag_pub.publish(mag_msg);
  }

  if (rpy_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = header;
    rpy_msg.vector.x = r.euler.get_scaled(1);
    rpy_msg.vector.y = r.euler.get_scaled(0);
    rpy_msg.vector.z = -r.euler.get_scaled(2);
    rpy_pub.publish(rpy_msg);
  }

  /*
  if (temp_pub.getNumSubscribers() > 0)
  {
    std_msgs::Float32 temp_msg;
    temp_msg.data = r.temperature.get_scaled(0);
    temp_pub.publish(temp_msg);
  }
  */
}


/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "um7_driver");

  // Load parameters from private node handle.
  std::string port;
  int32_t baud;
  ros::param::param<std::string>("port", port, "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A7045P4L-if00-port0");
  ros::param::param<int32_t>("baud", baud, 115200);

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

  ros::NodeHandle n;
  std_msgs::Header header;
  ros::param::param<std::string>("frame_id", header.frame_id, "imu_link");

  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      ser.open();
    }
    catch(const serial::IOException& e)
    {
      ROS_DEBUG("Unable to connect to port.");
    }
    if (ser.isOpen())
    {
      ROS_INFO("Successfully connected to serial port.");
      first_failure = true;
      try
      {
        ROS_INFO("In try.");
        um7::Comms sensor(&ser);
        configureSensor(&sensor);
        ROS_INFO("Configured sensor.");
        um7::Registers registers;
        ros::ServiceServer srv = n.advertiseService<um7::Reset::Request, um7::Reset::Response>(
                                   "reset", boost::bind(handleResetService, &sensor, _1, _2));
        ROS_INFO("Reset complete.");

        while (ros::ok())
        {
          //ROS_INFO("Waiting for trigger packet.");
          ROS_INFO_STREAM(sensor.receive(&registers));
          if (sensor.receive(&registers) == TRIGGER_PACKET)
          {
            // Triggered by arrival of final message in group.
            header.stamp = ros::Time::now();
            publishMsgs(registers, &n, header);
            ros::spinOnce();
          }
        }
      }
      catch(const std::exception& e)
      {
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("Attempting reconnection after error.");
        ros::Duration(1.0).sleep();
      }
    }
    else
    {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device "
                           << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }
  }
}
