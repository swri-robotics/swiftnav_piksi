/***************************************************************************//**
* \file piksi_driver.hpp
*
* \brief ROS Implementation of the C Driver (header)
* \author Scott K Logan
* \author Caleb Jamison
* \date February 23, 2014
*
* API for the ROS driver
*
* \section license License (BSD-3)
* Copyright (c) 2013, Scott K Logan\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef _piksi_driver_hpp
#define _piksi_driver_hpp

#include "swiftnav_piksi/piksi.h"

#include <libsbp/sbp.h>

#include <ros/ros.h>
#include <ros/rate.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <gps_common/GPSFix.h>

#include <boost/thread.hpp>

namespace swiftnav_piksi
{
  void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
  void attempt_gps_fix_pub(class Piksi* driver);

  struct gps_fix_sync
  {
    uint32_t dops_time;
    uint32_t pos_llh_time;
    uint32_t vel_time;
  };    

  class Piksi
  {
  public:
    Piksi(const ros::NodeHandle& nh);
    ~Piksi();
    bool PiksiOpen();
    void PiksiClose();

  private:
    bool PiksiOpenNoLock();
    void PiksiCloseNoLock();
    std::string GetFixDescription(uint8_t fix_type);
    int16_t GetFixCode(uint8_t fix_type);

    void spin();
    void spinOnce();
    double GetUtcTime(uint32_t ms);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string port_;
    int baud_;
    std::string frame_id_;
    int8_t piksid_;
    int32_t utc_offset_;
    boost::mutex cmd_lock;
    bool publish_invalid_fixes_;

    sbp_state_t state_;
    sbp_msg_callbacks_node_t heartbeat_callback_node;
    sbp_msg_callbacks_node_t time_callback_node;
    sbp_msg_callbacks_node_t pos_llh_callback_node;
    sbp_msg_callbacks_node_t dops_callback_node;
    sbp_msg_callbacks_node_t baseline_ned_callback_node;
    sbp_msg_callbacks_node_t vel_ned_callback_node;

    ros::Publisher dop_pub_;
    ros::Publisher llh_pub_;
    ros::Publisher rtk_pub_;
    ros::Publisher time_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher gps_fix_pub_;

    gps_common::GPSFix last_gps_fix_;

    gps_fix_sync gps_fix_sync_;

    ros::Rate spin_rate_;
    boost::thread spin_thread_;

    uint32_t open_failure_count_;

    friend void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    friend void time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    friend void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    friend void dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    friend void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    friend void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    friend void attempt_gps_fix_pub(class Piksi* driver);

  };
}

#endif /* _piksi_driver_hpp */
