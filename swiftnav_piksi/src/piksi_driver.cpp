#include "swiftnav_piksi/piksi_driver.h"
#include <libsbp/system.h>
#include <libsbp/navigation.h>

#include <iomanip>

#include <swiftnav_piksi_msgs/SbpBaseline.h>
#include <swiftnav_piksi_msgs/SbpGpsTime.h>
#include <swiftnav_piksi_msgs/SbpPosLlh.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>

namespace swiftnav_piksi
{
  Piksi::Piksi(const ros::NodeHandle& nh):
    nh_(nh),
    piksid_(-1),
    spin_rate_(2000),
    spin_thread_(&Piksi::spin, this),
    open_failure_count_(0)
  {
    pnh_ = ros::NodeHandle("~");
    pnh_.param("port", port_, std::string("/dev/ttyUSB0"));
    pnh_.param("baud", baud_, 152000);
    pnh_.param("frame_id", frame_id_, std::string("piksi_gps"));
    
    llh_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpPosLlh>("piksi/pos_llh", 2);
    rtk_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpBaseline>("piksi/baseline", 2);
    time_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpGpsTime>("piksi/time", 2);

    ROS_INFO("Opening Piksi on %s at %d baud", port_.c_str( ), baud_);
    if(!PiksiOpen( ))
    {
      ROS_ERROR( "Failed to open Piksi on %s at %d baud", port_.c_str( ), baud_);
    }
    else
    {
      ROS_INFO( "Piksi opened successfully on %s at %d baud", port_.c_str( ), baud_);
    }
  }

  Piksi::~Piksi()
  {
    spin_thread_.interrupt();
    PiksiClose();
  }

  bool Piksi::PiksiOpen()
  {
    boost::mutex::scoped_lock lock(cmd_lock);
    return PiksiOpenNoLock();
  }

  bool Piksi::PiksiOpenNoLock()
  {
    if (piksid_ >= 0)
    {
      return true;
    }

    piksid_ = piksi_open(port_.c_str(), baud_);

    if (piksid_ < 0)
    {
      open_failure_count_++;
      return false;
    }

    sbp_state_init(&state_);
    sbp_state_set_io_context(&state_, &piksid_);

    sbp_register_callback(&state_, SBP_MSG_HEARTBEAT,    &heartbeat_callback,    (void*) this, &heartbeat_callback_node);
    sbp_register_callback(&state_, SBP_MSG_GPS_TIME,     &time_callback,         (void*) this, &time_callback_node);
    sbp_register_callback(&state_, SBP_MSG_POS_LLH,      &pos_llh_callback,      (void*) this, &pos_llh_callback_node);
    sbp_register_callback(&state_, SBP_MSG_DOPS,         &dops_callback,         (void*) this, &dops_callback_node);
    sbp_register_callback(&state_, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*) this, &baseline_ned_callback_node);
    sbp_register_callback(&state_, SBP_MSG_VEL_NED,      &vel_ned_callback,      (void*) this, &vel_ned_callback_node);
    
    return true;
  }

  void Piksi::PiksiClose()
  {
    boost::mutex::scoped_lock lock(cmd_lock);
    PiksiCloseNoLock();
  }

  void Piksi::PiksiCloseNoLock()
  {
    int8_t old_piksid = piksid_;
    if (piksid_ < 0)
    {
      return;
    }
    piksid_ = -1;
    piksi_close(old_piksid);

    if (llh_pub_)
    {
      llh_pub_.shutdown();
    }

    if (time_pub_)
    {
      time_pub_.shutdown();
    }
  }

  void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void* context)
  {
    if (context == NULL)
    {
      ROS_ERROR("Context void. Exiting heartbeat callback.");
      return;
    }

    msg_heartbeat_t hb = *(msg_heartbeat_t*) msg;

    class Piksi* driver = (class Piksi*) context;
    
    // Nothing to do right now.

    return;
  }

  void time_callback(u16 sender_id, u8 len, u8 msg[], void* context)
  {
    if (context == NULL)
    {
      ROS_ERROR("Context void. Exiting Time callback.");
      return;
    }

    class Piksi* driver = (class Piksi*) context;

    msg_gps_time_t time = *(msg_gps_time_t*) msg;

    swiftnav_piksi_msgs::SbpGpsTimePtr time_msg( new swiftnav_piksi_msgs::SbpGpsTime );

    time_msg->header.frame_id = driver->frame_id_;
    time_msg->header.stamp = ros::Time::now( );

    time_msg->time_of_week = time.tow;
    time_msg->nano = time.ns;
    time_msg->time_source = (time.flags & 0x0007);

    driver->time_pub_.publish(time_msg);

    return;
  }

  void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void* context)
  {
    if (context == NULL)
    {
      ROS_ERROR("Context void. Exiting pos_llh callback.");
      return;
    }

    class Piksi* driver = (class Piksi*) context;

    msg_pos_llh_t llh = *(msg_pos_llh_t*) msg;

    swiftnav_piksi_msgs::SbpPosLlhPtr llh_msg( new swiftnav_piksi_msgs::SbpPosLlh );

    llh_msg->header.frame_id = driver->frame_id_;
    llh_msg->header.stamp = ros::Time::now( );

    llh_msg->time_of_week = llh.tow;

    llh_msg->latitude = llh.lat;
    llh_msg->longitude = llh.lon;
    llh_msg->altitude = llh.height;

    llh_msg->h_accuracy = static_cast<double>(llh.h_accuracy) / 1000.0;
    llh_msg->v_accuracy = static_cast<double>(llh.v_accuracy) / 1000.0;

    llh_msg->sats = llh.n_sats;

    //flags
    llh_msg->raim_repair = ((llh.flags & 0x0080) >> 7);
    llh_msg->fix_type = (llh.flags & 0x007);
    llh_msg->fix_description = driver->GetFixDescription(llh_msg->fix_type);

    driver->llh_pub_.publish(llh_msg);
    return;
  }

  void dops_callback(u16 sender_id, u8 len, u8 msg[], void* context)
  {
    if (context == NULL)
    {
      ROS_ERROR("Context void. Exiting dops_callback.");
      return;
    }
		
    msg_dops_t dops = *(msg_dops_t*) msg;

    class Piksi* driver = (class Piksi*) context;

    // Nothing to do here at them moment.

    return;
  }

  void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void* context)
  {
    if (context == NULL)
    {
      ROS_ERROR("Context void. Exiting baseline_ned_callback.");
      return;
    }

    class Piksi* driver = (class Piksi*) context;

    msg_baseline_ned_t sbp_ned = *(msg_baseline_ned_t*) msg;

    swiftnav_piksi_msgs::SbpBaselinePtr baseline_msg( new swiftnav_piksi_msgs::SbpBaseline );

    baseline_msg->header.frame_id = driver->frame_id_;
    // For best accuracy, header.stamp should maybe get tow converted to ros::Time
    baseline_msg->header.stamp = ros::Time::now( );

    baseline_msg->time_of_week = sbp_ned.tow;
    baseline_msg->north = static_cast<double>(sbp_ned.n) / 1000.0;
    baseline_msg->east = static_cast<double>(sbp_ned.e) / 1000.0;
    baseline_msg->down = static_cast<double>(sbp_ned.d) / 1000.0;
    baseline_msg->h_accuracy = static_cast<double>(sbp_ned.h_accuracy) / 1000.0;
    baseline_msg->v_accuracy = static_cast<double>(sbp_ned.v_accuracy) / 1000.0;
    baseline_msg->sats = sbp_ned.n_sats;
    baseline_msg->raim_repair = ((sbp_ned.flags & 0x0080) >> 7);
    baseline_msg->fix_type = (sbp_ned.flags & 0x0007);
    baseline_msg->fix_description = driver->GetFixDescription(baseline_msg->fix_type);

    driver->rtk_pub_.publish(baseline_msg);
    return;
  }

  void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void* context)
  {
    if (context == NULL)
    {
      ROS_ERROR("Context void. Exiting vel_ned_callback");
      return;
    }

    class Piksi* driver = (class Piksi*) context;

    msg_vel_ned_t sbp_vel = *(msg_vel_ned_t*) msg;

    // save velocity in the Twist member of a private Odometry msg, from where it
    // will be pulled to populate a published Odometry msg next time a
    // msg_baseline_ned_t message is received
    //    driver->rtk_vel_north = sbp_vel.n/1000.0;
    //    driver->rtk_vel_east = sbp_vel.e/1000.0;
    //    driver->rtk_vel_up = -sbp_vel.d/1000.0;

    return;
  }

  std::string Piksi::GetFixDescription(uint8_t fix_type)
  {
    std::string fix_description("");
    switch (fix_type)
    {
    case 0:
      fix_description = std::string("Invalid");
      break;
    case 1:
      fix_description = std::string("SPP");
      break;
    case 2:
      fix_description = std::string("DGNSS");
      break;
    case 3:
      fix_description = std::string("Float RTK");
      break;
    case 4:
      fix_description = std::string("Fixed RTK");
      break;
    default: 
      fix_description = "Invalid";
    }
    return fix_description;
  }
  void Piksi::spin()
  {
    while (ros::ok())
    {
      boost::this_thread::interruption_point();
      Piksi::spinOnce();
      spin_rate_.sleep();
    }
  }

  void Piksi::spinOnce()
  {
    int ret;

    cmd_lock.lock();
    if(piksid_ < 0 && !PiksiOpenNoLock())
    {
      cmd_lock.unlock();
      return;
    }

    ret = sbp_process( &state_, &read_data );
    cmd_lock.unlock();
  }
}
