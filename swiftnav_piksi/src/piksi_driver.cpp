#include "swiftnav_piksi/piksi_driver.h"
#include <libsbp/system.h>
#include <libsbp/navigation.h>

#include <swiftnav_piksi_msgs/SbpBaseline.h>
#include <swiftnav_piksi_msgs/SbpDops.h>
#include <swiftnav_piksi_msgs/SbpGpsTime.h>
#include <swiftnav_piksi_msgs/SbpPosLlh.h>
#include <swiftnav_piksi_msgs/SbpVelNed.h>
#include <gps_common/GPSFix.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>

namespace swiftnav_piksi
{
  Piksi::Piksi(const ros::NodeHandle& nh):
    nh_(nh),
    piksid_(-1),
    utc_offset_(-18), // As of 2016-12-31
    spin_rate_(2000),
    open_failure_count_(0),
    publish_invalid_fixes_(false)
  {
    pnh_ = ros::NodeHandle("~");
    pnh_.param("port", port_, std::string("/dev/ttyUSB0"));
    pnh_.param("baud", baud_, 152000);
    pnh_.param("frame_id", frame_id_, std::string("piksi_gps"));
    pnh_.param("publish_invalid_fixes", publish_invalid_fixes_, false);
    
    dop_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpDops>("piksi/dops", 2);
    llh_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpPosLlh>("piksi/pos_llh", 2);
    rtk_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpBaseline>("piksi/baseline", 2);
    time_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpGpsTime>("piksi/time", 2);
    vel_pub_ = nh_.advertise<swiftnav_piksi_msgs::SbpVelNed>("piksi/vel", 2);

    gps_fix_pub_ = nh_.advertise<gps_common::GPSFix>("piksi/gps", 2);
    
    //initial values must be different and not potentially valid
    gps_fix_sync_.dops_time = 0;
    gps_fix_sync_.pos_llh_time = 1;
    gps_fix_sync_.vel_time = 2;

    ROS_INFO("Opening Piksi on %s at %d baud", port_.c_str(), baud_);
    if(!PiksiOpen())
    {
      ROS_ERROR( "Failed to open Piksi on %s at %d baud", port_.c_str(), baud_);
      ros::shutdown();
      return;
    }
    else
    {
      ROS_INFO( "Piksi opened successfully on %s at %d baud", port_.c_str(), baud_);
    }

    // If we reach this point, we should be able to start the spin thread
    boost::thread t(boost::bind(&Piksi::spin, this));
    spin_thread_.swap(t);
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
    
    time_msg->weeks = time.wn;
    time_msg->time_of_week = time.tow;
    time_msg->nano = time.ns_residual;
    time_msg->time_source = (time.flags & 0x0007);
    std::string desc("None");
    if (time_msg->time_source)
    {
      desc = std::string("GNSS");
    }
    time_msg->time_source_description = desc;

    driver->time_pub_.publish(time_msg);

    return;
  }

  //assure that all parts of last_gps_fix_ are from the same time stamp
  void attempt_gps_fix_pub(class Piksi* driver)
  {
    if (driver->gps_fix_sync_.pos_llh_time == driver->gps_fix_sync_.dops_time)
    {
      if (driver->gps_fix_sync_.pos_llh_time == driver->gps_fix_sync_.vel_time)
      {
        if (driver->publish_invalid_fixes_ || driver->last_gps_fix_.position_covariance_type >= 0)
        {
          // By default, only publish if there is a fix
          driver->gps_fix_pub_.publish(driver->last_gps_fix_);
        }
      }
    }
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

    driver->gps_fix_sync_.pos_llh_time = llh.tow;

    llh_msg->header.frame_id = driver->frame_id_;
    llh_msg->header.stamp = ros::Time::now( );

    llh_msg->time_of_week = llh.tow;

    llh_msg->latitude = llh.lat;
    llh_msg->longitude = llh.lon;
    llh_msg->altitude = llh.height;

    llh_msg->h_accuracy = static_cast<double>(llh.h_accuracy) * 0.001;
    llh_msg->v_accuracy = static_cast<double>(llh.v_accuracy) * 0.001;

    llh_msg->sats = llh.n_sats;

    llh_msg->raim_repair = ((llh.flags & 0x0080) >> 7);
    llh_msg->fix_type = (llh.flags & 0x007);
    llh_msg->fix_description = driver->GetFixDescription(llh_msg->fix_type);

    driver->llh_pub_.publish(llh_msg);

    driver->last_gps_fix_.header = llh_msg->header;

    //unfortunatly libsbp does not support a message that provides the number
    //of satellites visible...only the number used in the solution
    driver->last_gps_fix_.status.satellites_used = llh.n_sats;
    driver->last_gps_fix_.status.satellites_visible = llh.n_sats;
    driver->last_gps_fix_.status.status = driver->GetFixCode(llh_msg->fix_type);
    
    driver->last_gps_fix_.time = driver->GetUtcTime(llh.tow);
    driver->last_gps_fix_.latitude = llh.lat;
    driver->last_gps_fix_.longitude = llh.lon;
    driver->last_gps_fix_.altitude = llh.height;

    driver->last_gps_fix_.err_horz = llh_msg->h_accuracy;
    driver->last_gps_fix_.err_vert = llh_msg->v_accuracy;
    driver->last_gps_fix_.position_covariance[0] = std::pow(llh_msg->h_accuracy, 2);
    driver->last_gps_fix_.position_covariance[4] = std::pow(llh_msg->h_accuracy, 2);
    driver->last_gps_fix_.position_covariance[8] = std::pow(llh_msg->v_accuracy, 2);
    driver->last_gps_fix_.position_covariance_type = 2;

    attempt_gps_fix_pub(driver);
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
    swiftnav_piksi_msgs::SbpDopsPtr dop_msg(new swiftnav_piksi_msgs::SbpDops);

    driver->gps_fix_sync_.dops_time = dops.tow;

    dop_msg->header.frame_id = driver->frame_id_;
    dop_msg->header.stamp = ros::Time::now();

    dop_msg->time_of_week = dops.tow;
    dop_msg->gdop = static_cast<double>(dops.gdop) * 0.01;
    dop_msg->pdop = static_cast<double>(dops.pdop) * 0.01;
    dop_msg->tdop = static_cast<double>(dops.tdop) * 0.01;
    dop_msg->hdop = static_cast<double>(dops.hdop) * 0.01;
    dop_msg->vdop = static_cast<double>(dops.vdop) * 0.01;
    dop_msg->raim_repair = ((dops.flags & 0x0080) >> 7);
    dop_msg->fix_type = (dops.flags & 0x0007);
    dop_msg->fix_description = driver->GetFixDescription(dop_msg->fix_type);

    driver->dop_pub_.publish(dop_msg);

    driver->last_gps_fix_.gdop = dop_msg->gdop;
    driver->last_gps_fix_.pdop = dop_msg->pdop;
    driver->last_gps_fix_.tdop = dop_msg->tdop;
    driver->last_gps_fix_.hdop = dop_msg->hdop;
    driver->last_gps_fix_.vdop = dop_msg->vdop;

    attempt_gps_fix_pub(driver);
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
    baseline_msg->north = static_cast<double>(sbp_ned.n) * 0.001;
    baseline_msg->east = static_cast<double>(sbp_ned.e) * 0.001;
    baseline_msg->down = static_cast<double>(sbp_ned.d) * 0.001;
    baseline_msg->h_accuracy = static_cast<double>(sbp_ned.h_accuracy) * 0.001;
    baseline_msg->v_accuracy = static_cast<double>(sbp_ned.v_accuracy) * 0.001;
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

    swiftnav_piksi_msgs::SbpVelNedPtr vel_msg(new swiftnav_piksi_msgs::SbpVelNed);

    driver->gps_fix_sync_.vel_time = sbp_vel.tow;

    vel_msg->header.frame_id = driver->frame_id_;
    vel_msg->header.stamp = ros::Time::now();

    vel_msg->time_of_week = sbp_vel.tow;
    vel_msg->vel_north = static_cast<double>(sbp_vel.n) * 0.001;
    vel_msg->vel_east = static_cast<double>(sbp_vel.e) * 0.001;
    vel_msg->vel_down = static_cast<double>(sbp_vel.d) * 0.001;
    vel_msg->h_accuracy = static_cast<double>(sbp_vel.h_accuracy) * 0.001;
    vel_msg->v_accuracy = static_cast<double>(sbp_vel.v_accuracy) * 0.001;
    
    vel_msg->sats = sbp_vel.n_sats;
    vel_msg->velocity_mode = (sbp_vel.flags & 0x0007);
    std::string desc("Invalid");
    if (vel_msg->velocity_mode == 1)
    {
      desc = std::string("Measured Doppler derived");
    }
    else if (vel_msg->velocity_mode == 2)
    {
      desc = std::string("Computed Doppler derived");
    }
    vel_msg->velocity_mode_description = desc;

    driver->vel_pub_.publish(vel_msg);

    // TODO(kkozak): We're using this velocity message to infer heading for now,
    //               but the piksi can output a heading message that can 
    //               possibly be used instead, though it may only give relative 
    //               direction of rover with respect to base, which is not the 
    //               heading that we care about.
    double vx = vel_msg->vel_north;
    double vy = vel_msg->vel_east;
    double speed = std::sqrt(vx*vx + vy*vy);
    driver->last_gps_fix_.speed = speed;

    //piksi track is (-180,180] but we need [0,360)
    double track = std::atan2(vy, vx) * 180.0 / M_PI;
    if (track < 0)
    {
      track = track + 360.0;
    }
    driver->last_gps_fix_.track = track;
    attempt_gps_fix_pub(driver);

    return;
  }


  double Piksi::GetUtcTime(uint32_t ms)
  {
    // TODO(kkozak): This needs to be verified, and offset needs to be captured
    //               automatically, rather than from the default value.
    // 2017-03-03 -- It looks right compared to online UTC clock
    // check for utc_offset_ message
    double seconds = static_cast<double>(ms) * 0.001 + utc_offset_;
    int32_t days = seconds / 86400.0;
    double utc_time = seconds - days * 86400.0;
    return utc_time;
  }

  int16_t Piksi::GetFixCode(uint8_t fix_type)
  {
    int16_t fix_code;
    switch (fix_type)
    {
    case 0:
      fix_code = -1;
      break;
    case 1:
      fix_code = 0;
      break;
    case 2:
      fix_code = 18;
      break;
    case 3:
      fix_code = 96;
      break;
    case 4:
      fix_code = 97;
      break;
    default: 
      fix_code = -1;
    }
    return fix_code;
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
