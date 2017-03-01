#include "swiftnav_piksi/piksi_driver.h"
#include <libsbp/system.h>
#include <libsbp/navigation.h>

#include <iomanip>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <ros/time.h>
#include <tf/tf.h>

namespace swiftnav_piksi
{	
	PIKSI::PIKSI( const ros::NodeHandle &_nh,
		const ros::NodeHandle &_nh_priv,
		const std::string _port ) :
		nh( _nh ),
		nh_priv( _nh_priv ),
		port( _port ),
		frame_id( "gps" ),
		piksid( -1 ),

        heartbeat_diag(nh, nh_priv, "ppiksi_time_diag"),
        llh_diag(nh, nh_priv, "ppiksi_llh_diag"),
        rtk_diag(nh, nh_priv, "ppiksi_rtk_diag"),

		min_llh_rate( 0.5 ),
		max_llh_rate( 50.0 ),
		min_rtk_rate( 0.5 ),
		max_rtk_rate( 50.0 ),
		min_heartbeat_rate( 0.5 ),
		max_heartbeat_rate( 50.0 ),

		llh_pub_freq( diagnostic_updater::FrequencyStatusParam(
                    &min_llh_rate, &max_llh_rate, 0.1, 50 ) ),
		rtk_pub_freq( diagnostic_updater::FrequencyStatusParam( 
                    &min_rtk_rate, &max_rtk_rate, 0.1, 50 ) ),
		heartbeat_pub_freq( diagnostic_updater::FrequencyStatusParam( 
                    &min_rtk_rate, &max_rtk_rate, 0.1, 50 ) ),

		io_failure_count( 0 ),
		last_io_failure_count( 0 ),
		open_failure_count( 0 ),
		last_open_failure_count( 0 ),
        heartbeat_flags( 0 ),

        num_llh_satellites( 0 ),
        llh_status( 0 ),
        llh_lat( 0.0 ),
        llh_lon( 0.0 ),
        llh_height( 0.0 ),
        llh_h_accuracy( 0.0 ),
        hdop( 1.0 ),

        rtk_status( 0 ),
        num_rtk_satellites( 0 ),
        rtk_north( 0.0 ),
        rtk_east( 0.0 ),
        rtk_height( 0.0 ),
        rtk_h_accuracy( 0.04 ),     // 4cm

		spin_rate( 2000 ),      // call sbp_process this fast to avoid dropped msgs
		spin_thread( &PIKSI::spin, this )
	{
		cmd_lock.unlock( );
		heartbeat_diag.setHardwareID( "piksi heartbeat" );
        heartbeat_diag.add( heartbeat_pub_freq );

		llh_diag.setHardwareID( "piksi lat/lon" );
		llh_diag.add( llh_pub_freq );

		rtk_diag.setHardwareID( "piksi rtk" );
		rtk_diag.add( "Piksi Status", this, &PIKSI::DiagCB );
		rtk_diag.add( rtk_pub_freq );

		nh_priv.param( "frame_id", frame_id, (std::string)"gps" );
	}

	PIKSI::~PIKSI( )
	{
		spin_thread.interrupt( );
		PIKSIClose( );
	}

	bool PIKSI::PIKSIOpen( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		return PIKSIOpenNoLock( );
	}

	bool PIKSI::PIKSIOpenNoLock( )
	{
		if( piksid >= 0 )
			return true;

		piksid = piksi_open( port.c_str( ) );

		if( piksid < 0 )
		{
			open_failure_count++;
			return false;
		}

		sbp_state_init(&state);
		sbp_state_set_io_context(&state, &piksid);

        sbp_register_callback(&state, SBP_MSG_HEARTBEAT, &heartbeat_callback, (void*) this, &heartbeat_callback_node);
        sbp_register_callback(&state, SBP_MSG_GPS_TIME, &time_callback, (void*) this, &time_callback_node);
//		sbp_register_callback(&state, SBP_POS_ECEF, &pos_ecefCallback, (void*) this, &pos_ecef_callback_node);
        sbp_register_callback(&state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*) this, &pos_llh_callback_node);
        sbp_register_callback(&state, SBP_MSG_DOPS, &dops_callback, (void*) this, &dops_callback_node);
//		sbp_register_callback(&state, SBP_BASELINE_ECEF, &baseline_ecefCallback, (void*) this, &baseline_ecef_callback_node);
        sbp_register_callback(&state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*) this, &baseline_ned_callback_node);
//		sbp_register_callback(&state, SBP_VEL_ECEF, &vel_ecefCallback, (void*) this, &vel_ecef_callback_node);
		sbp_register_callback(&state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*) this, &vel_ned_callback_node);

		llh_pub = nh.advertise<sensor_msgs::NavSatFix>( "gps/fix", 1 );
		rtk_pub = nh.advertise<nav_msgs::Odometry>( "gps/rtkfix", 1 );
		time_pub = nh.advertise<sensor_msgs::TimeReference>( "gps/time", 1 );

		return true;
	}

	void PIKSI::PIKSIClose( )
	{
		boost::mutex::scoped_lock lock( cmd_lock );
		PIKSICloseNoLock( );
	}

	void PIKSI::PIKSICloseNoLock( )
	{
		int8_t old_piksid = piksid;
		if( piksid < 0 )
		{
			return;
		}
		piksid = -1;
		piksi_close( old_piksid );
		if( llh_pub )
			llh_pub.shutdown( );
		if( time_pub )
			time_pub.shutdown( );
	}

	void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}
		
		msg_heartbeat_t hb = *(msg_heartbeat_t*) msg;

		class PIKSI *driver = (class PIKSI*) context;
        driver->heartbeat_pub_freq.tick();
        driver->heartbeat_flags |= (hb.flags & 0x7);    // accumulate errors for diags

		return;
	}

	void time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		msg_gps_time_t time = *(msg_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
        

		return;
	}

	void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		msg_pos_llh_t llh = *(msg_pos_llh_t*) msg;

		sensor_msgs::NavSatFixPtr llh_msg( new sensor_msgs::NavSatFix );

		llh_msg->header.frame_id = driver->frame_id;
		llh_msg->header.stamp = ros::Time::now( );

		llh_msg->status.status = 0;
		llh_msg->status.service = 1;

		llh_msg->latitude = llh.lat;
		llh_msg->longitude = llh.lon;
		llh_msg->altitude = llh.height;

        // populate the covariance matrix
        // FIXME: llh.h/v_accuracy doesn't work yet, so use HDOP temporarily
        // knowing that it's wrong, but in the ballpark
        //double h_covariance = llh.h_accuracy * llh.h_accuracy;
        //double v_covariance = llh.v_accuracy * llh.v_accuracy;
        double h_covariance = driver->hdop * driver->hdop;
        double v_covariance = driver->hdop * driver->hdop;
        llh_msg->position_covariance[0]  = h_covariance;   // x = 0, 0 
        llh_msg->position_covariance[4]  = h_covariance;   // y = 1, 1 
        llh_msg->position_covariance[8]  = v_covariance;   // z = 2, 2 

		driver->llh_pub.publish( llh_msg );

        // populate diagnostic data
		driver->llh_pub_freq.tick( );
        driver->llh_status |= llh.flags;
        driver->num_llh_satellites = llh.n_sats;
        driver->llh_lat = llh.lat;
        driver->llh_lon = llh.lon;
        driver->llh_height = llh.height;
        // FIXME: llh_h_accuracy doesn't work yet, so use hdop
        //driver->llh_h_accuracy = llh.h_accuracy / 1000.0;
        driver->llh_h_accuracy = driver->hdop;

		return;
	}

	void dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}
		
		msg_dops_t dops = *(msg_dops_t*) msg;

		class PIKSI *driver = (class PIKSI*) context;

        // FIXME: this is incorrect, but h_accuracy doesn't work yet
        driver->llh_h_accuracy = dops.hdop;
        //driver->heartbeat_pub_freq.tick();

		return;
	}

/*	void baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{

		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		// driver->piksi_pub_freq.tick( );

		return;
	}
*/
	void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{

		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		msg_baseline_ned_t sbp_ned = *(msg_baseline_ned_t*) msg;

		nav_msgs::OdometryPtr rtk_odom_msg( new nav_msgs::Odometry );

		rtk_odom_msg->header.frame_id = driver->frame_id;
        // For best accuracy, header.stamp should maybe get tow converted to ros::Time
		rtk_odom_msg->header.stamp = ros::Time::now( );

        // convert to meters from mm, and NED to ENU
		rtk_odom_msg->pose.pose.position.x = sbp_ned.e/1000.0;
		rtk_odom_msg->pose.pose.position.y = sbp_ned.n/1000.0;
		rtk_odom_msg->pose.pose.position.z = -sbp_ned.d/1000.0;

        // Set orientation to 0; GPS doesn't provide orientation
        rtk_odom_msg->pose.pose.orientation.x = 0;
        rtk_odom_msg->pose.pose.orientation.y = 0;
        rtk_odom_msg->pose.pose.orientation.z = 0;
        rtk_odom_msg->pose.pose.orientation.w = 0;

        float h_covariance = 1.0e3;
        float v_covariance = 0.0e3;

        // populate the pose covariance matrix if we have a good fix
        if ( 1 == sbp_ned.flags && 4 < sbp_ned.n_sats)
        {
            // FIXME: h_accuracy doesn't work yet, so use hard-coded 4cm
            // until it does
            //h_covariance = (sbp_ned.h_accuracy * sbp_ned.h_accuracy) / 1.0e-6;
            //v_covariance = (sbp_ned.v_accuracy * sbp_ned.v_accuracy) / 1.0e-6;
            h_covariance = driver->rtk_h_accuracy * driver->rtk_h_accuracy;
            v_covariance = driver->rtk_h_accuracy * driver->rtk_h_accuracy;
        }

        // Pose x/y/z covariance is whatever we decided h & v covariance is
        rtk_odom_msg->pose.covariance[0]  = h_covariance;   // x = 0, 0 in the 6x6 cov matrix
        rtk_odom_msg->pose.covariance[7]  = h_covariance;   // y = 1, 1
        rtk_odom_msg->pose.covariance[14] = v_covariance;  // z = 2, 2

        // default angular pose to unknown
        rtk_odom_msg->pose.covariance[21] = 1.0e3;  // x rotation = 3, 3
        rtk_odom_msg->pose.covariance[28] = 1.0e3;  // y rotation = 4, 4
        rtk_odom_msg->pose.covariance[35] = 1.0e3;  // z rotation = 5, 5

        // Populate linear part of Twist with last velocity reported: by vel_ned_callback
        rtk_odom_msg->twist.twist.linear.x = driver->rtk_vel_east;
        rtk_odom_msg->twist.twist.linear.y = driver->rtk_vel_north;
        rtk_odom_msg->twist.twist.linear.z = driver->rtk_vel_up;

        // Set angular velocity to 0 - GPS doesn't provide angular velocity
        rtk_odom_msg->twist.twist.angular.x = 0;
        rtk_odom_msg->twist.twist.angular.y = 0;
        rtk_odom_msg->twist.twist.angular.z = 0;

        // set up the Twist covariance matrix
        // FIXME: I don't know what the covariance of linear velocity should be.
        // 12/19 asked on swiftnav google group
        // GPS doesn't provide rotationl velocity
        rtk_odom_msg->twist.covariance[0]  = 1.0e3;   // x velocity = 0, 0 in the 6x6 cov matrix
        rtk_odom_msg->twist.covariance[7]  = 1.0e3;   // y velocity = 1, 1
        rtk_odom_msg->twist.covariance[14] = 1.0e3;  // z velocity = 2, 2
        rtk_odom_msg->twist.covariance[21] = 1.0e3;  // x rotational velocity = 3, 3
        rtk_odom_msg->twist.covariance[28] = 1.0e3;  // y rotational velocity = 4, 4
        rtk_odom_msg->twist.covariance[35] = 1.0e3;  // z rotational velocity = 5, 5

		driver->rtk_pub.publish( rtk_odom_msg );

        // save diagnostic data
		driver->rtk_pub_freq.tick( );
        driver->rtk_status = sbp_ned.flags;
        driver->num_rtk_satellites = sbp_ned.n_sats;
		driver->rtk_north = rtk_odom_msg->pose.pose.position.x;
		driver->rtk_east = rtk_odom_msg->pose.pose.position.y;
        driver->rtk_height = rtk_odom_msg->pose.pose.position.z;
        // FIXME: rtk.h_accuracy doesn't work yet
        //driver->rtk_h_accuracy = rtk.h_accuracy / 1000.0;

		return;
	}
/*
	void vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		sbp_gps_time_t time = *(sbp_gps_time_t*) msg;

		sensor_msgs::TimeReferencePtr time_msg( new sensor_msgs::TimeReference );

		time_msg->header.frame_id = driver->frame_id;
		time_msg->header.stamp = ros::Time::now( );

		time_msg->time_ref.sec = time.tow;
		time_msg->source = "gps";

		driver->time_pub.publish( time_msg );
		// driver->piksi_pub_freq.tick( );

		return;
	}
*/

	void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
	{
		if ( context == NULL )
		{
			std::cerr << "Context void, OHSHIT" << std::endl;
			return;
		}

		class PIKSI *driver = (class PIKSI*) context;

		msg_vel_ned_t sbp_vel = *(msg_vel_ned_t*) msg;

        // save velocity in the Twist member of a private Odometry msg, from where it
        // will be pulled to populate a published Odometry msg next time a
        // msg_baseline_ned_t message is received
        driver->rtk_vel_north = sbp_vel.n/1000.0;
        driver->rtk_vel_east = sbp_vel.e/1000.0;
        driver->rtk_vel_up = -sbp_vel.d/1000.0;

		return;
	}

	void PIKSI::spin( )
	{
		while( ros::ok( ) )
		{
			boost::this_thread::interruption_point( );
			PIKSI::spinOnce( );
			heartbeat_diag.update( );
			llh_diag.update( );
			rtk_diag.update( );
			spin_rate.sleep( );
		}
	}

	void PIKSI::spinOnce( )
	{
		int ret;

		cmd_lock.lock( );
		if( piksid < 0 && !PIKSIOpenNoLock( ) )
		{
			cmd_lock.unlock( );
			return;
		}

		ret = sbp_process( &state, &read_data );
		cmd_lock.unlock( );
	}

	void PIKSI::DiagCB( diagnostic_updater::DiagnosticStatusWrapper &stat )
	{
		stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "PIKSI status OK" );
		boost::mutex::scoped_lock lock( cmd_lock );

		if( piksid < 0 && !PIKSIOpenNoLock( ) )
		{
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected" );
			return;
		}
		else if( open_failure_count > last_open_failure_count )
        {
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, 
                            "Open Failure Count Increase" );
        }
		else if( io_failure_count > last_io_failure_count )
        {
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
                            "I/O Failure Count Increase" );
        }
        else if( 0 != heartbeat_flags & 0x7 )
        {
			stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, 
                            "Piksi Error indicated by heartbeat flags" );
        }
        else if( num_rtk_satellites < 5 )
        {
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
                            "RTK Satellite fix invalid: too few satellites in view" );
        }
        else if( rtk_status != 1 )
        {
			stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
                            "No GPS RTK fix" );
        }

		stat.add( "io_failure_count", io_failure_count );
		last_io_failure_count = io_failure_count;

		stat.add( "open_failure_count", open_failure_count );
		last_open_failure_count = open_failure_count;

        stat.add( "Heartbeat status (0 = good)", heartbeat_flags);
        stat.add( "Number of satellites used in GPS RTK solution", num_rtk_satellites );
        stat.add( "GPS RTK solution status (1 = good)", rtk_status );
        stat.add( "GPS RTK meters north", rtk_north );
        stat.add( "GPS RTK meters east", rtk_east );
        stat.add( "GPS RTK height difference (m)", rtk_height );
        stat.add( "GPS RTK horizontal accuracy (m)", rtk_h_accuracy );
        stat.add( "GPS RTK velocity north", rtk_vel_north );
        stat.add( "GPS RTK velocity east", rtk_vel_east );
        stat.add( "GPS RTK velocity up", rtk_vel_up );
        stat.add( "Number of satellites used for lat/lon", num_llh_satellites);
        stat.add( "GPS lat/lon solution status", llh_status );
        stat.add( "GPS latitude", llh_lat );
        stat.add( "GPS longitude", llh_lon );
        stat.add( "GPS altitude", llh_height );
        stat.add( "GPS lat/lon horizontal accuracy (m)", llh_h_accuracy);
	}

}
