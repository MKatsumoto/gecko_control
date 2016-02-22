/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * GeckoControl.h
 *
 *  GeckoControl class provides reference velocity based on
 * IMU data and adapt to slope environment.
 *
 *  Here, "velocity" means
 *   1. Base velocity (translational and rotaional)
 *   2. 4 flippers velocity (front and rear, left and right)
 *
 *  Reference velocity from other nodes are subscribed here,
 * and methods of this program modify them.
 *
 *
 * Author: Masahiro Katsumoto
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#ifndef GECKOCONTROL_H_20160214_
#define GECKOCONTROL_H_20160214_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "gecko_msgs/BaseVelocity.h"
#include "gecko_msgs/FlipperVelocity.h"
#include "gecko_msgs/MbedTx.h"

/* ---------- Data Structure ---------- */
struct WheelVelocity
{
  int16_t left;
  int16_t right;
};

struct Orientation
{
  double roll;
  double pitch;
  double yaw;
};

/* ---------- Class Definition ---------- */
class GeckoControl
{
public:
  GeckoControl(); // Initialize publisher and subscriber

private:
  ros::NodeHandle nh_;
  ros::Subscriber base_velocity_sub_;
  ros::Subscriber flipper_velocity_sub_;
  ros::Subscriber base_orientation_sub_;
  ros::Publisher  mbed_pub_;

  gecko_msgs::BaseVelocity base_velocity_;
  gecko_msgs::FlipperVelocity flipper_velocity_;
  Orientation base_orientation_;

  double WHEEL_RADIUS_;
  double TREAD_;

  // Callback function
  void baseVelocityCallback(const gecko_msgs::BaseVelocity::ConstPtr& msg);
  void flipperVelocityCallback(const gecko_msgs::FlipperVelocity::ConstPtr& msg);
  void baseOrientationCallback(const sensor_msgs::Imu::ConstPtr& msg);

  // Helper Functions
  void adaptVelocity2Slope(const gecko_msgs::BaseVelocity& velocity, gecko_msgs::BaseVelocity* modified_velocity);
  WheelVelocity baseVelocity2wheelVelocity(const gecko_msgs::BaseVelocity base_velocity);
  void wheelVelocity2mbedTxData(const WheelVelocity& wheel_velocity, gecko_msgs::MbedTx* tx_data);
  void flipperVelocity2mbedTxData(const gecko_msgs::FlipperVelocity& flipper_velocity, gecko_msgs::MbedTx* tx_data);
  inline void imposeVelocityLimit(int8_t& velocity)
  {
    if(velocity > 100) velocity = 100;
    else if(velocity < -100) velocity = -100;
  } // velocity is in [-100, 100]
};

#endif  // GECKOCONTROL_H_20160214_
