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
#include "gecko_control/BaseVelocity.h"
#include "gecko_control/FlipperVelocity.h"
#include "gecko_control/MbedTx.h"

/* ---------- Data Structure ---------- */
struct Velocity
{
  int linear;
  int angular;
};

struct FlipperVelocity
{
  int front_left;
  int rear_left;
  int front_right;
  int rear_right;
};

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
  // Callback function
  void baseVelocityCallback(const gecko_control::BaseVelocity::ConstPtr& msg);         // TODO: check
  void flipperVelocityCallback(const gecko_control::FlipperVelocity::ConstPtr& msg); // TODO: modify
  void baseOrientationCallback(const sensor_msgs::Imu::ConstPtr& msg);                  // TODO: check

private:
  ros::NodeHandle nh_;
  ros::Subscriber base_velocity_sub_;
  ros::Subscriber flipper_velocity_sub_;
  ros::Subscriber base_orientation_sub_;
  ros::Publisher  mbed_pub_;

  Velocity base_velocity_;
  FlipperVelocity flipper_velocity_;
  Orientation base_orientation_;

  double WHEEL_RADIUS_;
  double TREAD_;

  // Helper Functions
  void adaptVelocity2Slope(const Velocity& velocity, Velocity* modified_velocity);
  WheelVelocity baseVelocity2wheelVelocity(const Velocity base_velocity);
  void wheelVelocity2mbedTxData(const WheelVelocity& wheel_velocity, gecko_control::MbedTx* tx_data);
  void flipperVelocity2mbedTxData(const FlipperVelocity& flipper_velocity, gecko_control::MbedTx* tx_data);
  inline void imposeVelocityLimit(int& velocity)
  {
    if(velocity > 100) velocity = 100;
    else if(velocity < -100) velocity = -100;
  } // velocity is in [-100, 100]
};

#endif  // GECKOCONTROL_H_20160214_
