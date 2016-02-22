/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * GeckoControl.cpp
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
 * @ MbedTx.msg
 *    uint8    command
 *    uint8[4] data
 *
 *                << LEFT DATA >>               << RIGHT DATA >>
 * command  | data[0]        data[1]         data[2]       data[3]
 * 0x01     | (left MSB)     (left LSB)      (right MSB)   (right LSB)
 * 0x02     | (front left)   (rear left)     (front right) (rear right)
 *
 *
 * Author: Masahiro Katsumoto
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define DEBUG

#include "gecko_control/GeckoControl.h"
#include <tf/transform_datatypes.h>

/*!
 * \brief Initialize subscriber and publisher
 */
GeckoControl::GeckoControl()
{
  base_velocity_sub_      = nh_.subscribe("base_velocity", 10, &GeckoControl::baseVelocityCallback, this);
  flipper_velocity_sub_   = nh_.subscribe("flipper_velocity", 10, &GeckoControl::flipperVelocityCallback, this);
  base_orientation_sub_   = nh_.subscribe("imu/data", 10, &GeckoControl::baseOrientationCallback, this);
  mbed_pub_ = nh_.advertise<gecko_msgs::MbedTx>("mbed_tx", 10);

  // Get parameters
  if(!nh_.getParam("/wheel_radius", WHEEL_RADIUS_))
  {
    ROS_ERROR("/wheel_radius is not defined!");
  }
  if(!nh_.getParam("/tread", TREAD_))
  {
    ROS_ERROR("/tread is not defined!");
  }
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - *
 *
 *    Callback Functions
 *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/*!
 * Base velocity callback function.
 *
 * Flow of this callback function:
 *  1. Impose limit on velocity
 *  2. Adapt to slope
 *  3. Compute wheel velocity from modified base velocity
 *  4. Format wheel velocity for micro-controller (mbed)
 *  5. Publish the data to mbed_interface_node
 *
 * WARN: Need modification
 */
void GeckoControl::baseVelocityCallback(const gecko_msgs::BaseVelocity::ConstPtr& msg)
{
  base_velocity_ = *msg;

  imposeVelocityLimit(base_velocity_.linear);
  imposeVelocityLimit(base_velocity_.angular);

  gecko_msgs::BaseVelocity reference_velocity;
  adaptVelocity2Slope(base_velocity_, &reference_velocity);

  WheelVelocity wheel_velocity = baseVelocity2wheelVelocity(reference_velocity);

  gecko_msgs::MbedTx tx_data;
  wheelVelocity2mbedTxData(wheel_velocity, &tx_data);

  mbed_pub_.publish(tx_data);
}


/*!
 * \brief GeckoControl::flipperVelocityCallback
 * \param msg
 *
 * TODO: check
 */
void GeckoControl::flipperVelocityCallback(const gecko_msgs::FlipperVelocity::ConstPtr& msg)
{
  flipper_velocity_ = *msg;

  imposeVelocityLimit(flipper_velocity_.front_left);
  imposeVelocityLimit(flipper_velocity_.rear_left);
  imposeVelocityLimit(flipper_velocity_.front_right);
  imposeVelocityLimit(flipper_velocity_.rear_right);

  gecko_msgs::MbedTx tx_data;
  flipperVelocity2mbedTxData(flipper_velocity_, &tx_data);
  mbed_pub_.publish(tx_data);
}


/*!
 * \brief Callback function of base orientation msg from IMU.
 * \param Message from IMU
 *
 * TODO: check
 */
void GeckoControl::baseOrientationCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // AMUから姿勢データ受信
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q); // Quaternion msgからtf::Quaternionに変換
  tf::Matrix3x3(q).getRPY(base_orientation_.roll, base_orientation_.pitch, base_orientation_.yaw);	// tf::Quaternionからroll, pitch, yawを取得
#ifdef DEBUG
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", base_orientation_.roll, base_orientation_.pitch, base_orientation_.yaw);
#endif
}



/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - *
 *
 *    Helper Functions
 *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

// ----- baseVelocityCallback ----- //

/*!
 * \brief GeckoControl::adaptVelocity2Slope
 * \param velocity
 * \param modified_velocity
 *
 * WARN:動作テストのためにとりあえず補正しないようにしてある
 * TODO:Implementation
 * TODO:不感帯への対応？
 * TODO:速度の倍率変更？
 */
void GeckoControl::adaptVelocity2Slope(const gecko_msgs::BaseVelocity &velocity, gecko_msgs::BaseVelocity *modified_velocity)
{
  *modified_velocity = velocity;
//  modified_velocity->linear  = velocity.linear;
//  modified_velocity->angular = velocity.angular;
#ifdef DEBUG
  ROS_INFO("modified_velocity linear: %d, angular: %d", modified_velocity->linear, modified_velocity->angular);
#endif
}


/*!
 * \brief  Convert base velocity to wheel (crawler) velocity.
 * \param  Velocity base_velocity (linear and angular vel.)
 * \return WheelVelocity (left and right wheel vel.)
 * TODO: check
 */
WheelVelocity GeckoControl::baseVelocity2wheelVelocity(const gecko_msgs::BaseVelocity base_velocity)
{
  WheelVelocity wheel_velocity;
  static const float LINEAR_SCALE = 0.5;
  static const float ANGULAR_SCALE = 0.5;
  float scaled_linear_vel  = LINEAR_SCALE  * base_velocity_.linear;   // [100 * m/sec]
  float scaled_angular_vel = ANGULAR_SCALE * base_velocity_.angular;  // [100 * rad/sec]
#ifdef DEBUG
  ROS_INFO("scaled_linear_vel: %f, scaled_angular_vel: %f", scaled_linear_vel, scaled_angular_vel);
#endif
  wheel_velocity.left  = static_cast<int16_t>((scaled_linear_vel - 0.5 * TREAD_ * scaled_angular_vel) / WHEEL_RADIUS_); // [100 * rad/sec]
  wheel_velocity.right = static_cast<int16_t>((scaled_linear_vel + 0.5 * TREAD_ * scaled_angular_vel) / WHEEL_RADIUS_); // [100 * rad/sec]
#ifdef DEBUG
  ROS_INFO("wheel vel L: %d, wheel vel R: %d \n", wheel_velocity.left, wheel_velocity.right);
#endif
  return wheel_velocity;
}

/*!
 * \brief Convert wheel velocity to formatted data for mbed.
 * \param wheel_velocity (input)
 * \param tx_data (output)
 * TODO: check
 */
void GeckoControl::wheelVelocity2mbedTxData(const WheelVelocity &wheel_velocity, gecko_msgs::MbedTx* tx_data)
{
  // TODO: You must check and modify!
  tx_data->command = 0x01;
  tx_data->data[0] = static_cast<uint8_t>(wheel_velocity.left >> 8);
  tx_data->data[1] = static_cast<uint8_t>(wheel_velocity.left);
  tx_data->data[2] = static_cast<uint8_t>(wheel_velocity.right >> 8);
  tx_data->data[3] = static_cast<uint8_t>(wheel_velocity.right);
#ifdef DEBUG
  for (int i = 0; i<3; i++)
  {
    ROS_INFO("Crawler data[%d]:%d \n", i, tx_data->data[i]);
  }
#endif
}


/*!
 * \brief flipperVelocity2mbedTxData
 * \param flipper_velocity
 * \param tx_data
 *
 * NOTE: とりあえずそのまま送信する
 * TODO: check
 */
void GeckoControl::flipperVelocity2mbedTxData(const gecko_msgs::FlipperVelocity& flipper_velocity, gecko_msgs::MbedTx* tx_data)
{
  tx_data->command  = 0x02;
  tx_data->data[0] = flipper_velocity.front_left;
  tx_data->data[1] = flipper_velocity.rear_left;
  tx_data->data[2] = flipper_velocity.front_right;
  tx_data->data[3] = flipper_velocity.rear_right;
#ifdef DEBUG
  for (int i = 0; i<3; i++)
  {
    ROS_INFO("Flipper data[%d]:%d \n", i, tx_data->data[i]);
  }
#endif
}
