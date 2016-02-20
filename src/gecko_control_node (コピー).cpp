#include <ros/ros.h>
#include <gecko_control/mbed_tx.h>
#include <set_velocity_tele/Velocity_tele.h>//subscribe用に必要
//#include "std_msgs/Int16.h"
//#include "std_msgs/UInt8.h"

#define Rw 0.045 // 車輪半径
#define Tr 0.45 // トレッド幅
#define wMAX 2500

using namespace std;

class GeckoControl
{
public:
  GeckoControl();

private:
  void velCallback(const set_velocity_tele::Velocity_tele::ConstPtr& vel);

  ros::NodeHandle nh_;

  ros::Publisher vel_pub_;
  ros::Subscriber vel_sub_;
};

GeckoControl::GeckoControl()
{
//  vel_pub_ = nh_.advertise<gecko_control::Gecko_control>("mbed_tx", 10);
  vel_pub_ = nh_.advertise<gecko_control::mbed_tx>("mbed_tx", 10);
  vel_sub_ = nh_.subscribe<set_velocity_tele::Velocity_tele>("velocity_tele", 10, &GeckoControl::velCallback, this);
};


void GeckoControl::velCallback(const set_velocity_tele::Velocity_tele::ConstPtr& msg)
{
  gecko_control::mbed_tx txdata;
  int16_t data1, data2;

  if((msg->fvel[0] + msg->fvel[1] + msg->fvel[2] + msg->fvel[3]) == 0)
  {
    // クローラの速度を決定
    int16_t w_r, w_l;
    w_r = (int16_t)( (msg->vel + 0.5 *Tr * msg->avel) / Rw);
    w_l = (int16_t)( (msg->vel - 0.5 *Tr * msg->avel) / Rw);
    
    // limitter
    if(w_r > wMAX)  w_r = wMAX;
    else if(w_r < -wMAX)  w_r = -wMAX;
    if(w_l > wMAX)  w_l = wMAX;
    else if(w_l < -wMAX)  w_l = -wMAX;

    data1 = (int16_t)(w_r / (double)wMAX * 32767);
    data2 = (int16_t)(w_l / (double)wMAX * 32767);  // 負号付くかも

    txdata.id = 0x01; // mbed 1
    txdata.command = 0x01;  // crawler 
    txdata.data[0] = (int8_t)(data2 >> 8);
    txdata.data[1] = (int8_t)data2;
    txdata.data[2] = (int8_t)(data1 >> 8);
    txdata.data[3] = (int8_t)data1;
  } else {
    txdata.id = 0x01;
    txdata.command = 0x02;  // flipper (pitch)
    txdata.data[0] = (int8_t)(msg->fvel[0]+1);  // LF
    txdata.data[1] = (int8_t)(msg->fvel[2]+1);  // LR
    txdata.data[2] = (int8_t)(msg->fvel[1]+1);  // RF
    txdata.data[3] = (int8_t)(msg->fvel[3]+1);  // RR
  }

  vel_pub_.publish(txdata);

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gecko_control_node");
  GeckoControl object;

  ros::spin();
}

