#include <ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>


#define motor 10
#define rWheel 5
#define lWheel 6
#define wheelSpeed 5

ros::NodeHandle nh;

int moveR = 0, moveL = 0;
float x = 0, y = 0, avgs = 0;
bool detected = false;

void MotorCbX( const std_msgs::Float64& msg);
void MotorCbY( const std_msgs::Float64& msg);
void twistCb( const geometry_msgs::Twist& twist);

ros::Subscriber<std_msgs::Float64> subX("dix_x", &MotorCbX);
ros::Subscriber<std_msgs::Float64> subY("dix_y", &MotorCbY);
ros::Subscriber <geometry_msgs::Twist> subT("cmd_vel", &twistCb);

void MotorCbX( const std_msgs::Float64& msg){
    x += msg.data * 100;

    detected = true;
    while(detected){
      analogWrite(10, getVoltage(getAvg()));
      delay(3000);  
      analogWrite(10,0);
      detected = false;
    }
}

void MotorCbY( const std_msgs::Float64& msg) {
  y += msg.data * 100;
}

void twistCb( const geometry_msgs::Twist& twist) {
  moveR = (twist.linear.x - twist.angular.z);
  moveL = (twist.linear.x + twist.angular.z);
  if (moveR < 0) moveR *= -1;
  if (moveL < 0) moveL *= -1;

  analogWrite(rWheel, moveR+1);
  analogWrite(lWheel, moveL+1);
}

void setup()
{
  pinMode(lWheel, OUTPUT);
  pinMode(rWheel, OUTPUT);
  pinMode(motor, OUTPUT);
  
  nh.initNode();
  nh.subscribe(subX);
  nh.subscribe(subY);
  nh.subscribe(subT);
}


void loop() {
  while(!detected) {
    nh.spinOnce();
  }
}

float getAvg() {
    float avg = (x + y) / 2;
    avgs = (avgs + avg) / 2;
    return avgs * 1.15;
}

float getVoltage(float n) {
    float x[] = {100, 108, 115, 121, 128, 135, 144, 151, 158, 165, 171, 178, 183, 190, 196, 203, 208, 212, 216, 223, 230, 235, 237, 244, 250, 256, 262, 274, 283, 300, 310, 315, 322, 328, 335, 340};
    float y[] = {80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200, 205, 210, 215, 220, 225, 230, 235, 240, 245, 250, 255};
    
    for(int i = 0; i < sizeof(x)-1; i++) {
        if(x[i] <= n && x[i+1] > n)
            return (y[i+1]*(n-x[i])-y[i]*(n-x[i+1])) / (x[i+1] - x[i]);
    }
}
