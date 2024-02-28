#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;

int M1_1 = 2;
int M1_2 = 4;
int M1_en= 3;

int M2_1 = 5;
int M2_2 = 7;
int M2_en= 6;

int M3_1 = 8;
int M3_2 = 9;
int M3_en= 10;

int M4_1 = 12;
int M4_2 = 13;
int M4_en= 11;

geometry_msgs::Twist cmd_msg;
ros::Publisher pub("/cmd_vel_M", &cmd_msg);

void kinematics_callback(const geometry_msgs::Twist& cmd_msg){
  digitalWrite(M1_en, cmd_msg.linear.x);
  digitalWrite(M2_en, cmd_msg.linear.y);
  digitalWrite(M3_en, cmd_msg.linear.z);
  digitalWrite(M4_en, cmd_msg.angular.x);

  digitalWrite(M1_1, (cmd_msg.linear.x > 0));
  digitalWrite(M2_1, (cmd_msg.linear.y > 0));
  digitalWrite(M3_1, (cmd_msg.linear.z > 0));
  digitalWrite(M4_1, (cmd_msg.angular.x> 0));

  digitalWrite(M1_2, (cmd_msg.linear.x < 0));
  digitalWrite(M2_2, (cmd_msg.linear.y < 0));
  digitalWrite(M3_2, (cmd_msg.linear.z < 0));
  digitalWrite(M4_2, (cmd_msg.angular.x< 0));
  
  pub.publish(&cmd_msg);
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &kinematics_callback);

void setup(){
  pinMode(M1_1, OUTPUT);
  pinMode(M2_1, OUTPUT);
  pinMode(M3_1, OUTPUT);
  pinMode(M4_1, OUTPUT);
  
  pinMode(M1_2, OUTPUT);
  pinMode(M2_2, OUTPUT);
  pinMode(M3_2, OUTPUT);
  pinMode(M4_2, OUTPUT);
  
  pinMode(M1_en, OUTPUT);
  pinMode(M2_en, OUTPUT);
  pinMode(M3_en, OUTPUT);
  pinMode(M4_en, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
