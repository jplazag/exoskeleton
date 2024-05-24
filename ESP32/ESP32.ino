/*
 * Code file for the comunication between the pc and the hardware in the exoskeleton, here it could be fine the definition of the pins that are use
 * to power the motors and how the pwm that control them is used. 
 * 
 */

#include <ros.h>
//#include <ESP32Servo.h>                  //Library for ESP32
#include <Servo.h>                         //Library for Arduino
//#include <std_msgs/Float64.h>/
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle nh;

Servo servo_elbow;

Servo servo_wrist;

#define SERVO_ELBOW_PIN 12

#define SERVO_WRIST_PIN 13

double posServo1 = 500; 
double posServo2 = 2500; 
double toggle = true; 


//////////////////////////////// Maping function ///////////////////////////////////

// This function is in charge of transforming the angle given by the computer into time for the duty cycle of the PWM

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//////////////////////////////// Subscriber ////////////////////////////////////////

// The subscriber function takes the value of the code running in the computer and convert it into the activation time for the PWM, and then
// sends that duty cicle to the servos

void messageCb(const std_msgs::Float64MultiArray &msg)
{
  double subsVal_elbow = 3.14159 - 2 * ( msg.data[0] - 10 * 3.14159 / 180); //We use a mask to convert our 180 in 0 and our 0 in 180 and then minus 10 in order to change the range to 10 - 100
  double subsVal_wrist = 3.14159 - 2 * ( msg.data[1] - 10 * 3.14159 / 180);

  double angle_elbow;
  double angle_wrist; 


  if((subsVal_elbow <= 2 * 1.5707963) && (subsVal_elbow >= 0)){
    angle_elbow = mapf(subsVal_elbow, 0.0,3.14159, posServo1, posServo2);
    servo_elbow.writeMicroseconds(angle_elbow);
  }


  if((subsVal_wrist <= 2 * 1.5707963) && (subsVal_wrist >= 0)){
    angle_wrist = mapf(subsVal_wrist, 0.0,3.14159, posServo1, posServo2);
    servo_wrist.writeMicroseconds(angle_wrist);
  }
 
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("q_control_publisher", &messageCb);

//////////////////////////////// Service server ///////////////////////////////////////

// This function allows to attach and detach the servo in the elbow

void attach_deatach_servo(const std_srvs::SetBool::Request  &serviceRequest, std_srvs::SetBool::Response &serviceResponse)
{
  bool attach_deattach = serviceRequest.data;
  
  if(attach_deattach)
  {
    servo_elbow.attach(SERVO_ELBOW_PIN);
  }else{
    servo_elbow.detach();//
  }
  nh.loginfo("sending back response");
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> service_server("service_server",&attach_deatach_servo); 










void setup()
{
  pinMode(SERVO_ELBOW_PIN, OUTPUT);

  pinMode(SERVO_WRIST_PIN, OUTPUT);
  
  servo_elbow.attach(SERVO_ELBOW_PIN);

  servo_wrist.attach(SERVO_WRIST_PIN);

  nh.getHardware()->setBaud(921600);

  nh.initNode();

  nh.subscribe(sub);


  nh.advertiseService(service_server);


  delay(1);
  
}

void loop()
{  


  nh.spinOnce();
//  delay(1);
}
