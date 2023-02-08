#include <ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <Servo.h>

ros::NodeHandle nh;

geometry_msgs::TwistStamped cmd_state;
geometry_msgs::TwistStamped servo_feedback;

const int base1FeedbackPin = A0;
const int base2FeedbackPin = A1;
const int torsoFeedbackPin = A2;
const int headRotFeedbackPin = A3;
const int headTiltFeedbackPin = A4;

const int baseServoPin = 2;
const int baseServo2Pin = 3;
const int torsoServoPin = 4;
const int headRotServoPin = 5;
const int headTiltServoPin = 6;
Servo baseServo1;
Servo baseServo2;
Servo torsoServo;
Servo headRotServo;
Servo headTiltServo;

const int resetPositions[4] = {90, 90, 90, 90};
int basePos = resetPositions[0];
int torsoPos = resetPositions[1];
int headRotPos = resetPositions[2];
int headTiltPos = resetPositions[3];

int baseOffset = 0;
int torsoOffset = 0;
int headRotOffset = 0;
int headTiltOffset = 0;

const int baseLimits[2] = {-68, 90};
const int torsoLimits[2] = {-90, 90};
const int headRotLimits[2] = {-90, 90};
const int headTiltLimits[2] = {-45, 45};

void twist_callback(const geometry_msgs::TwistStamped& twist_msg){
  cmd_state = twist_msg;
  baseOffset = cmd_state.twist.linear.x;
  torsoOffset = cmd_state.twist.linear.y;
  headRotOffset = cmd_state.twist.linear.z;
  headTiltOffset = cmd_state.twist.angular.x;
}

ros::Subscriber<geometry_msgs::TwistStamped> sb_pose_sub("sb_cmd_state", twist_callback);
ros::Publisher feedback_pub("sb_servo_feedback", &servo_feedback);

void setup() {
  nh.loginfo("Started Survivor Buddy hardware setup");
  nh.initNode();
  nh.subscribe(sb_pose_sub);
  nh.advertise(feedback_pub);
  baseServo1.attach(baseServoPin);
  baseServo2.attach(baseServo2Pin);
  torsoServo.attach(torsoServoPin);
  headRotServo.attach(headRotServoPin);
  headTiltServo.attach(headTiltServoPin);
  nh.loginfo("Finished Survivor Buddy hardware setup");

  writeAllServos();
  nh.loginfo("Set Survivor Buddy to upright default position. Defaults can be changed in Arduino code");
}

void loop() {
  writeAllServos();
  readAllFeedbacks();
  nh.spinOnce();
  delay(10);
}

void readAllFeedbacks(){
  servo_feedback.twist.linear.x = analogRead(base1FeedbackPin);
  servo_feedback.twist.linear.y = analogRead(base2FeedbackPin);
  servo_feedback.twist.linear.z = analogRead(torsoFeedbackPin);
  servo_feedback.twist.angular.x = analogRead(headRotFeedbackPin);
  servo_feedback.twist.angular.y = analogRead(headTiltFeedbackPin);
  feedback_pub.publish(&servo_feedback);
}

void writeAllServos(){
  boundsCheckAngles();
  baseServo1.write(basePos);
  baseServo2.write(180-basePos);
  torsoServo.write(torsoPos);
  headRotServo.write(headRotPos);
  headTiltServo.write(headTiltPos);
}


void boundsCheckAngles() {
  if (baseOffset <= baseLimits[0]){
    baseOffset = baseLimits[0];
  } else if (baseOffset >= baseLimits[1]){
    baseOffset = baseLimits[1];
  }

  if (torsoOffset <= torsoLimits[0]){
    torsoOffset = torsoLimits[0];
  } else if (torsoOffset >= torsoLimits[1]){
    torsoOffset = torsoLimits[1];
  }

  if (headRotOffset <= headRotLimits[0]){
    headRotOffset = headRotLimits[0];
  } else if (headRotOffset >= headRotLimits[1]){
    headRotOffset = headRotLimits[1];
  }

  if (headTiltOffset <= headTiltLimits[0]){
    headTiltOffset = headTiltLimits[0];
  } else if (headTiltOffset >= headTiltLimits[1]){
    headTiltOffset = headTiltLimits[1];
  }

  basePos = resetPositions[0] - baseOffset;
  torsoPos = resetPositions[1] - torsoOffset;
  headRotPos = resetPositions[2] - headRotOffset;
  headTiltPos = resetPositions[3] - headTiltOffset;
}