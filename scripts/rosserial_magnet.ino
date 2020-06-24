// the setup function runs once when you press reset or power the board

#include <ros.h>
#include <std_msgs/Int32.h>
#define MAGNET_PIN 2

ros::NodeHandle nh;

void serialIoCallback(const std_msgs::Int32 &msg){
  if (msg.data == 0){
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(MAGNET_PIN, LOW);
  }
  else{
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(MAGNET_PIN, HIGH);
  }
}

ros::Subscriber<std_msgs::Int32> sub("serial_io", &serialIoCallback );

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, LOW);

  nh.initNode();
  nh.subscribe(sub);
}

// the loop function runs over and over again forever
void loop() {
  nh.spinOnce();
  delay(0.01);
}
