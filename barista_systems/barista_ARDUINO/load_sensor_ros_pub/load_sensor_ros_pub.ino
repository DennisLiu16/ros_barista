/*
 * Load Sensor Publisher ROS
 */
#include <HX711.h>
#include <ros.h>
#include <std_msgs/Float32.h>

HX711 scale(3, 2);
ros::NodeHandle  nh;

float calibration_factor = -375; // this calibration factor is adjusted according to my load cell
float units;

std_msgs::Float32 float32_msg;
ros::Publisher chatter("load_sensor", &float32_msg);

void setup()
{
  // Load sensor setup  
  scale.set_scale();
  scale.tare();  //Reset the scale to 0, Error Unable to sync with device 
  
  nh.initNode();
  nh.advertise(chatter);
}

void updateUnits(){
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  units = scale.get_units(), 10;
  if (units < 0)
  {
    units = 0.00;
  }
}

void loop()
{
  // Load Sensor Readings
  updateUnits();

  // ROS Publishers
  float32_msg.data = units;
  chatter.publish( &float32_msg );
  nh.spinOnce();
  delay(20);
}
