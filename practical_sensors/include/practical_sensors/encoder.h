//  Copyright 2024 <Ian McNally>

#ifndef PRACTICAL_SENSORS_INCLUDE_PRACTICAL_SENSORS_ENCODER_H_
#define PRACTICAL_SENSORS_INCLUDE_PRACTICAL_SENSORS_ENCODER_H_

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <string>

class MockPigpiodIf2;

class Encoder {
 public:
  Encoder(ros::NodeHandle&, int pi, unsigned int encoderPin,
          unsigned int reversePin, const std::string& wheelName);
  ~Encoder();

  void setupCallback();
  bool isInitialized() const;

  void eventCallback(int pi, unsigned int gpio, unsigned int edge,
                     unsigned int tick, void* userdata);
  static void eventCallbackWrapper(int pi, unsigned int gpio, unsigned int edge,
                                   unsigned int tick, void* userdata);

  void setPublisher(const ros::Publisher& publisher);
  void publishCount();

  void setPigpiodIf2(MockPigpiodIf2* mockPigpiodIf2);

 private:
  void updateCount(bool reverse);

  ros::Publisher pub_;
  ros::NodeHandle nh_;
  std_msgs::Int16 count_;
  std::string wheelName_;
  MockPigpiodIf2* pigpiodIf2;

  static const int encoderMin = -32768;
  static const int encoderMax = 32767;

  int callbackId_;
  int pi_;
  unsigned int encoderPin_;
  unsigned int reversePin_;
};
#endif  // PRACTICAL_SENSORS_INCLUDE_PRACTICAL_SENSORS_ENCODER_H_
