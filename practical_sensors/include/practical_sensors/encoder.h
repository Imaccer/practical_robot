#ifndef PRACTICAL_SENSORS_INCLUDE_PRACTICAL_SENSORS_ENCODER_H_
#define PRACTICAL_SENSORS_INCLUDE_PRACTICAL_SENSORS_ENCODER_H_

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <string>

class Encoder {
 public:
  Encoder(ros::NodeHandle&, int pi, unsigned int encoderPin,
          unsigned int reversePin, const std::string& wheelName);
  ~Encoder();

  void setupCallback();
  void eventCallback(int pi, unsigned int gpio, unsigned int edge,
                     unsigned int tick, void* userdata);
  void publishCount();
  bool isInitialized() const;
  static void eventCallbackWrapper(int pi, unsigned int gpio, unsigned int edge,
                                   unsigned int tick, void* userdata);

 private:
  unsigned int encoderPin_;
  int callbackId_;
  int pi_;
  unsigned int reversePin_;
  std_msgs::Int16 count_;
  std::string wheelName_;

  static const int encoderMin = -32768;
  static const int encoderMax = 32767;
  void updateCount(bool reverse);

  ros::Publisher pub_;
  ros::NodeHandle nh_;
};
#endif  // PRACTICAL_SENSORS_INCLUDE_PRACTICAL_SENSORS_ENCODER_H_
