#include "practical_sensors/encoder.h"
#include <ros/ros.h>
#include <pigpiod_if2.h>
#include <std_msgs/Int16.h>

Encoder::Encoder(ros::NodeHandle& nh, int pii, unsigned int encoderPin, unsigned int reversePin, const std::string& wheelName)
    : nh_(nh), pi_(pii), encoderPin_(encoderPin), reversePin_(reversePin), wheelName_(wheelName)
{
    set_mode(pi_, encoderPin_, PI_INPUT);
    set_mode(pi_, reversePin_, PI_INPUT);
    set_pull_up_down(pi_, encoderPin_, PI_PUD_UP);
    set_pull_up_down(pi_, reversePin_, PI_PUD_UP);

    count_.data = 0;
    pub_ = nh_.advertise<std_msgs::Int16>(wheelName_, 1000);
}

Encoder::~Encoder() {
  std::cout << "Cancelling callback: " << wheelName_ << std::endl;
    callback_cancel(callbackId_);
}

bool Encoder::isInitialized() const {
 return (this != nullptr);
}

void Encoder::setupCallback() {
    //callbackId_ = callback(pi_, encoderPin_, EITHER_EDGE, reinterpret_cast<CBFunc_t>(&Encoder::eventCallback));
    callbackId_ = callback_ex(pi_, encoderPin_, EITHER_EDGE, &Encoder::eventCallbackWrapper, static_cast<void*>(this));

    if (callbackId_ >= 0) {
      std::cout << "Callback set up successfully for " << wheelName_ << std::endl;
      std::cout << "callbackId_ : "<< callbackId_ << std::endl;
    } else {
      std::cerr << "Failed to set up callback for " << wheelName_ << std::endl;
    }

    // Print the 'this' pointer
    std::cout << "Inside setupCallback. 'this' pointer: " << this << std::endl;
}

void Encoder::eventCallbackWrapper(int pi, unsigned int gpio, unsigned int edge, unsigned int tick, void* userdata) {
 // Cast the userdata back to Encoder and call the actual eventCallback
 if (userdata) {
   Encoder* encoder = static_cast<Encoder*>(userdata);
   encoder->eventCallback(pi, gpio, edge, tick, userdata);
    }
}
                             
void Encoder::eventCallback(int pi, unsigned int gpio, unsigned int edge, unsigned int tick, void* userdata) {
    std::cout << "Inside setupCallback. 'this' pointer: " << this << std::endl;
    std::cout << "gpio_read output: " << this->reversePin_ << std::endl;
    bool reverse = (gpio_read(pi, reversePin_) == 0);

    if (reverse) {
        updateCount(true);
    } else {
        updateCount(false);
    }
}

void Encoder::updateCount(bool reverse) {
    if (reverse) {
        if (count_.data == encoderMin) {
            count_.data = encoderMax;
        } else {
            count_.data--;
        }
    } else {
        if (count_.data == encoderMax) {
            count_.data = encoderMin;
        } else {
            count_.data++;
        }
    }
}

void Encoder::publishCount() {
    pub_.publish(count_);
}


