#include <pigpiod_if2.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "practical_sensors/encoder.h"

using ::testing::Return;
using ::testing::_;

// Mocking the ros::NodeHandle class for testing purposes
class MockNodeHandle : public ros::NodeHandle {
 public:
  MockNodeHandle() : ros::NodeHandle() {}
};

// Mocking the ros::Publisher class for testing purposes
class MockPublisher : public ros::Publisher {
 public:
  MockPublisher() : ros::Publisher() {}
};

// Mocking the pigpiod_if2 functions for testing purposes
class MockPigpiodIf2 {
 public:
  MOCK_METHOD4(callback_ex, int(int, unsigned int, unsigned int, CBFunc_t));
  MOCK_METHOD3(set_mode, int(int, unsigned int, unsigned int));
  MOCK_METHOD3(set_pull_up_down, int(int, unsigned int, unsigned int));
  MOCK_METHOD3(gpio_read, int(int, unsigned int, unsigned int));
  MOCK_METHOD2(pigpio_start, int(const char*, const char*));
  MOCK_METHOD1(pigpio_stop, int(int));
  // Add other mock function declarations as needed
};


class EncoderTest : public ::testing::Test {
protected:
    MockNodeHandle nh;
    MockPublisher pub;
    MockPigpiodIf2 pigpiodMock;
    Encoder encoder;

public:
    EncoderTest() : encoder(nh, 1, 2, 3, "testWheel") {
        encoder.setPublisher(pub);
        encoder.setPigpiodIf2(&pigpiodMock);
    }
};

// Test case to check if the Encoder object is initialized correctly
//TEST_F(EncoderTest, Initialization) { EXPECT_TRUE(encoder.isInitialized()); }

/*
// Test case to check if setupCallback method is setting up the callback
// correctly
TEST_F(EncoderTest, SetupCallback) {
  // Mocking the callback_ex function to return a positive value
//  ON_CALL(pigpiodMock, callback_ex(_, _, _, _)).WillByDefault(Return(1));

  // Expect setupCallback to set up the callback successfully
  EXPECT_NO_THROW(encoder.setupCallback());
}

// Test case to check if eventCallback is updating count correctly
TEST_F(EncoderTest, EventCallback) {
  // Mocking gpio_read to return a specific value
  ON_CALL(pigpiodMock, gpio_read(_, _)).WillByDefault(Return(0));

  // Call the eventCallback and expect count to be updated accordingly
  encoder.eventCallback(1, 2, 3, 4, nullptr);
  EXPECT_EQ(encoder.count_.data, Encoder::encoderMax);

  // Mock gpio_read to return a different value
  ON_CALL(encoder, gpio_read(_, _)).WillByDefault(Return(1));

  // Call the eventCallback and expect count to be updated accordingly
  encoder.eventCallback(1, 2, 3, 4, nullptr);
  EXPECT_EQ(encoder.count_.data, Encoder::encoderMin);
  
}
*/
/*
// Test case to check if publishCount is publishing correctly
TEST_F(EncoderTest, PublishCount) {
  // Mocking ros::Publisher's publish method
  ON_CALL(pub, publish(_)).WillByDefault(Return());

  // Call publishCount and expect publish to be called without errors
  EXPECT_NO_THROW(encoder.publishCount());
}
*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "encoder_test_node");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
