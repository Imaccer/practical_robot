#include <gtest/gtest.h>
#include "practical_sensors/encoder.h"

TEST(TestSuite, testcounter) {
  int count = 2;
  ASSERT_EQ(2, count);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

