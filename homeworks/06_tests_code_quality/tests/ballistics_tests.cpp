#include "../include/ballistics.hpp"

#include <gtest/gtest.h>
#include <iostream>

TEST(Ballistics, SampleTest)
{
  std::cerr << "-------------H is: " << get_h(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f) << std::endl;

  EXPECT_NEAR(1, 1, 0.01);
}