#include "../include/ballistics.hpp"

#include <gtest/gtest.h>

TEST(Ballistics, ComputesValidFireXY)
{
  const BallisticInput bi = {
    .xd = 180,
    .yd = 180,
    .zd = 100,
    .targetX = 200,
    .targetY = 200,
    .v0 = 10,
    .accelerationPath = 10,
    .ammo_name = "VOG-17",
  };

  FirePoint fp;
  calculateFirePoint(bi, fp);

  EXPECT_NEAR(fp.fireX, 173.759, 0.01);
  EXPECT_NEAR(fp.fireY, 173.759, 0.01);
}

TEST(Ballistics, InvalidInputCase)
{
  const float mockBadInput = -1.0f;

  const BallisticInput bi = {
    .xd = mockBadInput,
    .yd = mockBadInput,
    .zd = mockBadInput,
    .targetX = mockBadInput,
    .targetY = mockBadInput,
    .v0 = mockBadInput,
    .accelerationPath = mockBadInput,
    .ammo_name = "BAD_AMMO",
  };

  FirePoint fp = {
    .fireX = mockBadInput,
    .fireY = mockBadInput,
    .hasManeuver = false,
    .maneuverX = mockBadInput,
    .maneuverY = mockBadInput,
    .success = false,
  };

  calculateFirePoint(bi, fp);

  EXPECT_EQ(fp.fireX, mockBadInput);
  EXPECT_EQ(fp.fireY, mockBadInput);
  EXPECT_EQ(fp.success, false);
}