// Copyright 2026 BXI Robotics
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <thread>

#include "bxi_imu/reader_scheduling.hpp"

TEST(ReaderScheduling, DisabledAndInvalidRequestsKeepCurrentPolicy)
{
  const int policy = sched_getscheduler(0);
  sched_param before{};
  ASSERT_EQ(sched_getparam(0, &before), 0);
  EXPECT_EQ(bxi_imu::set_reader_priority(0), 0);
  EXPECT_EQ(bxi_imu::set_reader_priority(-1), EINVAL);
  EXPECT_EQ(bxi_imu::set_reader_priority(100), EINVAL);
  sched_param after{};
  ASSERT_EQ(sched_getparam(0, &after), 0);
  EXPECT_EQ(sched_getscheduler(0), policy);
  EXPECT_EQ(after.sched_priority, before.sched_priority);
}

TEST(ReaderScheduling, ChangesOnlyTheCallingThreadOrReportsPermissionError)
{
  const int parent_policy = sched_getscheduler(0);
  sched_param parent_parameters{};
  ASSERT_EQ(sched_getparam(0, &parent_parameters), 0);
  int result = 0;
  int child_policy = -1;
  sched_param child_parameters{};
  std::thread reader([&]() {
      result = bxi_imu::set_reader_priority(20);
      child_policy = sched_getscheduler(0);
      EXPECT_EQ(sched_getparam(0, &child_parameters), 0);
    });
  reader.join();
  if (result == 0) {
    EXPECT_EQ(child_policy, SCHED_RR);
    EXPECT_EQ(child_parameters.sched_priority, 20);
  } else {
    EXPECT_EQ(result, EPERM);
    EXPECT_EQ(child_policy, parent_policy);
    EXPECT_EQ(child_parameters.sched_priority, parent_parameters.sched_priority);
  }
  sched_param after{};
  ASSERT_EQ(sched_getparam(0, &after), 0);
  EXPECT_EQ(sched_getscheduler(0), parent_policy);
  EXPECT_EQ(after.sched_priority, parent_parameters.sched_priority);
}
