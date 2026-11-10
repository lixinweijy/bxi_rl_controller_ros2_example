// Copyright 2026 BXI Robotics
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <pthread.h>
#include <sched.h>

#include <cerrno>

namespace bxi_imu
{

inline int set_reader_priority(int priority)
{
  if (priority < 0 || priority > 99) {
    return EINVAL;
  }
  if (priority == 0) {
    return 0;
  }
  const sched_param parameters{priority};
  return pthread_setschedparam(pthread_self(), SCHED_RR, &parameters);
}

}  // namespace bxi_imu
