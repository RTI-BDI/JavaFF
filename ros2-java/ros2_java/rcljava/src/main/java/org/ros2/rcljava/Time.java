/* Copyright 2017-2018 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros2.rcljava;

import java.util.concurrent.TimeUnit;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.time.ClockType;

public final class Time {
  private static final Logger logger = LoggerFactory.getLogger(Time.class);

  static {
    try {
      JNIUtils.loadImplementation(Time.class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  /**
   * Private constructor so this cannot be instantiated.
   */
  private Time() {}

  public static builtin_interfaces.msg.Time now() {
    return Time.now(ClockType.SYSTEM_TIME);
  }

  public static builtin_interfaces.msg.Time now(ClockType clockType) {
    long rclTime = 0;

    switch (clockType) {
      case ROS_TIME:
        throw new UnsupportedOperationException("ROS Time is currently not implemented");
      case SYSTEM_TIME:
        rclTime = rclSystemTimeNow();
        break;
      case STEADY_TIME:
        rclTime = rclSteadyTimeNow();
        break;
    }

    builtin_interfaces.msg.Time msgTime = new builtin_interfaces.msg.Time();
    msgTime.setSec((int) TimeUnit.SECONDS.convert(rclTime, TimeUnit.NANOSECONDS));
    msgTime.setNanosec((int) rclTime % (1000 * 1000 * 1000));
    return msgTime;
  }

  public static long toNanoseconds(builtin_interfaces.msg.Time msgTime) {
    return TimeUnit.NANOSECONDS.convert(msgTime.getSec(), TimeUnit.SECONDS)
        + (msgTime.getNanosec() & 0x00000000ffffffffL);
  }

  public static long difference(
      builtin_interfaces.msg.Time msgTime1, builtin_interfaces.msg.Time msgTime2) {
    long difference =
        (Time.toNanoseconds(msgTime1) - Time.toNanoseconds(msgTime2)) & 0x00000000ffffffffL;
    return difference;
  }

  private static long rclSystemTimeNow() {
    return nativeRCLSystemTimeNow(); // new RuntimeException("Could not get current time");
  }

  private static long rclSteadyTimeNow() {
    return nativeRCLSteadyTimeNow(); // new RuntimeException("Could not get current time");
  }

  private static native long nativeRCLSystemTimeNow();

  private static native long nativeRCLSteadyTimeNow();
}
