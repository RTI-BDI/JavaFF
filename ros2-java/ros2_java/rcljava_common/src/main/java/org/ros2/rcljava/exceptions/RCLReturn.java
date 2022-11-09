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

package org.ros2.rcljava.exceptions;

public enum RCLReturn {
  OK(0),
  ERROR(1),
  TIMEOUT(2),
  BAD_ALLOC(10),
  INVALID_ARGUMENT(11),
  ALREADY_INIT(100),
  NOT_INIT(101),
  MISMATCHED_RMW_ID(102),
  TOPIC_NAME_INVALID(103),
  SERVICE_NAME_INVALID(104),
  UNKNOWN_SUBSTITUTION(105),
  NODE_INVALID(200),
  NODE_INVALID_NAME(201),
  NODE_INVALID_NAMESPACE(202),
  PUBLISHER_INVALID(300),
  SUBSCRIPTION_INVALID(400),
  SUBSCRIPTION_TAKE_FAILED(401),
  CLIENT_INVALID(500),
  CLIENT_TAKE_FAILED(501),
  SERVICE_INVALID(600),
  SERVICE_TAKE_FAILED(601),
  TIMER_INVALID(800),
  TIMER_CANCELED(801),
  WAIT_SET_INVALID(900),
  WAIT_SET_EMPTY(901),
  WAIT_SET_FULL(902);

  private final int value;

  private final static RCLReturn[] values = RCLReturn.values();

  RCLReturn(final int value) {
    this.value = value;
  }

  public int getValue() {
    return value;
  }

  public static RCLReturn fromInt(int code) {
    for (RCLReturn value : values) {
      if(value.getValue() == code) {
        return value;
      }
    }
    return null;
  }
}
