# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import robomaster
from robomaster import robot
import math

radius = 0.05  # meter


def sub_esc_info_handler(esc_info):
    global sensor_speed_ms
    speed, angle, timestamp, state = esc_info
    avg_speed_rpm = (
        abs(speed[0]) + abs(speed[1]) + abs(speed[2]) + abs(speed[3])
    ) / len(speed)
    avg_speed_rps = avg_speed_rpm / 60
    sensor_speed_ms = round(avg_speed_rps * (2 * math.pi * radius), 4)
    # print(sensor_speed_ms)


if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    # Subscribe to chassis ESC information
    ep_chassis.sub_esc(freq=10, callback=sub_esc_info_handler)
    checkpoint_speed = 0.25
    slp = 3

    # Turn right front wheel
    # ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
    time.sleep(slp)

    # PID controller constants
    kp = 3  # -0.609 -0.65
    ki = 0
    kd = 0  # -0.00135 0

    accumulate_err = 0
    count = 0
    data_ = []
    for i in range(300):
        err_speed = checkpoint_speed - sensor_speed_ms
        if count >= 1:
            speed = (
                (kp * err_speed)
                + kd * ((prev_err - err_speed) / (60))  # 60 from freq 10 in sub esc
                + ki * (accumulate_err)
            )
            accumulate_err += err_speed
            if speed < 0:
                speed = 0
            print(speed, err_speed)
            # ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
            ep_chassis.drive_speed(x=speed, y=0, z=0, timeout=2)
            # speed_y = (
            #     (p * err_y)
            #     + d * ((prev_err_y - err_y) / (prev_time - after_time))
            #     + i * (accumulate_err_y)
            # )
            # ep_gimbal.drive_speed(pitch_speed=-speed_y, yaw_speed=speed_x)
            # data_pith_yaw.append(
            #     list(list_of_data)
            #     + [err_x, err_y, round(speed_x, 3), round(speed_y, 3)]
            # )

        count += 1
        # prev_time = time.time()
        prev_err = err_speed
        time.sleep(0.001)

    ep_chassis.unsub_esc()

    ep_robot.close()
