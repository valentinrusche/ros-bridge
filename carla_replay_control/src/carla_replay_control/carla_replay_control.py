#!/usr/bin/env python
#
# Copyright (c) 2023 Valentin Rusche
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import print_function

from threading import Thread
import signal
from time import sleep
from typing import NoReturn

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.exceptions import *
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

import carla

from carla_msgs.msg import CarlaWorldInfo


class StopReplay(Exception):
    pass

class ReplayControl(CompatibleNode):

    def __init__(self) -> None:
        super(ReplayControl, self).__init__("carla_replay_control")
        self.log_file: str = str(self.get_param("log_file"))
        self.start: int = int(self.get_param("start"))
        self.duration: int = int(self.get_param("duration"))
        self.camera_id: int = int(self.get_param("camera_id"))
        self.replay_sensors: int = bool(self.get_param("replay_sensors"))
        self.world = None

        self.connect_to_carla()
        self.replay_file(self.log_file, self.start, self.duration, self.camera_id, self.replay_sensors)

    def connect_to_carla(self) -> None:

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo(f"CARLA world available. Trying to connect to {host}:{port}")

        self.carla_client = carla.Client(host=host, port=port)
        self.carla_client.set_timeout(timeout)

        try:
            self.world = self.carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")


    def replay_file(self, path: str, start: int, duration: int, camera_id: int, replay_sensors: bool) -> None:
        while self.world is None:
            self.loginfo("World not available. Cannot start replay. Sleeping for 5 seconds")
            sleep(5.0)
        # register StopReplayHandler to signal
        self.thread = Thread(target=self.stop_replay_and_shutdown)
        self.thread.start()

        try:
           self.loginfo("Starting replay of file {} with start {} and duration {}".format(path, start, duration))
        #    self.carla_client.set_replayer_time_factor(1.0)
        #    self.carla_client.set_replayer_ignore_hero(False)
        #    self.carla_client.set_replayer_ignore_spectator(True)
           self.loginfo(self.carla_client.replay_file(path, start, duration, camera_id, replay_sensors))
        finally:
            pass

    def stop_replay_and_shutdown(self) -> None:
        sleep(float(self.duration))
        self.carla_client.stop_replayer()
        self.destroy_node()

def main(args=None) -> None:
    """
    main function
    """
    roscomp.init("replay_control", args=args)


    replay_control_node = ReplayControl()

    executor = roscomp.executors.MultiThreadedExecutor()
    executor.add_node(replay_control_node)

    spin_thread = Thread(target=replay_control_node.spin)
    spin_thread.start()

    roscomp.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
