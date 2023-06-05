#!/usr/bin/env python
#
# Copyright (c) 2023 Valentin Rusche
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Publishes current state information about the actor vehicle as a service
"""
import math
import carla
import numpy

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header
from carla_msgs.msg import CarlaWorldInfo, CarlaStatus, CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel
from carla_actor_state_types.srv import GetActorState
from carla_msgs.msg import CarlaActorList, CarlaActorInfo

from carla_ros_bridge.ego_vehicle import EgoVehicle


class CarlaActorStatePublisher(CompatibleNode):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaActorStatePublisher, self).__init__(
            'carla_actor_state_publisher')
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.current_tick = 0.0
        self.vehicle_actors = set()

        # initialize ros service
        self.get_state_service = self.new_service(
            GetActorState,
            '/carla_actor_state_publisher/get_actor_state',
            self.get_actor_state)

        # use callback to wait for ego vehicle
        self.loginfo("Waiting for ego vehicle...")

        self.carla_status = CarlaStatus()
        self.status_subscriber = self.new_subscription(
            CarlaStatus,
            "/carla/status",
            self.carla_status_updated,
            qos_profile=10)
        
        self.vehicle_actors_publisher = self.new_publisher(
            CarlaActorList,
            '/carla/all_vehicle_actors',
            qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.on_tick = self.world.on_tick(self.__refresh_actors)

    def destroy(self) -> None:
        """
        Destructor
        """
        self.ego_vehicle = None

    def __refresh_actors(self, _) -> None: # carla expects the second argument
        vehicle_list_changed = False
        current_world_actors = set()
        for actor in self.world.get_actors():
            if actor.type_id.startswith("vehicle"):
                current_world_actors.add(actor)
        if len(self.vehicle_actors) == len(current_world_actors):
            vehicle_list_changed = False
        else:
            self.loginfo("Some vehicle actors changed.")
            vehicle_list_changed = True

        if vehicle_list_changed:
            self.loginfo("Publishing available vehicle actors")
            self.vehicle_actors_publisher.publish(self.__create_vehicle_msg())

    def __create_vehicle_msg(self) -> CarlaActorList:
        msg_actors: list[CarlaActorInfo] = []
        for actor in self.world.get_actors():
            if actor.type_id.startswith("vehicle"):
                actor_info = CarlaActorInfo()
                actor_info.id = actor.id
                actor_info.parent_id = actor.parent.id if not actor.parent is None else 0
                actor_info.type = actor.type_id
                actor_info.rolename = actor.attributes.get('role_name')
                msg_actors.append(actor_info)
                self.vehicle_actors.add(actor)
        
        carla_actor_list_msg = CarlaActorList()
        carla_actor_list_msg.actors = msg_actors
        return carla_actor_list_msg

    def get_actor_state(self, req, response=None):
        """
        Convenience method to get the waypoint for an actor
        """
        actor = self.world.get_actors().find(req.id)

        response = roscomp.get_service_response(GetActorState)
        if actor:
            response.actor_state.current_tick = self.current_tick

            response.actor_state.is_ego_vehicle = actor.attributes.get(
                'role_name') == 'ego_vehicle'

            simulator_waypoint = self.map.get_waypoint(actor.get_location())
            self.append_waypoint_data(waypoint = simulator_waypoint, response = response)

            vehicle_status: CarlaEgoVehicleStatus = self.build_actor_status(actor = actor)

            response.actor_state.vehicle_status = vehicle_status

            vehicle_info: CarlaEgoVehicleInfo = self.build_actor_info(actor = actor)

            response.actor_state.vehicle_info = vehicle_info
        else:
            self.logwarn(
                "get_actor_state(): Actor {} not valid.".format(req.id))
        return response

    def connect_to_carla(self) -> None:

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")

    def carla_status_updated(self, data) -> None:
        """
        Callback on carla status
        """
        self.carla_status = data
        # if self.carla_status.fixed_delta_seconds:
        #     self.current_tick = float(
        #         self.carla_status.fixed_delta_seconds) * float(self.carla_status.frame)
        # else:
        self.current_tick = (1.0 / 20.0) * float(self.carla_status.frame)

    def append_waypoint_data(self, waypoint, response) -> None:
        response.actor_state.current_waypoint.pose = trans.carla_transform_to_ros_pose(
            waypoint.transform)
        response.actor_state.current_waypoint.is_junction = waypoint.is_junction
        response.actor_state.current_waypoint.road_id = waypoint.road_id
        response.actor_state.current_waypoint.section_id = waypoint.section_id
        response.actor_state.current_waypoint.lane_id = waypoint.lane_id

        loc_x = response.actor_state.current_waypoint.pose.position.x
        loc_y = response.actor_state.current_waypoint.pose.position.y
        loc_z = response.actor_state.current_waypoint.pose.position.z
        location = carla.Location(loc_x, loc_y, loc_z)

        geo_location = self.map.transform_to_geolocation(location)

        response.actor_state.geo_latitude = geo_location.latitude
        response.actor_state.geo_longitude = geo_location.longitude
        response.actor_state.geo_altitude = geo_location.altitude

    def build_actor_status(self, actor) -> CarlaEgoVehicleStatus:
        header = Header()
        header.frame_id = str(self.current_tick)
        timestamp = self.get_time()
        header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)

        vehicle_status = CarlaEgoVehicleStatus(header=header)
        vehicle_status.velocity = math.sqrt(
            EgoVehicle.get_vehicle_speed_squared(actor))
        vehicle_status.acceleration.linear = trans.carla_acceleration_to_ros_accel(
        actor.get_acceleration()).linear
        vehicle_status.orientation = trans.carla_transform_to_ros_pose(
            actor.get_transform()).orientation
        vehicle_status.control.throttle = actor.get_control().throttle
        vehicle_status.control.steer = actor.get_control().steer
        vehicle_status.control.brake = actor.get_control().brake
        vehicle_status.control.hand_brake = actor.get_control().hand_brake
        vehicle_status.control.reverse = actor.get_control().reverse
        vehicle_status.control.gear = actor.get_control().gear
        vehicle_status.control.manual_gear_shift = actor.get_control().manual_gear_shift
        return vehicle_status
    
    def build_actor_info(self, actor) -> CarlaEgoVehicleInfo:
        vehicle_info = CarlaEgoVehicleInfo()
        vehicle_info.id = actor.id
        vehicle_info.type = actor.type_id
        vehicle_info.rolename = actor.attributes.get('role_name')
        vehicle_info.wheels = []
        vehicle_physics = actor.get_physics_control()
        for wheel in vehicle_physics.wheels:
            wheel_info = CarlaEgoVehicleInfoWheel()
            wheel_info.tire_friction = wheel.tire_friction
            wheel_info.damping_rate = wheel.damping_rate
            wheel_info.max_steer_angle = math.radians(
                wheel.max_steer_angle)
            wheel_info.radius = wheel.radius
            wheel_info.max_brake_torque = wheel.max_brake_torque
            wheel_info.max_handbrake_torque = wheel.max_handbrake_torque

            inv_T = numpy.array(
                actor.get_transform().get_inverse_matrix(), dtype=float)
            wheel_pos_in_map = numpy.array([wheel.position.x/100.0,
                                            wheel.position.y/100.0,
                                            wheel.position.z/100.0,
                                            1.0])
            wheel_pos_in_ego_vehicle = numpy.matmul(
                inv_T, wheel_pos_in_map)
            wheel_info.position.x = wheel_pos_in_ego_vehicle[0]
            wheel_info.position.y = -wheel_pos_in_ego_vehicle[1]
            wheel_info.position.z = wheel_pos_in_ego_vehicle[2]
            vehicle_info.wheels.append(wheel_info)

        vehicle_info.max_rpm = vehicle_physics.max_rpm
        vehicle_info.max_rpm = vehicle_physics.max_rpm
        vehicle_info.moi = vehicle_physics.moi
        vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
        vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
            vehicle_physics.damping_rate_zero_throttle_clutch_engaged
        vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
            vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
        vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
        vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
        vehicle_info.clutch_strength = vehicle_physics.clutch_strength
        vehicle_info.mass = vehicle_physics.mass
        vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
        vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
        vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
        vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z
        return vehicle_info


def main(args=None) -> None:
    """
    main function
    """
    roscomp.init('carla_actor_state_publisher', args)

    actor_state_publisher = None
    try:
        actor_state_publisher = CarlaActorStatePublisher()
        actor_state_publisher.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if actor_state_publisher:
            actor_state_publisher.destroy()
        roscomp.shutdown()


if __name__ == "__main__":
    main()
