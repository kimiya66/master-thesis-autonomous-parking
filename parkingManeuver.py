#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import interface
import argparse
import math
import random
import time 
import struct, binascii
import config
import maneuver
import numpy
import weakref




def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))
def main():
    vehicle = None
    data=None
    vehicle1=None
    vehicle2=None
    vehicle3=None
    vehicle4=None
    vehicle5=None
    actor_list = []
    sensors = []

   
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()
    
    try:
       client = carla.Client(args.host, args.port)
       client.set_timeout(1.0)
       world = client.get_world()
       ourMap = world.get_map()
       spectator = world.get_spectator();

       #*************** deffinition of sensors ****************************
       lidar_blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
       camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
       obstacleSensor_blueprint = world.get_blueprint_library().find('sensor.other.obstacle')
      
       #*********** definition of blueprints for vehicles *******************
       blueprint = world.get_blueprint_library().find('vehicle.audi.tt') #for main(ego) vehicle
       blueprint.set_attribute('role_name', 'hero')
       #*********** for non-player vehicles *********************************
       non_playerBlueprint1 = random.choice(world.get_blueprint_library().filter('vehicle.bmw.grandtourer'))
       non_playerBlueprint2 = random.choice(world.get_blueprint_library().filter('vehicle.tesla.*'))
       non_playerBlueprint4 = random.choice(world.get_blueprint_library().filter('vehicle.nissan.micra'))
       non_playerBlueprint3 = random.choice(world.get_blueprint_library().filter('vehicle.toyota.*'))


#******************************* set weather **********************************
       weather = carla.WeatherParameters(
        cloudyness=0.0,
        precipitation=0.0,
        sun_altitude_angle=70.0,
        sun_azimuth_angle=50.0)

       world.set_weather(weather)


#************************ spawn non-player vehicles ******************************

       class spawn_vehicle:
        def __init__(self, blueprint, x, y):
            self.vehicle=None;
            spawn_points = ourMap.get_spawn_points()
            spawn_point = spawn_points[30];
            spawn_point.location.x += x;
            spawn_point.location.y += y;
            self.vehicle = world.spawn_actor(blueprint, spawn_point)

     
       
       # first non-player vehicle
       data=spawn_vehicle(non_playerBlueprint2, -12, 1.0);
       vehicle1=data.vehicle;
       actor_list.append(vehicle1)


       # 2nd non-player vehicle
       data=spawn_vehicle(non_playerBlueprint1, -20, 1.0);
       vehicle2=data.vehicle;
       actor_list.append(vehicle2)

       # 3rd non-player vehicle
       data=spawn_vehicle(non_playerBlueprint3, 2.0, -1.5);
       vehicle3=data.vehicle;
       actor_list.append(vehicle3)

       #4th nonplayer vehicle
       data=spawn_vehicle(non_playerBlueprint4, 10.0, 1.0);
       vehicle4=data.vehicle;
       actor_list.append(vehicle4)
      

       #********************** spawn main vehicle *****************************
       data=spawn_vehicle(blueprint, -24.0, -1.5);
       vehicle=data.vehicle;
       actor_list.append(vehicle)
      
      
       
       #************************ camera-sensor settings ***********************
       camera_location = carla.Transform(carla.Location(x=1.2, z=1.7))
       camera_blueprint.set_attribute('sensor_tick', '0.4');
       camera_sensor = world.spawn_actor(camera_blueprint, camera_location, attach_to=vehicle);


       #=======================================obstacle sensors for maneuver==============================================================


       class ObstacleSensor(object):
        def __init__(self, parent_actor,x,y,z,angle):
          self.sensor = None
          self._parent = parent_actor
          self.actor = 0.0
          self.distance = 0.0
          self.x=x
          self.y=y
          self.z=z
          self.angle=angle;
          world = self._parent.get_world()
          bp = world.get_blueprint_library().find('sensor.other.obstacle')
          bp.set_attribute('debug_linetrace', 'true');
          self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=self.x, y=self.y, z=self.z), carla.Rotation(yaw=self.angle)), attach_to=self._parent);
          sensors.append(self.sensor);
          weak_self = weakref.ref(self)
          self.sensor.listen(lambda event: ObstacleSensor._on_event(weak_self, event))

        @staticmethod
        def _on_event(weak_self, event):
         self = weak_self()
         if not self:
            return
         self.actorId = event.other_actor.id
         self.actorName = event.other_actor.type_id
         self.distance = event.distance



       forward_sensor = ObstacleSensor(vehicle, x=1.5, y=0.0, z=0.6, angle=180);
       rear_sensor = ObstacleSensor(vehicle, x=-1.3, y=0.9, z=0.6, angle=90);
       center_sensor = ObstacleSensor(vehicle, x=0.0, y=0.9, z=0.6, angle=90);
       back_sensor = ObstacleSensor(vehicle, x=-1.3, y=0.0, z=0.6, angle=180);
      

       def control():
     

        if hasattr(rear_sensor, 'actorId') and rear_sensor.distance < 0.5:
          print('stop');
          '''angularVel=vehicle.get_angular_velocity();
          angularVel.x = angularVel.y = angularVel.z = 0;
          vehicle.set_angular_velocity(angularVel);
          vel=vehicle.get_velocity();
          vel.x = vel.y = vel.z = 0;
          vehicle.set_velocity(vel);'''
          vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0));
          control=vehicle.get_control();
          throttle = control.throttle;
          brake=control.brake;
          print('throttle', throttle, 'brake', brake)
          print('rear_sensor info:', rear_sensor.actorId, rear_sensor.actorName, rear_sensor.distance);
        else:
          #print('go!');
          vehicle.apply_control(carla.VehicleControl(throttle=0.5));
        if hasattr(back_sensor, 'actorId'):
          print('back_sensor info:', back_sensor.actorId, back_sensor.actorName, back_sensor.distance);

       #control();


   


   

       

       #set simulator view to the location of the vehicle
       while True:
        time.sleep(0.1)
        control();
        spectator.set_transform(get_transform(vehicle.get_location(), -180))



    finally:
      print('\ndestroying %d actors' % len(actor_list))
      for actor in actor_list:
        if actor is not None:
          actor.destroy()
      for sensor in sensors:
        if sensor is not None:
          sensor.destroy()
      




if __name__ == '__main__':
  try:
    main();
  except KeyboardInterrupt:
    pass;
  finally:
    print('\ndone.');