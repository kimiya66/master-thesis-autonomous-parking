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
       client.set_timeout(10.0)
       world = client.get_world()
       ourMap = world.get_map()
       #********* setting fixed time stamp for simulation **************
       settings = world.get_settings();
       settings.fixed_delta_seconds = 0.05;
       world.apply_settings(settings);
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

       print(world.get_weather())



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
       data=spawn_vehicle(non_playerBlueprint1, -20, 1.2);
       vehicle2=data.vehicle;
       actor_list.append(vehicle2)

       # 3rd non-player vehicle
       data=spawn_vehicle(non_playerBlueprint3, 2.0, 1.0);
       vehicle3=data.vehicle;
       actor_list.append(vehicle3)

       #4th nonplayer vehicle
       data=spawn_vehicle(non_playerBlueprint4, 10.0, 1.0);
       vehicle4=data.vehicle;
       actor_list.append(vehicle4)
      

       #********************** spawn main vehicle *****************************
       data=spawn_vehicle(blueprint, -24.0, -2.8);
       vehicle=data.vehicle;
       actor_list.append(vehicle)
       control = carla.VehicleControl(throttle=0.5)
       vehicle.apply_control(control)
      
       
       #************************ camera-sensor settings ***********************
       camera_location = carla.Transform(carla.Location(x=1.2, z=1.7))
       camera_blueprint.set_attribute('sensor_tick', '0.5');
       camera_sensor = world.spawn_actor(camera_blueprint, camera_location, attach_to=vehicle);


       #=======================================obstacle sensors for maneuver==============================================================
       #obstacle_sensor for right-side center
       sensor_location = carla.Transform(carla.Location(x=0.0, y=0.9, z=0.6), carla.Rotation(yaw=90))
       obstacleSensor_blueprint.set_attribute('sensor_tick','1.0');
       obstacleSensor_blueprint.set_attribute('distance', '2.0');
       obstacleSensor_blueprint.set_attribute('debug_linetrace', 'true');
       sensor_obstacle_rightSide = world.spawn_actor(obstacleSensor_blueprint, sensor_location, attach_to=vehicle)
       sensors.append(sensor_obstacle_rightSide);


       #obstacle_sensor for rear_right_wheel
       sensor_location = carla.Transform(carla.Location(x=-1.3, y=0.9, z=0.6), carla.Rotation(yaw=90))
       obstacleSensor_blueprint.set_attribute('sensor_tick','1.0');
       obstacleSensor_blueprint.set_attribute('distance', '1.0');
       obstacleSensor_blueprint.set_attribute('debug_linetrace', 'true');
       sensor_obstacle_rearRight = world.spawn_actor(obstacleSensor_blueprint, sensor_location, attach_to=vehicle)
       sensors.append(sensor_obstacle_rearRight);

       

       #obstacle_sensor for rear_center
       sensor_location = carla.Transform(carla.Location(x=-1.3, y=0.0, z=0.6), carla.Rotation(yaw=180))
       obstacleSensor_blueprint.set_attribute('sensor_tick','1.0');
       obstacleSensor_blueprint.set_attribute('distance', '1.0');
       obstacleSensor_blueprint.set_attribute('debug_linetrace', 'true');
       sensor_obstacle_rearCenter = world.spawn_actor(obstacleSensor_blueprint, sensor_location, attach_to=vehicle)
       sensors.append(sensor_obstacle_rearCenter);


       #================================first step:Parking decision=====================================================
       global image;
       #camera_sensor.listen(lambda image: parking_decision(image));

       def parking_decision(data):
        camera_sensor.stop();
        cameraControl = carla.VehicleControl(throttle=0);
        vehicle.apply_control(cameraControl);
        output=interface.decision(data);
        overlapRatio = output['result'];
        print('the rate of Overlap:', overlapRatio);
        if overlapRatio < 0.1:
          print('the parking place is between the next 2 cars');
          parking_preparation();
        else:
          print('no parking place has been detected right now');
          cameraControl = carla.VehicleControl(throttle=0.5);
          vehicle.apply_control(cameraControl);
          print('car is moving');
          camera_sensor.listen(lambda image: parking_decision(image));

    


       #**********************step2: set the start location for parking(position phase)****************************
       def parking_preparation():
        vehicle.apply_control(carla.VehicleControl(throttle=1));
        sensor_obstacle_rightSide.listen(lambda data: detect_non_players(data));

       def detect_non_players(data):
          current_vhcl = data.other_actor.id;
          print('current detected vehicle:',current_vhcl);
          if config.nonplayer != current_vhcl and current_vhcl != 0:
            config.count += 1;
            config.nonplayer = current_vhcl;
            print('number of detected vehicles detected till now:', config.count);

          print('number of detected vehicles till now:', config.count);
          if config.count == 3:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0,steer=0.0));
            print('time of parking');
            frontDistance=data.distance;
            print('dataDistance', frontDistance)
            sensor_obstacle_rearRight.listen(lambda data: stablePosition(frontDistance, data));

            
          def stablePosition(event):
            print('distance to side of detected vehicle.',event.distance, 'OF VEHICLE:',event.other_actor.type_id);
            print('other distance',frontDistance, 'OF VEHICLE:',event.other_actor.type_id);
           if event.distance <= 1:
              vehicle.applyControl(carla.VehicleControl(throttle=0.0,steer=0.0));
              parking_maneuver();





       #***************************step3: parking maneuver*******************************
       def parking_maneuver():
        time.sleep(3);
        maneuver.calculate_maneuverTime(vehicle);
        maneuver.calculate_max_steeringAng(vehicle);
        time.sleep(3);
        print('time duration:',config.T,'max steeringAngle:',config.phi_max);
        sensor_obstacle_rearCenter.listen(lambda data: control_maneuver(data));
        def control_maneuver(data):
          for t in numpy.arange(0,config.T,config.sampling_period):
            if data.distance <= 1.2:
              vehicle.apply_control(carla.VehicleControl(steer=0,throttle=0));
              break;
            maneuver.parking(t,vehicle); #call parking function from maneuver.py
            time.sleep(0.05);
      

       #set simulator view to the location of the vehicle
       while True:
        time.sleep(0.1)
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