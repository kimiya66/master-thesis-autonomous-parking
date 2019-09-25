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
import matlab
from pymatbridge import Matlab





def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def main():
    #start matlab connection
    mlab = Matlab()
    mlab.start()
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

       client.set_timeout(2.0)
       

       world = client.get_world()
       ourMap = world.get_map()
       debug = world.debug;
#********* setting fixed time stamp for simulation **************
       settings = world.get_settings();
       settings.fixed_delta_seconds = 0.05;
       world.apply_settings(settings);
       spectator = world.get_spectator();

#*************** deffinition of sensors ****************************
       
       camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
      
#*********** definition of blueprints for vehicles *******************
       blueprint = world.get_blueprint_library().find('vehicle.audi.tt') #for main(ego) vehicle
       blueprint.set_attribute('role_name', 'hero')
#*********** for non-player vehicles *********************************
       non_playerBlueprint1 = random.choice(world.get_blueprint_library().filter('vehicle.bmw.grandtourer'))
       non_playerBlueprint2 = random.choice(world.get_blueprint_library().filter('vehicle.tesla.*'))
       non_playerBlueprint4 = random.choice(world.get_blueprint_library().filter('vehicle.mercedes-benz.coupe'))
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

     
       


       #first non-player vehicle
       data=spawn_vehicle(non_playerBlueprint1, -20, 1.0);
       vehicle1=data.vehicle;
       actor_list.append(vehicle1)


       #second non-player vehicle
       data=spawn_vehicle(non_playerBlueprint2, -12, 1.0);
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
       data=spawn_vehicle(blueprint, -23.0, -1.5);
       vehicle=data.vehicle;
       actor_list.append(vehicle)
       #set the first movement of ego car
       control = carla.VehicleControl(throttle=0.1)
       vehicle.apply_control(control)
      
       
       #************************ camera-sensor settings ***********************
       camera_location = carla.Transform(carla.Location(x=1.3, z=2.0))
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
          bp.set_attribute('debug_linetrace', 'false');
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



       forward_sensor = ObstacleSensor(vehicle, x=1.3, y=0.9, z=0.6, angle=90);
       rearSide_sensor = ObstacleSensor(vehicle, x=-1.3, y=0.9, z=0.6, angle=90);
       center_sensor = ObstacleSensor(vehicle, x=0.0, y=0.9, z=0.6, angle=90);
       back_sensor = ObstacleSensor(vehicle, x=-1.3, y=0.0, z=0.6, angle=180);
       
      
  


       #====================================step1:Parking Decision==========================================================
       global image;
       time.sleep(5.0);
       textLocation = vehicle.get_location();
       textLocation.x +=5.0;
       textLocation.y -=0.8;
       textLocation.z +=1.8;
       startText = "Search for parking place"
       debug.draw_string(textLocation, startText, True, carla.Color(255,255,0), 3, True);
       textLocation.y +=1.0;
       debug.draw_box(carla.BoundingBox(textLocation,carla.Vector3D(0.2,1.5,0.5)),carla.Rotation(yaw=180), 0.1, carla.Color(255,0,0),3, True);
       camera_sensor.listen(lambda image: parking_decision(image));


       
       def parking_decision(data):
        camera_sensor.stop();
        cameraControl = carla.VehicleControl(throttle=0);
        vehicle.apply_control(cameraControl);
        output=interface.decision(data,mlab);
        overlapRatio = output['result'];
        print('rate of Overlap:', overlapRatio);
        if overlapRatio < 0.1:
          print('parking place is between the next 2 vehicles');
          location = vehicle.get_location();
          location.x +=15;
          location.y +=1.0;
          location.z +=1.8;
          text = 'Parking vacancy detected';
          debug.draw_string(location, text, True, carla.Color(255,255,0), 4.0, True);
          location.y +=1.5;
          debug.draw_box(carla.BoundingBox(location,carla.Vector3D(0.2,2.0,0.8)),carla.Rotation(yaw=180), 0.1, carla.Color(255,0,0),4.0, True);
          time.sleep(3.0);
          mlab.end();
          if hasattr(center_sensor, 'actorId'):
            print('center_sensor info in detection time:', center_sensor.actorName, center_sensor.actorId);
            if center_sensor.actorId != 0:
              number=2;
            else:
              number=1;
          print('matlab disconnected');
          parking_preparation(number);
        else:
          print('no parking place has been detected right now');
          cameraControl = carla.VehicleControl(throttle=0.2);
          vehicle.apply_control(cameraControl);
          print('car is moving');
          camera_sensor.listen(lambda image: parking_decision(image));

  
      

       #==================================step2:Parking Preparation(position phase)=============================================
       def parking_preparation(limit):
        print('limit',limit);
        x1=x2=0;
        vehicle.apply_control(carla.VehicleControl(throttle=0.3));
        while config.count < limit:
          current_vhcl = center_sensor.actorId;
          if config.nonplayer != current_vhcl and current_vhcl != 0:
            config.nonplayer = current_vhcl;
            config.count += 1;
            continue;
        
        else:
          first_parked_car = forward_sensor.actorId;
          print('first_parked_car', first_parked_car);
          while config.count == limit:
            if forward_sensor.actorId == 0:
                  print('first_parked_car passed');
                  config.parkingWidth = forward_sensor.distance; #lateral distance of the parking bay
                  x1 = vehicle.get_location().x; #vehicle first place in parking bay
                  x1 -= 1.3; #because this location is from center of the car
                  print('parkingWidth:', config.parkingWidth, 'x1:', x1);
                  break;
          while forward_sensor.actorId == 0:
            continue;

          else:
            if forward_sensor.actorId != 0 and forward_sensor.actorId != first_parked_car:
                  print('we reached second static car:', forward_sensor.actorId);
                  print('second_parked_car', forward_sensor.actorId);
                  x2 = vehicle.get_location().x; #vehicle second place in parking bay
                  x2 += 1.3; 
                  config.parkingLength = abs(x2 - x1); #length of the parking
                  print('parkingLength:', config.parkingLength, 'x2:', x2);
                  while rearSide_sensor.distance > 1.0:
                    vehicle.apply_control(carla.VehicleControl(steer=0.0, throttle=0.2));
                    time.sleep(1.0);
                    if rearSide_sensor.actorId != 0: #when the static vehicle detected
                      vehicle.apply_control(carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0));
                      parking_maneuver();
                      break;
                  else:
                    vehicle.apply_control(carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0));
                    time.sleep(3.0);
                    parking_maneuver();


       #====================================step3:Parking Maneuver=================================================================
       def parking_maneuver():
        time.sleep(3.0);
        maneuver.calculate_maneuverTime(vehicle);
        maneuver.calculate_max_steeringAng(vehicle);
        time.sleep(3.0);
        for t in numpy.arange(0,config.T,config.sampling_period):
          time.sleep(0.08);
          if (hasattr(back_sensor, 'actorId') and (back_sensor.actorId != 0) and (back_sensor.distance < 4)): #or (hasattr(rearSide_sensor, 'actorId') and (rearSide_sensor.actorId == 0) and (rearSide_sensor.distance < 5.0)):
            print('1****vehicle should be stopped!', back_sensor.distance);
            vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0.0, brake=1.0));
            control=vehicle.get_control();
            print('throttle', control.throttle, 'brake', control.brake, 'steer', control.steer)
          else:
            print('2****move');
            maneuver.parking(t,vehicle); #call parking function from maneuver.py
            control=vehicle.get_control();
            print('throttle', control.throttle, 'brake', control.brake, 'steer', control.steer)
        

       

       #set simulator view to the location of the vehicle
       while True:
        time.sleep(1.0)
        viewLocation = vehicle.get_location();
        viewLocation.z += 1.0;
        spectator.set_transform(get_transform(viewLocation, -180))



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
