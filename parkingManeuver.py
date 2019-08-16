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
import argparse
import math
import random
import time 
import struct, binascii
import config
import numpy



def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))
def main():

    vehicle = None
    vehicle1=None
    vehicle2=None
    vhcl3=None
    vhcl4=None
    vhcl5=None
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
       #+++++setting the fixed time stamp+++++++++
       settings = world.get_settings();
       settings.fixed_delta_seconds = 0.05;
       #eeedddsettings.synchronous_mode = True;
       world.apply_settings(settings);
       spectator = world.get_spectator()
       #snapShot = world.get_snapshot();

       #sensor deffinition
       lidar_blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
       camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
       GnssSensor_blueprint = world.get_blueprint_library().find('sensor.other.gnss')
       obstacleSensor_blueprint = world.get_blueprint_library().find('sensor.other.obstacle')        
       
       blueprint = world.get_blueprint_library().find('vehicle.audi.tt')
       blueprint.set_attribute('role_name', 'hero')
       

       non_blueprint2 = random.choice(world.get_blueprint_library().filter('vehicle.tesla.*'))
       non_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.bmw.grandtourer'))
       non_blueprint5 = random.choice(world.get_blueprint_library().filter('vehicle.nissan.micra'))
       non_blueprint4 = random.choice(world.get_blueprint_library().filter('vehicle.toyota.*'))



       #------------parking maneuver----------------------
       def parking(t):
       
        #print('*****parking control*****');
        #print('time before calling calculate_maneuverTime func:', config.T)
        #+++++++++++++++++calculate maneuver-time and phi_max++++++++++++++++++++++++++++
        
        angle=steeringAngle(t);
        v=velocity(t);
          
        print('steer angle = ', angle,'velocity function:',v);
        
        #physics_control=vehicle.get_physics_control()
        #vehicle.set_simulate_physics(enabled=True);
        velo = vehicle.get_velocity().y;
        control=vehicle.get_control();
        throttle = control.throttle;
        brake = control.brake;
        '''if velo < abs(v):
          print('throttle:',throttle,'control.throttle:',control.throttle);
          throttle = throttle + 0.5;
        else:
          brake = brake + 0.5;
          print('brake value:',control.brake);'''

        #print('throttle=',throttle,'brake=',brake);
        vehicle.apply_control(carla.VehicleControl(throttle=v, manual_gear_shift=True, gear=0,steer=angle,reverse=True));
        #print('vehicle speed of vx:',vehicle.get_velocity().x,'vy:',vehicle.get_velocity().y,'vz:',vehicle.get_velocity().z);
        # print(vehicle.get_speed_limit());
        
        print('steer:',control.steer);




       


       #..............calculation of steeringAngl at each time of maneuver..............
       def steeringAngle(t):
        #print('phi_max which is used here:', config.phi_max);
        
        result=config.phi_max * config.sideOfParking * _A(t);
        #print('A(t)= ', _A(t));
        #print('phi value:', result);
        return result;



      #..............calculation of velocity at each time of maneuver....................
       def velocity(t):
        #print('v_max=',config.v_max,'direction=',config.direction,'B func result:',_B(t));
        for type_error in _B(t):
          
          result = config.v_max * config.direction * type_error;
        #print('result of multiply:',result);
        return result;


      #...............calculation of _A and _B functions to control phi and v...............
       def _A(t):
        #print('what we have as T in A(t): ', config.T);
        #print('T value at the moment: ', config.T, 'value for T-start:', config.T_star);
        t_prime = (config.T - config.T_star)/2; 
        #print('t_prime=',t_prime,'t value = ',t);
        result = 0;     #output of the function
        if 0 <= t < t_prime:
          result = 1;
        elif t_prime <= t <= config.T-t_prime:
          result = math.cos((math.pi*(t-t_prime))/config.T_star)
        elif  config.T-t_prime < t <= config.T:
          result = -1;
        return result;

       def _B(t):
       # print('what we have as T in B(t): ', config.T);
        result = 0,5*(1-math.cos((4*math.pi*t)/config.T));
        return result;


       #.....................calculation of T (whole time of parking maneuver)................
       def calculate_maneuverTime():
        x=vehicle.get_location().x;
        y=vehicle.get_location().y;

        orientAngl=vehicle.get_transform().rotation.yaw;
        ts=0;
        cond = True;
        while cond:
          for ts in numpy.arange(0,config.T,config.sampling_period):
            s_angle = steeringAngle(ts);
            #print('s_angle value:',s_angle);
            velo = velocity(ts);
            if(s_angle == 0):
               orientAngl_lastStep = orientAngl;
               orientAngl = orientAngl;
               x = x + (velo * config.sampling_period * math.cos(orientAngl));
               y = y + (velo * config.sampling_period * math.sin(orientAngl));
               #print('x,y in if-clause:',x,y);
            else:
              orientAngl_lastStep = orientAngl;
              orientAngl = orientAngl + (((velo * config.sampling_period)/config.vehicle_length)*math.sin(s_angle));
              x =  x + ((config.vehicle_length / math.tan(s_angle)) * (math.sin(orientAngl) - math.sin(orientAngl_lastStep)));
              y =  y - ((config.vehicle_length / math.tan(s_angle)) * (math.cos(orientAngl) - math.cos(orientAngl_lastStep)));
              #print('x,y in else clause:',x,y);
          
          cond=longitudinal_condition(vehicle.get_location().x,x,vehicle.get_location().y,y,vehicle.get_transform().rotation.yaw);
          print('longitudinal cond:', cond);
          config.T += config.sampling_period;
          print('T calc values',config.T);
        config.T -= config.sampling_period;

       #.....................calculation of phi_max ..................................................
       def calculate_max_steeringAng():
       
        x=vehicle.get_location().x;
        y=vehicle.get_location().y;
        orientAngl=vehicle.get_transform().rotation.yaw;
        ts=0;
        cond = False;
        #print('+++++++++phi_max calculation++++++++++++++++');
        #print('T_max first value:',config.T);
        while not cond:
          config.phi_max -= 0.0872665
          #print('config.phi_max value:',config.phi_max);
          #print('value of T_max in phi_max calculation:',config.T);
          for ts in numpy.arange(0,config.T,config.sampling_period):
            s_angle = steeringAngle(ts);
            velo = velocity(ts);
            if(s_angle == 0):
               orientAngl_lastStep = orientAngl;
               orientAngl = orientAngl;
               #print('orientationAngl for the last step:',orientAngl_lastStep,'orientAngl for this step:',orientAngl)
               x = x + (velo * config.sampling_period * math.cos(orientAngl));
               y = y + (velo * config.sampling_period * math.sin(orientAngl));
            else:
              orientAngl_lastStep = orientAngl;
              orientAngl = orientAngl + (((velo * config.sampling_period)/config.vehicle_length)*math.sin(s_angle));
              x =  x + ((config.vehicle_length / math.tan(s_angle)) * (math.sin(orientAngl) - math.sin(orientAngl_lastStep)));
              y =  y - ((config.vehicle_length / math.tan(s_angle)) * (math.cos(orientAngl) - math.cos(orientAngl_lastStep)));
          cond=lateral_condition(vehicle.get_location().x,x,vehicle.get_location().y,y,vehicle.get_transform().rotation.yaw);
          print('max steeringAngle from calculation:',config.phi_max);
          

       #............limitation for calculating time.........................
       def longitudinal_condition(x0,xT,y0,yT,orientAngl):
        x = math.fabs(((xT-x0)*math.cos(orientAngl))+((yT-y0)*math.sin(orientAngl)));
        print('value of lon calc:', x);
        cond = x < config.parkingLength;
        print('longitudinal condition result',cond);
        return cond;
       #.............condition to calculate phi_max............................
       def lateral_condition(x0,xT,y0,yT,orientAngl):
        x = math.fabs(((x0-xT)*math.sin(orientAngl))+((yT-y0)*math.cos(orientAngl)));
        print('value of lat calc:', x);
        cond = x < config.parkingWidth;
        #print('lateral condition result:',cond);
        return cond;


#********************non-player vehicles and sensors************************************

       
       # first non-player vehicle
       
       if vehicle1 is not None:
          coordinate1 = vehicle1.get_transform()
          coordinate1.location.z += 2.0
          coordinate1.rotation.roll = 0.0
          coordinate1.rotation.yaw = 0.0
          for actor in actor_list:
              if actor is not None:
                 actor.destroy()
          
         # staticPoint1 = carla.Transform(carla.Location(200, 303), carla.Rotation(yaw=180))
          spawn_points = ourMap.get_spawn_points()
          spawn_point = spawn_points[30]
          spawn_point.location.y +=1.0
          spawn_point.location.x -=12.0
          vehicle1 =world.spawn_actor(non_blueprint2, spawn_point)
          print('location of static-vehicle1: ')
          print(vehicle1.get_transform().location.x, vehicle1.get_transform().location.y)
          actor_list.append(vehicle1)

       while vehicle1 is None:
              #staticPoint1 = carla.Transform(carla.Location(200, 303), carla.Rotation(yaw=180))
              spawn_points = ourMap.get_spawn_points()
              spawn_point = spawn_points[30]
              spawn_point.location.y +=1.0
              spawn_point.location.x -=12.0
              vehicle1 =world.spawn_actor(non_blueprint2, spawn_point)
              print('location of static-vehicle1: ')
              print(vehicle1.get_transform().location.x, vehicle1.get_transform().location.y)
              actor_list.append(vehicle1)




       # 2nd non-player vehicle

       if vehicle2 is not None:
          coordinate2 = vehicle1.get_transform()
          coordinate2.location.z += 2.0
          coordinate2.rotation.roll = 0.0
          coordinate2.rotation.yaw = 0.0
          for actor in actor_list:
              if actor is not None:
                 actor.destroy()
          
          spawn_points2 = ourMap.get_spawn_points()
          spawn_point2 = spawn_points2[30]
          spawn_point2.location.y +=1.2
          spawn_point2.location.x -=20.0
          vehicle2 =world.spawn_actor(non_blueprint, spawn_point2)
          #print('location of static-vehicle2: ')
          #print(vehicle2.get_transform().location.x, vehicle2.get_transform().location.y)
          actor_list.append(vehicle2)

       while vehicle2 is None:
              spawn_points2 = ourMap.get_spawn_points()
              spawn_point2 = spawn_points2[30]
              spawn_point2.location.y +=1.2
              spawn_point2.location.x -=20.0
              vehicle2 =world.spawn_actor(non_blueprint, spawn_point2)
           #   print('location of static-vehicle2: ')
            #  print(vehicle2.get_transform().location.x, vehicle2.get_transform().location.y)
              actor_list.append(vehicle2)


       

 # 4rd non-player vehicle

       if vhcl4 is not None:
          coordinate4 = vhcl4.get_transform()
          coordinate4.location.z += 2.0
          coordinate4.rotation.roll = 0.0
          coordinate4.rotation.yaw = 0.0
          for actor in actor_list:
              if actor is not None:
                 actor.destroy()
          
          spawn_points = ourMap.get_spawn_points()
          spawn_point = spawn_points[30]
          spawn_point.location.y +=1.0
          spawn_point.location.x +=2.0
          vhcl4 =world.spawn_actor(non_blueprint4, spawn_point)
          actor_list.append(vhcl4)

       while vhcl4 is None:
              #staticPoint1 = carla.Transform(carla.Location(200, 303), carla.Rotation(yaw=180))
              spawn_points = ourMap.get_spawn_points()
              spawn_point = spawn_points[30]
              spawn_point.location.y +=1.0
              spawn_point.location.x +=2.0
              vhcl4=world.spawn_actor(non_blueprint4, spawn_point)
          
              actor_list.append(vhcl4)


#5th nonplayer vehicle
       if vhcl5 is not None:
          coordinate5 = vhcl5.get_transform()
          coordinate5.location.z += 2.0
          coordinate5.rotation.roll = 0.0
          coordinate5.rotation.yaw = 0.0
          for actor in actor_list:
              if actor is not None:
                 actor.destroy()
          
          spawn_points = ourMap.get_spawn_points()
          spawn_point = spawn_points[30]
          spawn_point.location.y +=1.0
          spawn_point.location.x +=10.0
          vhcl5 =world.spawn_actor(non_blueprint5, spawn_point)
          actor_list.append(vhcl5)

       while vhcl5 is None:
              spawn_points = ourMap.get_spawn_points()
              spawn_point = spawn_points[30]
              spawn_point.location.y +=1.0
              spawn_point.location.x +=10.0
              vhcl5=world.spawn_actor(non_blueprint5, spawn_point)
          
              actor_list.append(vhcl5)


       #-----------------spawn main vehicle---------------------------------
       if vehicle is not None:
            spawn_point = vehicle.get_transform()
            spawn_point.location.z += 2.5
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.yaw = 0.0
            for actor in actor_list:
                if actor is not None:
                    actor.destroy()
            spawn_points = ourMap.get_spawn_points()
            spawn_point = spawn_points[30];
            spawn_point.location.x += 2;
            spawn_point.location.y -= 1.5;
            vehicle = world.spawn_actor(blueprint, spawn_point)       
            actor_list.append(vehicle)

          

       while vehicle is None:
             spawn_points = ourMap.get_spawn_points()
             spawn_point = spawn_points[30];
             spawn_point.location.x += 2;
             spawn_point.location.y -= 1.5;
             vehicle = world.spawn_actor(blueprint, spawn_point)
             actor_list.append(vehicle)
     
            

       #========inja bayad check konam ke age car be yek noghteie resid az halghe biad biroon======get_location ghablo bad loop chek beshe*  

       


       time.sleep(3);
       calculate_maneuverTime();
       calculate_max_steeringAng();
       time.sleep(3);
       print('time duration:',config.T,'max steeringAngle:',config.phi_max);
       print('maneuver time.2')
       for t in numpy.arange(0,config.T,config.sampling_period):
            parking(t);
            time.sleep(0.05);
          
        


       '''while True:
        t=0;
        while t < config.T:
         parking(t);
         t += config.sampling_period;
         time.sleep(0.5)'''
         #print('maneuver time:',t);
       #=======after loop car be vaziate parked(sabet) bereseh=====apply_control meghdare steer ra 0 kone*
      


       


       while True:
        time.sleep(10)

    finally:
      print('\ndestroying %d actors' % len(actor_list))
      for actor in actor_list:
        if actor is not None:
          actor.destroy()

      for sen in sensors:
        if sen is not None:
          sen.destroy();




if __name__ == '__main__':
  try:
    main();
  except KeyboardInterrupt:
    pass;
  finally:
    print('\ndone.');
