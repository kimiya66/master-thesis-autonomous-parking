#!/usr/bin/python


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
import parkingTest
import numpy


def parking(t,vehicle):
       
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
        if velo < abs(v):
          print('throttle:',throttle,'control.throttle:',control.throttle);
          throttle = throttle + 0.5;
        else:
          brake = brake + 0.5;
          print('brake value:',control.brake);

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
def calculate_maneuverTime(vehicle):
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
def calculate_max_steeringAng(vehicle):     
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