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
import autoParking
import numpy


def parking(t,vehicle):
  angle=steeringAngle(t);
  v=velocity(t);
  vehicle.apply_control(carla.VehicleControl(throttle=v, manual_gear_shift=True, gear=0,steer=angle,reverse=True));

      
#..............calculation of steeringAngl at each time of maneuver..............
def steeringAngle(t):

      
        result=config.phi_max * config.sideOfParking * _A(t);
        return result;

#..............calculation of velocity at each time of maneuver....................
def velocity(t):
        for type_error in _B(t):
          result = config.v_max * config.direction * type_error;
        return result;

#...............calculation of _A and _B functions to control phi and v...............
def _A(t):
        t_prime = (config.T - config.T_star)/2; 
        result = 0;     #output of the function
        if 0 <= t < t_prime:
          result = 1;
        elif t_prime <= t <= config.T-t_prime:
          result = math.cos((math.pi*(t-t_prime))/config.T_star)
        elif  config.T-t_prime < t <= config.T:
          result = -1;
        return result;

def _B(t):
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
            velo = velocity(ts);
            if(s_angle == 0):
               orientAngl_lastStep = orientAngl;
               orientAngl = orientAngl;
               x = x + (velo * config.sampling_period * math.cos(orientAngl));
               y = y + (velo * config.sampling_period * math.sin(orientAngl));
            else:
              orientAngl_lastStep = orientAngl;
              orientAngl = orientAngl + (((velo * config.sampling_period)/config.vehicle_length)*math.sin(s_angle));
              x =  x + ((config.vehicle_length / math.tan(s_angle)) * (math.sin(orientAngl) - math.sin(orientAngl_lastStep)));
              y =  y - ((config.vehicle_length / math.tan(s_angle)) * (math.cos(orientAngl) - math.cos(orientAngl_lastStep)));
          
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
        while not cond:
          config.phi_max -= 0.0872665
          for ts in numpy.arange(0,config.T,config.sampling_period):
            s_angle = steeringAngle(ts);
            velo = velocity(ts);
            if(s_angle == 0):
               orientAngl_lastStep = orientAngl;
               orientAngl = orientAngl;
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
        return cond;
