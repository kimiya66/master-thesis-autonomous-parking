#!/usr/bin/python


# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import os
import sys
import glob
import matlab
import autoParking
from pymatbridge import Matlab

def decision(data,mlab):
	image = data.save_to_disk('./camera_result/%06d.jpg' % data.frame_number);
	print('sending camera result to matlab: ',image);
	overlap = mlab.run_func('parking.m', {'img': image});
	return overlap;

