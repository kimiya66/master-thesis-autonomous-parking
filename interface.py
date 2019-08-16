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
import matlab.engine
import parkingTest
from pymatbridge import Matlab

def decision(data):
	image=data.save_to_disk('/home/user1/Downloads/Carla9.5/PythonAPI/my projects/camera_result/%06d.jpg' % data.frame_number);
	print('this is the path to send to matlab: ',image); 
	mlab = Matlab()
	mlab.start()
	overlap = mlab.run_func('parking.m', {'img': image})

	return overlap




     
