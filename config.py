 #parking motion values: 
T_star = 10; #empirical value for time in motion formula
T = T_star;  #duration of the parking motion 
sideOfParking = -1.0; #if we park to left side or right side of the parking
direction = -1.0; #if car moving forward or backward(-1)
vehicle_length = 4;    #...vehicle length should be calculated later (according to Gnss positions)...
speed_limit=30 #m/s
v_max = 0.3  #max value of velocity
#v_max=abs(vehicle.get_speed_limit());
phi_max = 0.67#initiate max value of steering angle
sampling_period=0.05; #sampling period for time and phi_max calculation--value of the current timestamp
L=1.765 #wheel base
parkingLength = 14
parkingWidth = 3
count=0; #number of non_player/parked cars detected
nonplayer=None;