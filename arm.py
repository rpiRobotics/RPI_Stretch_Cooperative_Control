'''
force compliance: arm
motion compliance: N/A
desired force = 20N along arm direction
desired velocity = 0m/s
'''

from RobotRaconteur.Client import *     #import RR client library
import sys, time, traceback
import numpy as np
from scipy.signal import filtfilt
from scipy import stats
import matplotlib.pyplot as plt
import scipy
import PID

#url of each robot
url1='rr+tcp://192.168.1.64:23232/?service=stretch'
url2='rr+tcp://192.168.1.28:23232/?service=stretch'

#Startup, connect, and pull out the arm/lift/eoa from the objref in robot obj  
robot1=RRN.ConnectService(url1)
robot2=RRN.ConnectService(url2)

# create robot 1 instance
base1 = robot1.get_base()
lift1=robot1.get_lift()
arm1=robot1.get_arm()
end_of_arm1=robot1.get_end_of_arm()

# create robot 2 instance
base2 = robot2.get_base()
lift2=robot2.get_lift()
arm2=robot2.get_arm()
end_of_arm2=robot2.get_end_of_arm()

# Connect to robot1 status RR wire
arm1_status = arm1.status_rr.Connect()
lift1_status=lift1.status_rr.Connect()
base1_status=base1.status_rr.Connect()

# Connect to robot2 status RR wire
arm2_status = arm2.status_rr.Connect()
lift2_status=lift2.status_rr.Connect()
base2_status=base2.status_rr.Connect()

# start the robots' motors
lift1.move_to(0.383) #Reach all the way out
arm1.move_to(0.05)
base1.translate_by(0.0)
lift2.move_to(0.4) #Reach all the way out
arm2.move_to(0.05)
base2.translate_by(0.0)
robot1.push_command()
robot2.push_command()
time.sleep(1)
now=time.time()

def bandpassfilter(signal):
	fs = 25.0
	lowcut = 2
	#highcut = 50.0

	nyq = 0.5*fs
	low = lowcut / nyq
	#high = highcut / nyq

	order = 6
	b,a = scipy.signal.butter(order, low, btype='low', analog=False)
	y = scipy.signal.filtfilt(b,a,signal, axis=0)

	return y

# some parameters
f1_d = 20
f2_d = 20
v1_d = 0
v2_d = 0
#K1_fwd = 0.00037
#K2_fwd = 0.00035

K1_fwd = 0.00025
K2_fwd = 0.000157
K1_bwd = 0.00005
K2_bwd = 0.0002

#feed_forward_1 = 1.05
#feed_forward_2 = -0.5
feed_forward_1 = 0.3
feed_forward_2 = 0


time.sleep(2)
filter_flag = 0
f1_record = []
f2_record = []
x1_dot_record = []
x2_dot_record = []

# parameters for PID controller
P = 0.0

I_1 = 0.000021
D_1 = 0.00001



I_2 = 0.000036
D_2 = 0.000036

#I_2 = 0.00001
#D_2 = 0.00001

pid1 = PID.PID(P, I_1, D_1)
pid2 = PID.PID(P, I_2, D_2)
pid1.SetPoint = f1_d
pid2.SetPoint = f2_d
pid1.setSampleTime(0.1)
pid2.setSampleTime(0.1)


# discard the first few noisy readings
for i in range(30):
	discard1 = lift1_status.InValue['force']
	discard2 = lift2_status.InValue['force']
	discard3 = arm1_status.InValue['force']
	discard4 = arm2_status.InValue['force']
	time.sleep(0.05) # the motor sampling frequency is 25 Hz

x1_dot = 0.0
x2_dot = 0.0
f1_reading = 0.0
f2_reading = 0.0
global_count = 0
while True:
	try:
		global_count += 1
		now=time.time()

		if global_count > 20:
			K2_fwd = 0.00018
			feed_forward_2 = -1

		# collect 25 data points. When the num is reached, remove the oldest point and add the newest point
		if filter_flag == 0:
			count = 0
			arm1_force = []
			arm2_force = []
			while count < 25:
				arm1_force.append(arm1_status.InValue['force'] + feed_forward_1)  # don't know why, but it seems add extra 1.55 to 1027 and let it have better performance
																		# feedforward			
				arm2_force.append(arm2_status.InValue['force'] + feed_forward_2) 
				count += 1
				time.sleep(0.06) # the motor sampling frequency is 25 Hz
		else:
			arm1_force.pop(0)
			arm2_force.pop(0)
			count_ = 0

			# force reading is very noisy when the arm is moving
			# this while loop is created to discard the first few readings
			prev_value=arm1_status.InValue['force']
			count_=0
			while True:
				if count_==2:
					break
				if prev_value == arm1_status.InValue['force']:
					continue
				else:
					count_+=1
					prev_value=arm1_status.InValue['force']


			f1_reading = arm1_status.InValue['force'] + feed_forward_1
			f2_reading = arm2_status.InValue['force'] + feed_forward_2
			arm1_force.append(f1_reading)
			arm2_force.append(f2_reading)

		filter_flag = 1
		arm1_force_filtered = np.array(bandpassfilter(arm1_force))
		arm2_force_filtered = np.array(bandpassfilter(arm2_force))
		arm1_force_mean = np.mean(arm1_force_filtered)
		arm2_force_mean = np.mean(arm2_force_filtered)

		f1 = round(arm1_force_mean,8)
		f2 = round(arm2_force_mean,5)
		diff_f1 = f1_d - f1
		diff_f2 = f2_d - f2 

		if abs(diff_f1) < 1.0:
			diff_f1 = 0

		if abs(diff_f2) < 1.0:
			diff_f2 = 0

		f1_record.append(f1)
		f2_record.append(f2)

		pid1.update(f1)
		pid2.update(f2)
		offset1 = pid1.output
		offset2 = pid2.output

		# various gains for forward and backward motion
		if diff_f1 > 0:
			x1_dot = K1_fwd * diff_f1 + v1_d 
		elif diff_f1 == 0:
			x1_dot = 0
		else:		
			x1_dot = K1_bwd * diff_f1 + v1_d 
		
		if diff_f2 > 0:
			x2_dot = K2_fwd * diff_f2 + v2_d + offset2
		elif diff_f2 == 0:
			x2_dot = 0
		else:
			x2_dot = K2_bwd * diff_f2 + v2_d + offset2

		
		x1_dot_record.append(x1_dot)
		x2_dot_record.append(x2_dot)

		arm1.move_by(x1_dot)
		arm2.move_by(x2_dot)
		lift1.move_to(0.383) # maintain the pose of the lift
		lift2.move_to(0.4) # maintain the pose of the lift
		base1.translate_by(0.0) # maintain the pose of the base
		base2.translate_by(0.0) # maintain the pose of the base
		base1.rotate_by(0.0)
		base2.rotate_by(0.0)
		robot1.push_command()
		robot2.push_command()
		print(time.time()-now)

	except:
		traceback.print_exc()
		break

time.sleep(0.5)
lift1.move_to(0.3)
lift2.move_to(0.3)
arm1.move_to(0.0)
arm2.move_to(0.0)
robot1.push_command( )
robot2.push_command( )
print ('Retracting...')


# data processing
n_f = np.linspace(0,len(f1_record),len(f1_record))
f1_record = np.array(f1_record)
f2_record = np.array(f2_record)
fig1 = plt.figure()
ax1 = fig1.add_subplot(2,1,1)
ax2 = fig1.add_subplot(2,1,2)
ax1.plot(n_f, f1_record,label='1027')
ax1.set_xlabel('frame')
ax1.set_ylabel('force (N)')
ax2.plot(n_f, f2_record,label='1028')
ax2.set_xlabel('frame')
ax2.set_ylabel('force (N)')
ax1.legend()
ax2.legend()
ax1.set_ylim([0,40])
ax2.set_ylim([0,40])

n_x = np.linspace(0,len(x1_dot_record),len(x1_dot_record))
x1_dot_record = np.array(x1_dot_record)
x2_dot_record = np.array(x2_dot_record)
fig2 = plt.figure()
ax3 = fig2.add_subplot(2,1,1)
ax4 = fig2.add_subplot(2,1,2)
ax3.plot(n_x, x1_dot_record,label='1027')
ax3.set_xlabel('frame')
ax3.set_ylabel('velocity (m/s)')
ax4.plot(n_x, x2_dot_record,label='1028')
ax4.set_xlabel('frame')
ax4.set_ylabel('velocity (m/s)')
ax3.legend()
ax4.legend()
ax3.set_ylim([-0.005,0.005])
ax4.set_ylim([-0.005,0.005])

plt.show()

np.savez('thesis_data_1.npy',f1_record, f2_record, x1_dot_record, x2_dot_record)