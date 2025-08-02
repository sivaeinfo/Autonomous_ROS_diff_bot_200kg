#!/usr/bin/env python
import rospy
import roslib
import math 
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt8
class TwistToMotors(object):
	def __init__(self, node_name='twist_to_motors'):
    		rospy.init_node(node_name)
    		nodename = rospy.get_name()
    		rospy.loginfo("%s started" % nodename)
    		self.w = rospy.get_param("~base_width", 0.38)
    		self.pub_motor_vel = rospy.Publisher('modbus/regs_write', UInt16MultiArray, queue_size = 10)
    		self.pub_disp = rospy.Publisher('disp_spd', Float32MultiArray, queue_size = 10)
    		#self.pub_motor_switch = rospy.Publisher('modbus/coils_write', ByteMultiArray, queue_size = 10)
    		rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    		rospy.Subscriber("button_action_CMD", UInt8, self.calc_pub_pulse)
    		rospy.Subscriber("slider_CMD", Float32MultiArray, self.whl_bot_spdSLD_limit)
    		self.rate = rospy.get_param("~rate", 20)
    		self.racdc = rospy.Rate(1)
    		self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
    		self.jog_timeout_ticks = rospy.get_param("~jog_timeout_ticks", 2)
    		#self.amr_switch = ByteMultiArray()
    		self.lr_wh_vel = UInt16MultiArray()
    		self.lr_wh_vel.layout.dim = []
    		self.lr_wh_vel.layout.data_offset = 0
    		self.disp_vels = Float32MultiArray()
    		#self.disp_vels.layout.data_offset = 0
    		self.disp_vels.layout.dim = []
    		self.disp_vels.layout.data_offset = 0
    		#self.amr_switch.layout.data_offset = False
    		self.ticks_since_jog_target = 0
    		self.rad_wh = 0.16
    		self.auto = False
    		self.manual = False
    		self.LV = 0.0
    		self.AV = 0.0
    		self.dx = self.dr = 0.0
    		self.AV_SLD = self.LV_SLD = 0.0
    		self.rgtwhSLD = self.lftwhSLD = 0.0
    		self.rgwh_rpm = self.lfwh_rpm = 0.0
    		self.wh = False    		
    		self.at_lin_acc = self.at_lin_dec = self.at_lin_dec = self.at_ang_dec = 0.01
    		self.j_lin_acc = self.j_lin_dec = self.j_ang_acc = self.j_ang_dec = 0.01
    		self.j_cur_lrpm =  self.j_cur_rrpm = 0.0 
    		self.j_tar_lrpm = self.j_tar_rrpm = 0.0
    		self.cur_lin_vel = self.cur_ang_vel = 0.0
    		self.j_cur_lin_vel =  self.j_cur_ang_vel = 0.0 
    		    		
	def spin(self):
   		r = rospy.Rate(self.rate)
   		idle = rospy.Rate(10) #20ms
   		then = rospy.Time.now()
   		self.ticks_since_target = self.timeout_ticks
   		self.ticks_since_jog_target = self.jog_timeout_ticks
   		while not rospy.is_shutdown():
	   		if self.auto == True and self.ticks_since_target < self.timeout_ticks:
	   			self.spinOnce()
	   			#print("Auto Mode of operation ***********") 			
	   		if self.manual == True and self.ticks_since_jog_target < self.jog_timeout_ticks:
	   			#rospy.loginfo("Manual Mode of operation BEGIN***********%d", self.ticks_since_jog_target)
	   			self.jogspinonce()
	   			#print("Manual Mode of operation FINISH***********")
	   		else:
	   			self.lr_wh_vel.data = [0, 0, 1, 0]
	   			if self.ticks_since_jog_target < self.jog_timeout_ticks:
	   				#print("STOP Mode of operation ***********")
	   				self.ticks_since_jog_target = self.ticks_since_jog_target + 1
	   		idle.sleep()
	   	#print("stooooooooooooooooooooooopped")
   				
	def spinOnce(self):		
		if self.wh == False and self.auto == True and self.manual == False:		
	   		if self.dx > 0.5:
	   			self.dx = 0.5
	   		elif self.dx < -0.5:
	   			self.dx = -0.5
	   		if self.dr > 0.5:
	   			self.dr = 0.5
	   		elif self.dr < -0.5:
	   			self.dr = -0.5
	   		tar_lin_vel = self.dx
	   		tar_ang_vel = self.dr	   			   	
	   		lvp = lvn = avp = avn = 0	
	   		fin_lin = fin_ang = False
	   		print("CURRENT Lin Vel - TARGET Vel")
	   		print(self.cur_lin_vel,tar_lin_vel)
	   		print("CURRENT Ang Vel - TARGET Vel")
	   		print(self.cur_ang_vel,tar_ang_vel)
	   		while not rospy.is_shutdown() and (fin_lin != True or fin_ang != True):
	   			if (self.at_lin_acc < (tar_lin_vel - self.cur_lin_vel)) and lvn != 1 and fin_lin != True:
	   				self.cur_lin_vel += self.at_lin_acc
	   				lvp = 1
	   			elif (self.at_lin_dec > (tar_lin_vel - self.cur_lin_vel)) and lvp != 1 and fin_lin != True:
	   				self.cur_lin_vel -= self.at_lin_dec
	   				lvn = 1
	   			else:
	   				self.cur_lin_vel = tar_lin_vel
	   				fin_lin = True
	   				lvp = 0
	   				lvn = 0
	   				
	   			if (self.at_ang_acc < (tar_ang_vel - self.cur_ang_vel)) and avn != 1 and fin_ang != True:
	   				self.cur_ang_vel += self.at_ang_acc
	   				avp = 0
	   			if (self.at_ang_acc > (tar_ang_vel - self.cur_ang_vel)) and lvp != 1 and fin_ang != True:
	   				self.cur_ang_vel -= self.at_ang_dec
	   				avn = 0
	   			else:
	   				self.cur_ang_vel = tar_ang_vel
	   				fin_ang = True
	   				avp = 0
	   				avn = 0		
	   		   				
	   				at_rgt_whl_vel = ((self.cur_lin_vel + ((cur_ang_vel * self.w )/ 2))/self.rad_wh)
	   				at_lft_whl_vel = ((self.cur_lin_vel - ((cur_ang_vel * self.w )/ 2))/self.rad_wh) 
	   				self.rgwh_rpm = at_rgt_whl_rpm = (((at_rgt_whl_vel/(2*math.pi))*60) * self.rgtwhSLD/255)
	   				self.lfwh_rpm = at_lft_whl_rpm = (((at_lft_whl_vel/(2*math.pi))*60) * self.lftwhSLD/255)
	   				self.disp_vels.data = [self.dx, self.dr, self.lfwh_rpm, self.rgwh_rpm]
	   				self.pub_disp.publish(self.disp_vels)
		   			at_rgt_whl_rps = int((at_rgt_whl_rpm/60)*10000)
		   			at_lft_whl_rps = int((at_lft_whl_rpm/60)*10000)
		   			#at_rgt_whl_hz  = ((at_rgt_whl_vel)/(2*math.pi))
		   			if at_lft_whl_rps < 0 and at_rgt_whl_rps >= 0:
			      			self.lr_wh_vel.data = [abs(at_lft_whl_rps), abs(at_rgt_whl_rps), 1, 0]
		   			elif at_rgt_whl_vel < 0 and at_lft_whl_rps >= 0:
			      			self.lr_wh_vel.data = [abs(at_lft_whl_rps), abs(at_rgt_whl_rps), 0, 1]
		   			elif at_rgt_whl_rps < 0 and at_lft_whl_rps < 0:
			      			self.lr_wh_vel.data = [abs(at_lft_whl_rps), abs(at_rgt_whl_rps), 1, 1]
		   			else:
			      			self.lr_wh_vel.data = [abs(at_lft_whl_rps), abs(at_rgt_whl_rps), 0, 0]		   		
		   			self.pub_motor_vel.publish(self.lr_wh_vel)		   		
		   		self.racdc.sleep()
		   		self.ticks_since_target = self.ticks_since_target + 1
		   	rospy.loginfo("Auto_lftwh_V = %f,rtwh_V = %f,dir1=%d,  dir2= %d", at_lft_whl_rpm, at_rgt_whl_rpm, self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])

	def twistCallback(self, msg):
   		#rospy.loginfo("-D- twistCallback: %s" % str(msg))   		
   		if self.auto == True:
   			self.dx = (msg.linear.x *(self.lnvelSLD/255)) 
   			self.dr = (msg.angular.z *(self.anvelSLD/255))
   			self.dy = msg.linear.y
   			self.ticks_since_target = 0
			
	def jogspinonce(self):
		if self.wh == False and self.manual == True and self.auto == False:
			if self.LV > 0.5:
				self.LV = 0.5
			if self.AV > 0.5:
				self.AV = 0.5
			if self.LV < -0.5:
				self.LV = 0.5
			if self.AV < -0.5:
				self.AV = -0.5
			j_tar_lin_vel = self.LV
			j_tar_ang_vel = self.AV			
			print("CURRENT Lin Vel - TARGET Lin Vel")
			print(self.j_cur_lin_vel , j_tar_lin_vel)
			print("CURRENT Ang Vel - TARGET Ang Vel")
			print(self.j_cur_ang_vel , j_tar_ang_vel)
			jfinlacc = jfinaacc = jfinldec = jfinadec = False
			jlvacc = javacc = javdec = jlvdec = 0
			count_true = 0			
			while not rospy.is_shutdown() and count_true < 2:
				if self.j_cur_lin_vel <= j_tar_lin_vel: 
					if (self.j_lin_acc < (j_tar_lin_vel - self.j_cur_lin_vel)) and jlvdec != 1: 
						self.j_cur_lin_vel += self.j_lin_acc
						print("Lin_vel Acc %f", self.j_cur_lin_vel)						
					else:
						self.j_cur_lin_vel = j_tar_lin_vel
						print("Lin_vel Acc %f", self.j_cur_lin_vel)
						print("Lin_Vel_Acc_Finished -----------------")
						jfinlacc = True
						jlvacc = 1
				if self.j_cur_ang_vel <= j_tar_ang_vel:
					if (self.j_ang_acc < (j_tar_ang_vel - self.j_cur_ang_vel)) and javdec != 1:
						self.j_cur_ang_vel += self.j_ang_acc
						print("Ang_vel Acc %f", self.j_cur_ang_vel)						
					else:
						self.j_cur_ang_vel = j_tar_ang_vel
						print("Ang_vel Acc %f", self.j_cur_ang_vel)
						print("Ang_Vel_Acc_ Finished -----------------")
						jfinaacc = True
						javacc = 1				
				if self.j_cur_lin_vel > j_tar_lin_vel: 
					if (self.j_lin_dec > (j_tar_lin_vel - self.j_cur_lin_vel)) and jlvacc != 1:
						self.j_cur_lin_vel -= self.j_lin_dec
						print("Lin_vel Dec %f", self.j_cur_lin_vel)						
					else:
						self.j_cur_lin_vel = j_tar_lin_vel
						print("Lin_vel Dec %f", self.j_cur_lin_vel)
						print("Lin_Vel_Dec _ Finished -----------------")
						jfinldec = True
						jlvdec = 1
				if self.j_cur_ang_vel > j_tar_ang_vel:
					if (self.j_ang_dec > (j_tar_ang_vel - self.j_cur_ang_vel)) and javacc != 1:
						self.j_cur_ang_vel -= self.j_ang_dec
						print("Ang_vel Dec %f", self.j_cur_ang_vel)					
					else:
						self.j_cur_ang_vel = j_tar_ang_vel
						print("Ang_vel Dec %f", self.j_cur_ang_vel)
						print("Ang_Vel_Dec _ Finished-----------------")
						jfinadec = True
						javdec = 1						
				x = [jfinlacc, jfinaacc, jfinldec, jfinadec] 
				count_true = sum(x)
				rgt_whl_vel = (self.j_cur_lin_vel + ((self.j_cur_ang_vel * (self.w / 2))))/self.rad_wh
				lft_whl_vel = (self.j_cur_lin_vel - ((self.j_cur_ang_vel * (self.w / 2))))/self.rad_wh			
				self.rgwh_rpm = rgt_whl_rpm = (((rgt_whl_vel/(2*math.pi))*60) * (self.rgtwhSLD/255))
				self.lfwh_rpm = lft_whl_rpm = (((lft_whl_vel/(2*math.pi))*60) * (self.lftwhSLD/255))
				#print("RPM ++++++++++++++++++++++++++++++++")
				#print(self.rgwh_rpm,self.lfwh_rpm)
				self.disp_vels.data = [j_tar_lin_vel, j_tar_ang_vel, self.lfwh_rpm, self.rgwh_rpm]
				self.pub_disp.publish(self.disp_vels)
				
				rgt_whl_rps = int((rgt_whl_rpm/60)*10000)
				lft_whl_rps = int((lft_whl_rpm/60)*10000)
				if lft_whl_rps < 0.0 and rgt_whl_rps >= 0.0:
					self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 1, 0]
					#print("-ve left")
					self.pub_motor_vel.publish(self.lr_wh_vel)
					#rospy.loginfo("-Ve lft Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
					self.ticks_since_jog_target = self.ticks_since_jog_target + 1
				if rgt_whl_rps < 0.0 and lft_whl_rps >= 0.0:
					self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 0, 1]
					#print("-ve right")
					self.pub_motor_vel.publish(self.lr_wh_vel)
					#rospy.loginfo("-ve rigt Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
					self.ticks_since_jog_target = self.ticks_since_jog_target + 1
				if rgt_whl_rps < 0 and lft_whl_rps < 0:
					self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 1, 1]
					#print("Both -ve")
					self.pub_motor_vel.publish(self.lr_wh_vel)
					#rospy.loginfo("both -ve Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
					self.ticks_since_jog_target = self.ticks_since_jog_target + 1
				if rgt_whl_rps >= 0 and lft_whl_rps >= 0:
					self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 0, 0]
					#print("Positve")
					self.pub_motor_vel.publish(self.lr_wh_vel)
					#rospy.loginfo("both +ve Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
					self.ticks_since_jog_target = self.ticks_since_jog_target + 1
				self.racdc.sleep()
				
				
		elif self.wh == True and self.manual == True and self.auto == False:
			self.rgwh_rpm = rgt_whl_rpm = (self.RW*10*(self.rgtwhSLD/255))
			self.lfwh_rpm = lft_whl_rpm = (self.LW*10*(self.lftwhSLD/255))
			print(lft_whl_rpm,rgt_whl_rpm)
			rgt_whl_rps = int((self.rgt_whl_rpm/60)*10000)
			lft_whl_rps = int((self.lft_whl_rpm/60)*10000)
			if lft_whl_rps < 0 and rgt_whl_rps >= 0:
				self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 1, 0]
				print("-ve left")
				self.pub_motor_vel.publish(self.lr_wh_vel)
				#rospy.loginfo("Wheel Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
				self.ticks_since_jog_target = self.ticks_since_jog_target + 1
			if rgt_whl_rps < 0 and lft_whl_rps >= 0.0:
				self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 0, 1]
				print("-ve right")
				self.pub_motor_vel.publish(self.lr_wh_vel)
				#rospy.loginfo("Wheel Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
				self.ticks_since_jog_target = self.ticks_since_jog_target + 1
			if rgt_whl_rps < 0 and lft_whl_rps < 0:
				self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 1, 1]
				print("Both -ve")
				self.pub_motor_vel.publish(self.lr_wh_vel)
				#rospy.loginfo("Wheel Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
				self.ticks_since_jog_target = self.ticks_since_jog_target + 1
			elif rgt_whl_rps >= 0 and lft_whl_rps >= 0:
				self.lr_wh_vel.data = [abs(lft_whl_rps), abs(rgt_whl_rps), 0, 0]
				print("Positve")
				self.pub_motor_vel.publish(self.lr_wh_vel)
				#rospy.loginfo("Wheel Manual lftwh vel = %d rtwh vel = %d   dir1 = %d  dir2 = %d", self.lr_wh_vel.data[0], self.lr_wh_vel.data[1], self.lr_wh_vel.data[2], self.lr_wh_vel.data[3])
				self.ticks_since_jog_target = self.ticks_since_jog_target + 1
				self.racdc.sleep()	
	
	def move_forward(self):
		self.LV = 0.3
		self.AV = 0.0
		self.wh = False
		self.ticks_since_jog_target = 0
			
	def forward_left(self):
		self.LV = 0.2
		self.AV = 0.3
		self.wh = False
		self.ticks_since_jog_target = 0
		
	def forward_right(self):
		self.LV = 0.2
		self.AV = -0.3
		self.wh = False
		self.ticks_since_jog_target = 0
		
	def left(self):
		self.LV = 0.0
		self.AV = 0.3
		self.wh = False
		self.ticks_since_jog_target = 0
		
	def right(self):
		self.LV = 0.0
		self.AV = -0.3
		self.wh = False
		self.ticks_since_jog_target = 0
		
	def backward(self):
		self.LV = -0.3
		self.AV = 0.0
		self.wh = False
		self.ticks_since_jog_target = 0
		
	def backward_right(self):
		self.LV = -0.2
		self.AV = 0.3
		self.wh = False
		self.ticks_since_jog_target = 0
		
	def backward_left(self):
		self.LV = -0.2
		self.AV = -0.3
		self.wh = False
		self.ticks_since_jog_target = 0

	def right_wh_forward(self):
		self.wh = True
		self.RW = 1
		self.LW = 0
		self.ticks_since_jog_target = 0
	
	def right_wh_backward(self):
		self.wh = True
		self.RW = -1
		self.LW = 0
		self.ticks_since_jog_target = 0
		
	def left_wh_forward(self):
		self.wh = True
		self.RW = 0
		self.LW = 1
		self.ticks_since_jog_target = 0
		
	def left_wh_backward(self):
		self.wh = True
		self.RW = 0
		self.LW = -1
		self.ticks_since_jog_target = 0
		
	def amr_stop(self):
		self.LV = 0.0
		self.AV = 0.0
		if self.manual == True:		
			self.wh = False
			self.ticks_since_jog_target = 0		
		if self.auto == True:
			self.ticks_since_target 
		
	def auto_mode(self):
		self.auto = True
		self.wh = False
		self.manual = False
		print("auto_mode")
	
	def manual_mode(self):
		self.manual = True
		self.auto = False		
		print("manual_mode")	
			
	def whl_bot_spdSLD_limit(self, wh_bt_SPD_lmt):
		self.lftwhSLD = wh_bt_SPD_lmt.data[0]
		self.rgtwhSLD = wh_bt_SPD_lmt.data[1]
		self.lnvelSLD = wh_bt_SPD_lmt.data[2]
		self.anvelSLD = wh_bt_SPD_lmt.data[3]
		self.j_ang_acc	= self.j_lin_acc = wh_bt_SPD_lmt.data[4]
		self.j_ang_dec	= self.j_lin_dec = wh_bt_SPD_lmt.data[5]	
		#print(self.lftwhSLD, self.rgtwhSLD, self.lnvelSLD, self.anvelSLD)		
		
	def calc_pub_pulse(self,message):		
		if message.data == 14:
			self.auto_mode()
			#rospy.loginfo("AUTO MODE OF OPER=%s", self.auto)
		elif message.data == 15:
			self.manual_mode()
			#rospy.loginfo("MANUAL MODE OF OPER=%s", self.manual)		
		elif message.data == 1 and self.manual == True:
			self.move_forward()
		elif message.data == 2 and self.manual == True:
			self.forward_left()
		elif message.data == 3 and self.manual == True:
			self.forward_right()
		elif message.data == 4 and self.manual == True:
			self.left()
		elif message.data == 5 and self.manual == True:
			self.right()
		elif message.data == 6 and self.manual == True:
			self.backward()
		elif message.data == 7 and self.manual == True:
			self.backward_right()
		elif message.data == 8 and self.manual == True:
			self.backward_left()
		elif message.data == 10 and self.manual == True:			
			self.right_wh_forward()
		elif message.data == 11 and self.manual == True:
			self.right_wh_backward()
		elif message.data == 12 and self.manual == True:
			self.left_wh_forward()
		elif message.data == 13 and self.manual == True:
			self.left_wh_backward()	
		elif message.data == 9 and self.manual == True:
			self.amr_stop()
		elif message.data == 9 and self.auto == True:
			self.amr_stop()
		else:
			if self.auto == True:
				print("AUTO MODE IS ON, NO MANUAL CONTROL")
			else:
				print("AMR NOT YET STARTED, RUN VIA AUTO/MANUAL")
		#rospy.loginfo("Received data %s AUTO = %s Manual = %s", message.data, str(self.auto), str(self.manual))
			
if __name__ == '__main__':
	twtomot = TwistToMotors(node_name='twist_to_motors')
	while not rospy.is_shutdown():	
		twtomot.spin()


