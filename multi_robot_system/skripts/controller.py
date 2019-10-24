#! /usr/bin/env python


import rospy
import numpy as np
import multi_robot_system.srv as srv
import rosbag

import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

#Interface Class for Controllers. Without any further implementations it just gets the input and passes it to output.
class RobotController():  
	 #Initialize a controler, with repsect to:
    #       node_name: The name of the node the controller is running at
    #       frequenzy: The publishing rate the controller is publishing at its main topic
	#		queue_size: Size of the output queue
	#		message_type: Objeckt of type Ros-message for definition of input and output message type of the controller
	#       topic: the name of its main topic
	def __init__(	self,
					node_name="my_robot_controller",
					frequenzy=10,
					queue_size=10,
					message_type=Twist,
					topic_name="ctr"):
		self.node_name=node_name
		rospy.init_node(self.node_name)

		self.queue_size=queue_size

		self.set_frequenzy(frequenzy)		
	

		self.set_input_message_type(message_type)	
		self.set_output_message_type(message_type)
		

		self.link_input_topic(topic_name+"_in")
		self.link_output_topic(topic_name+"_out")

		

	#callback procedure wich is called if a new input message occures	
	def income(self,msg):
		self.msg_in=msg


	#Initializes an rate object for ros timing
	def set_rate(self,rate):
		self.rate=rospy.Rate(self.frequenzy)
	
	#links the source of the controller to the given topic_name
	def link_input_topic(self,topic_name):
		if hasattr(self, 'sub'):
			self.sub.unregister()
		self.sub=rospy.Subscriber(topic_name,self.message_type_in,self.income)
	
	#links the sink of the controller to the given topic_name	
	def link_output_topic(self,topic_name):		
		if hasattr(self, 'pub'):
			self.pub.unregister()
		self.pub=rospy.Publisher(topic_name,self.message_type_out,queue_size=self.queue_size)
	
	#Setter for Control frequenzy
	def set_frequenzy(self,frequenzy):	
		self.frequenzy=frequenzy
		self.set_rate(self.frequenzy)
		self.time_step=np.float64(1/np.float64(self.frequenzy))

	#Reinitialzies the given message tye to input
	def set_input_message_type(self,MessageType):
		self.message_type_in=MessageType
		self.msg_in=MessageType()

	#Reinitialzies the given message tye to input
	def set_output_message_type(self,MessageType):
		self.message_type_out=MessageType		
		self.msg_out=MessageType()
	
	#is called every ros cycle
	def execute(self):
		self.msg_out=self.msg_in

	#is called just before ros cycle starts
	def startup(self):
		rospy.loginfo("Initialised pass trhough controller"+self.node_name+ " at "+str(self.frequenzy)+"Hz!")	

	#Setter for the flag of measureing data. If measure is true input and output topic are logged
	def set_measure(self):
		self.set_measure=True
		self.bag=ros
	#Execution prcodeure of the controller. Initialization of the node and the ros-scope runs here. 
	#Therfore publishing runs here and as RobotCorntroller is a parent class the input is passed through as output without doing anything
	def run(self):		
		self.startup()
		while not rospy.is_shutdown():
			self.execute()
			self.pub.publish(self.msg_out)
			self.rate.sleep()


#Class for a slave robot. It handles complete motion of the sleve with respect to a give input
class Slave(RobotController):
	#Constructor for the Slave class
	def __init__(	self,				
					node_name="my_slave",
					frequenzy=10,
					position=np.array(3,dtype=np.float64), #numpy vector x,y,z					
					omega=1.0,
					velocity=1.0,
					queue_size=10,
					topic_name="slave"):
		RobotController.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)		#calling parent constructor
		
		self.omega_max=omega
		self.velocity_max=velocity

		self.omega=0.0
		self.velocity=0.0

		self.phi=0.0

		self.position=position	
		self.distance=np.sqrt(self.position[0]**2+self.position[1]**2)
		self.prepare_roation=False
		self.prepare_translation=False
		
		self.rotate=False
		self.translate=False
		self.combined=False
		self.forward=True

		self.state="wait"

	#service for the preparation of a motion command
	def prepare_motion(self,req):
		if req.prepare_rotation and req.prepare_translation:
			self.combined=True
			self.state="prepare_translation"
		elif req.prepare_rotation:
			self.state="prepare_rotation"
		elif req.prepare_translation:
			self.state="prepare_translation"
		elif not req.prepare_rotation and not req.prepare_translation:
			self.state="wait"		
		else:	
			raise rospy.ServiceException(self.node_name+": Wrong reqest send!")
			return False
		return True
		

	
	def path_planning(self):
		#prepare the slave for a rotation command
		if self.state=="wait":
			pass
		
		elif self.state=="prepare_rotation":			
			self.confirm_preparation(self.node_name,False)	
			alpha=np.arctan2(self.position[1],self.position[0])		#angle between slave x-axis and master x-axis				
			if alpha >=0:				#first or second quadrant
				if alpha <=np.pi/2:			#first quadrant
					alpha=np.pi/2-alpha
					self.omega=-self.omega_max	#turn in negative dircetion
				elif alpha>np.pi/2: 		#second quadrant
					alpha-=np.pi/2
					self.omega=self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")
				self.forward=False			#move backward due rotation
			

			elif alpha<0:				#third or forth quadrant
				if alpha<=-np.pi/2:		#third quadrant
					alpha=-np.pi/2-alpha
					self.omega=-self.omega_max	#turn in negative dircetion
				elif alpha>-np.pi/2:		#fourth quadrant
					alpha=-alpha
					self.omega=self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")
				self.forward=True			#move forward due rotation

			else:							#error handling
					raise ValueError("Slave "+self.node_name+" angle incorrect!")	
				
					
			dist_deg=alpha-self.phi
			deg_step=self.omega_max*self.time_step
			if dist_deg<=deg_step:		
				self.phi+=dist_deg
				self.omega=self.omega*dist_deg/deg_step	
				self.confirm_preparation(self.node_name,True)
				self.state="rotate"					
			else:				
				self.phi+=deg_step

		elif self.state=="prepare_translation":	#Prepare the slave for a rotation around the master therefore it position vektor (relative to the master) and its orientation vektor have to be orthogonal to each other
			self.confirm_preparation(self.node_name,False)	
			#calculate the proper angle for get above descriped property
			alpha=np.arctan2(self.position[1],self.position[0])		#angle between slave x-axis and master x-axis				
			if alpha >=0:				#first or second quadrant
				if alpha <=np.pi/2:			#first quadrant
					self.omega=self.omega_max	#turn in negative dircetion
				elif alpha>np.pi/2: 		#second quadrant
					self.omega=-self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")			

			elif alpha<0:				#third or forth quadrant
				if alpha<=-np.pi/2:		#third quadrant
					self.omega=+self.omega_max	#turn in negative dircetion
				elif alpha>-np.pi/2:		#fourth quadrant
					self.omega=-self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")				

			else:							#error handling
					raise ValueError("Slave "+self.node_name+" angle incorrect!")	

			dist_deg=self.phi
			deg_step=self.omega_max*self.time_step
			if dist_deg<=deg_step:
				self.omega=self.omega*dist_deg/deg_step			
				self.phi-=dist_deg			
				self.confirm_preparation(self.node_name,True)
				if self.combined:
					self.state="combined"
				else:
					self.state="translate"							
			else:
				self.phi-=deg_step

		elif self.state=="rotate":					#Do the forced rotation around master				
			if self.forward:
				self.omega=self.msg_in.angular.z
				self.velocity=self.omega*self.distance
			else:
				self.omega=self.msg_in.angular.z
				self.velocity=-self.omega*self.distance

		elif self.state=="translate":				#Do the forced translation wih master			
			self.omega=0.0				
			self.velocity=self.msg_in.linear.x		
		
		elif self.state=="combined":				#Do the forced combination wih master
			self.omega=0.0
			self.velocity=0.0	#Not implemented. Kinamtik too difficult yet
			
		else:
			raise Exception(self.node_name+" in undefined state!")
		#Write out velocities


		self.msg_out.linear.x=self.velocity
		self.msg_out.angular.z=self.omega
	

	def startup(self):
		self.srv_prepare_motion=rospy.Service("srv_prepare_motion" ,srv.prepare_motion,self.prepare_motion)		#Prepare ros service for rekonfiguration of slave
		self.confirm_preparation=rospy.ServiceProxy("/srv_confirm",srv.confirm_preparation)						#prepare ros service for confirmation of slaves after reconfiguration
		rospy.loginfo("Initialized Slave"+self.node_name+ " with "+str(self.time_step)+" seconds time stamp!")	

		self.bc=tf.TransformBroadcaster()					#Broatcaster for transformation of robot frame basefootprint to a global base_footprint
		
	def execute(self):		
			self.bc.sendTransform(	(self.position[0],self.position[1],0),				#send the transform of robot frame to global base footprint (not correct yet)
								tf.transformations.quaternion_from_euler(0,0,0),
								rospy.Time.now(),
								self.node_name+"/base_link",
								"/map"	)		
			self.path_planning()							#calculate the output velocity wich is input for slaves
			
		
#Class for a master robot. It handles complete motion of the master while handeling the slaves
class Master(RobotController):
	#Service routine that lists the rsponse of slaves after reconfiguration
	def confirm_preparation(self,req):
		if req.name in self.slaves:
			self.slaves[req.name][1]=req.state		
			return True
		else:
			return False	
	

	def __init__(	self,				
					node_name="master",
					frequenzy=10,
					position=np.array(3,dtype=np.float64), #numpy vector x,y,z					
					omega=1.0,
					velocity=1.0,
					queue_size=10,
					topic_name="master"):
		RobotController.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
		self.slaves=dict()
		self.translation=False
		self.rotation=False
		self.prepare_rotation=False
		self.prepare_translation=False
		self.bc=tf.TransformBroadcaster()
		
		self.set_limits(velocity,omega)

		self.state="wait"

	#registers a slave for the master
	def add_slave(self,name):
		self.slaves[name]=list()

	#sets motion limits of the master
	def set_limits(self,v_max,omega_max):
		self.velocity_max=v_max
		self.omega_max=omega_max

	#calculates the output of the master from given input
	def path_planning(self):		
		if self.state=="wait":
			if not (-0.01<self.msg_in.angular.z<0.01) and not (-0.01<self.msg_in.linear.x<0.01): #check if combination is nessesarry
				self.state="prepare_combination"
			elif not (-0.01<self.msg_in.angular.z<0.01):			#Check if rotation is nessesarry
				self.state="prepare_rotation"
			elif not (-0.01<self.msg_in.linear.x<0.01):			#Check if rotation is nessesarry
				self.state="prepare_translation"
			else:
				self.sate="wait"

		elif self.state=="prepare_combination":				#Prepares the slaves for a combinational motion
			for slave in self.slaves:
				self.slaves[slave][0](1,1)
				self.slaves[slave][1]=False
			self.state="wait_response"
			self.combination=True

		elif self.state=="prepare_rotation":			#Prepares the slaves for a rotation
			for slave in self.slaves:
				self.slaves[slave][0](1,0)
				self.slaves[slave][1]=False
			self.state="wait_response"
			self.rotation=True
			
		
		elif self.state=="prepare_translation":			#prepares the slaves for a translation
			for slave in self.slaves:
				self.slaves[slave][0](0,1)
				self.slaves[slave][1]=False
			self.state="wait_response"
			self.translation=True
			
	
		elif self.state=="wait_response":				#waits for finishig preparations of the slaves
			motion=True
			for slave in self.slaves:
				motion*=self.slaves[slave][1]
			if motion:
				for slave in self.slaves:
					self.slaves[slave][1]=False
				if self.translation:
					self.state="translate"
				elif self.rotation:
					self.state="rotate"
				elif self.combination:
					self.state="combined"
				self.rotation=False
				self.translation=False
				self.combination=False
		
		elif self.state=="rotate":																	#Do a rotation with limitation of angular velocity
			if np.abs(self.msg_in.angular.z)>self.omega_max:
				self.msg_out.angular.z=np.sign(self.msg_in.angular.z)*self.omega_max
			else:		
				self.msg_out.angular.z=self.msg_in.angular.z
			if not (-0.01<self.msg_in.angular.z<0.01) and not (-0.01<self.msg_in.linear.x<0.01): 	#check if combination is nessesarry
				self.state="prepare_combination"
			elif not (-0.01<self.msg_in.angular.z<0.01):											#Check if rotation is nessesarry
				self.state="rotate"
			elif not (-0.01<self.msg_in.linear.x<0.01):												#Check if translation is nessesarry
				self.state="prepare_translation"
			
		
		elif self.state=="translate":																#Do a translation with limitation of angular velocity
			if np.abs(self.msg_in.linear.x)>self.velocity_max:
				self.msg_out.linear.x=np.sign(self.msg_in.linear.x)*self.velocity_max
			self.msg_out.linear.x=self.msg_in.linear.x		
				
			if not (-0.01<self.msg_in.angular.z<0.01) and not (-0.01<self.msg_in.linear.x<0.01): 	#check if combination is nessesarry
				self.state="prepare_combination"
			elif not (-0.01<self.msg_in.angular.z<0.01):											#Check if rotation is nessesarry
				self.state="prepare_rotation"
			elif not (-0.01<self.msg_in.linear.x<0.01):												#Check if translation is nessesarry
				self.state="translate"
		
			
		elif self.state=="combined":
			self.msg_out=self.message_type_out()
			# self.msg_out=self.msg_in
			if not (-0.01<self.msg_in.angular.z<0.01) and not (-0.01<self.msg_in.linear.x<0.01): 	#check if combination is nessesarry
				self.state="combined"
			elif not (-0.01<self.msg_in.angular.z<0.01):											#Check if rotation is nessesarry
				self.state="prepare_rotation"
			elif not (-0.01<self.msg_in.linear.x<0.01):												#Check if translation is nessesarry
				self.state="prepare_translation"
		
		
		else:
			raise Exception("Master is in undefined state!")


	def execute(self):
		self.bc.sendTransform(	(0,0,0),
							tf.transformations.quaternion_from_euler(0,0,0),
							rospy.Time.now(),
							self.node_name+"/base_link",
							"/map"	)		
		self.path_planning()
	

	def startup(self):	
		for key in self.slaves:
			try:
				self.slaves[key].append(rospy.ServiceProxy("/"+key+"/"+"srv_prepare_motion",srv.prepare_motion))
				self.slaves[key].append(False)
			except:
				raise Exception("problem with slave-key: "+key)
				rospy.logwarn("Problems due setup of slave: "+key)
		self.srv_confirm_preparation=rospy.Service("/srv_confirm",srv.confirm_preparation,self.confirm_preparation)
		rospy.loginfo("Initialized Master: "+self.node_name+ " with "+str(self.time_step)+" seconds time stamp!")
		for key in self.slaves:
			rospy.loginfo("Slave: "+key)	
		


		

