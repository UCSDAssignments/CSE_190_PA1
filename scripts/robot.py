#!/usr/bin/env python

from std_msgs.msg import Bool
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.srv import requestTexture
import rospy

temperature_data = temperatureMessage()
texture_data = String()

def temperature_callback(data):
	temperature_data.data = data.data

	
def activate_publish():
	temp_act_pub = rospy.Publisher("/temp_sensor/activation",
		Bool, queue_size=10)
	temp_act_pub.publish(Bool(data=True))

def fetch_all_data():
	#get temperature data from the topic
	rospy.Subscriber("/temp_sensor/data", temperatureMessage,
		temperature_callback)
	
	rospy.wait_for_service("requestTexture")
	texture_req = rospy.ServiceProxy("requestTexture", requestTexture)
	texture_data.data = texture_req().data


 
def startRobot():
	rospy.init_node("robot_node", anonymous=True)
	activate_publish()
	fetch_all_data()

if __name__ == '__main__':
	try:
		startRobot()
	except rospy.ROSInterruptException:
		pass



