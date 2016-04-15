#!/usr/bin/env python

from std_msgs.msg import Bool
from std_msgs.msg import String
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.srv import requestTexture
from read_config import read_config
import rospy, math, json

HOT_TEMP = 40
ROOM_TEMP = 25
COLD_TEMP = 20

pipe_map = None
json_data = None
variance = None
texture_data = String()
temperature_data = temperatureMessage()
res_pipe_map = None

def update_temp_beliefs():
	global res_pipe_map, texture_data
	
	num_rows = len(pipe_map)
	num_cols = len(pipe_map[0])
	prob_x = 1.0/(num_rows * num_cols)
	res_pipe_map= [[prob_x] * num_cols] * num_rows
	
	for idx_r, row in enumerate(pipe_map):
		for idx_c, element in enumerate(row):
			prob_T_given_Xi = get_gaussian(temperature_data.temperature,
				element)
			prob_res = prob_T_given_Xi * res_pipe_map[idx_r][idx_c]
			res_pipe_map[idx_r][idx_c] = prob_res
	
	total_sum = sum(map(sum, res_pipe_map))
	res_pipe_map = map(lambda row: map(lambda x : x/total_sum,
		row),res_pipe_map)

	
	rospy.wait_for_service("requestTexture")
	texture_req = rospy.ServiceProxy("requestTexture", requestTexture)
	texture_data = texture_req()
	update_tex_beliefs()
		
def update_tex_beliefs():
	global res_pipe_map
	
	txt_map = json_data["texture_map"]
	prob_corr = json_data["prob_tex_correct"]
	for idx_r, row in enumerate(txt_map):
		for idx_c, col in enumerate(row):
			prob_txt_data = 0
			if col != texture_data.data:
				prob_txt_data = 1 - prob_corr
			else:
				prob_txt_data = prob_corr

			prob_res = prob_txt_data * res_pipe_map[idx_r][idx_c]
			res_pipe_map[idx_r][idx_c] = prob_res

def get_gaussian(temperature, base):
	error = temperature - base
	var = (1.0/math.sqrt(2 * math.pi) * variance)
	power_e = math.e ** (-((error**2)/(2*(variance**2))))
	return var * power_e

def publish_all_data():	
	#publish_temperature(temperature_data.temperature)
	publish_texture(texture_data)

def publish_temperature(temp_data):
	temp_pub = rospy.Publisher("/results/temperature_data",
		temperatureMessage, queue_size=10)
	temp_pub.publish(temp_data)

def publish_texture(txt_data):
	txt_pub = rospy.Publisher("/results/texture_data", String,
		queue_size=10)
	txt_pub.publish(txt_data)

def temperature_callback(data):
	global temperature_data

	temperature_data = data
	update_temp_beliefs()
	
def activate_publish():
	temp_act_pub = rospy.Publisher("/temp_sensor/activation",
		Bool, queue_size=10)
	rospy.sleep(1)
	temp_act_pub.publish(Bool(data=True))

def fetch_all_data():

	#get temperature data from the topic
	rospy.Subscriber("/temp_sensor/data", temperatureMessage,
		temperature_callback)
	
	#rospy.sleep(2)

def load_config_file():
	global pipe_map, json_data, variance 

	json_data = read_config()

	pipe_map = json_data["pipe_map"]
	variance = json_data["temp_noise_std_dev"]

	for idx_r in range(len(pipe_map)):
		for idx_c in range(len(pipe_map[0])):
			element = pipe_map[idx_r][idx_c]			
			if element  == "C":
				pipe_map[idx_r][idx_c] = COLD_TEMP
			elif element == "-":
				pipe_map[idx_r][idx_c] = ROOM_TEMP
			elif element == "H":
				pipe_map[idx_r][idx_c] = HOT_TEMP

def startRobot():
	rospy.init_node("robot_node", anonymous=True)
	activate_publish()
	load_config_file()
	fetch_all_data()
	publish_all_data()

if __name__ == '__main__':
	try:
		startRobot()
	except rospy.ROSInterruptException:
		pass
	
	while not rospy.is_shutdown():
		pass



