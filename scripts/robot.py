#!/usr/bin/env python

from std_msgs.msg import Bool
from std_msgs.msg import String
from cse_190_assi_1.msg import *
from cse_190_assi_1.srv import requestTexture
from cse_190_assi_1.srv import moveService
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
temp_act_pub = None
prob_move_correct = 0
prob_move_inc = 0
num_rows = 0
num_cols = 0

def update_temp_beliefs():
	global res_pipe_map, texture_data
	
	for idx_r, row in enumerate(pipe_map):
		for idx_c, element in enumerate(row):
			prob_T_given_Xi = get_gaussian(temperature_data.temperature,
				element)
			prob_res = prob_T_given_Xi * res_pipe_map[idx_r][idx_c]
			res_pipe_map[idx_r][idx_c] = prob_res
	normalize()

	rospy.wait_for_service("requestTexture")
	texture_req = rospy.ServiceProxy("requestTexture", requestTexture)
	texture_data = texture_req()
	update_tex_beliefs()
	update_move_beliefs()
	#publish_all_data()
	#create_output_files()
	deactivate_temp()
	rospy.sleep(1)
	rospy.signal_shutdown()
		
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
	
	normalize()

def update_move_beliefs():
	rospy.wait_for_service("moveService")
	moveSrvProxy = rospy.ServiceProxy("moveService", moveService)

	move_list = json_data["move_list"]
	temp_res_map = [[[]] * num_cols] * num_rows

	for move in move_list:
		moveSrvProxy(move)
		isLR = True if move[0] != 0 else False
		for idx_r in range(num_rows):
			for idx_c in range(num_cols):
				if isLR:
					prior_prob= res_pipe_map[idx_r][idx_c + move[0]]
					updated_belief = prior_prob * prob_move_correct
					temp_res_map[idx_r][idx_c+move[0]]= updated_belief
					
					#isRight = True if move[0] > 0 else False
					#direction_taken = "R" if isRight == True else "L"
					currPos = [idx_r, idx_c]
					updateRestOfBeliefs(currPos, direction_taken) 
				else:
					prior_prob = res_pipe_map[idx_r + move[1]][idx_c]
					updated_belief = prior_prob * prob_move_correct
					temp_res_map[idx_r + move[1]][idx_c]= updated_belief


				
def updateRestOfBeliefs(curr_pos, direction_taken, temp_res_map):
	#not finished
	directions = {"R", "L", "D", "U", "S"}
	for direct in directions:
		if direct != direction_taken:
			r = curr_pos[1]
			c = curr_pos[0]
			
			
				
			



def normalize():
	global res_pipe_map

	total_sum = sum(map(sum, res_pipe_map))
	res_pipe_map = map(lambda row: map(lambda x : x/total_sum,
		row),res_pipe_map)

def get_gaussian(temperature, base):
	error = temperature - base
	var = (1.0/math.sqrt(2 * math.pi) * variance)
	power_e = math.e ** (-((error**2)/(2*(variance**2))))
	return var * power_e

def create_output_files():
	out_file = rospy.Publisher("/map_node/sim_complete", Bool,
		queue_size=10)
	out_file.publish(Bool(data=True))

def publish_all_data():	
	publish_probabilities()
	#publish_temperature(temperature_data.temperature)
	publish_texture(texture_data)

def publish_probabilities():
	float_vector = RobotProbabilities()
	float_vector.data = reduce(lambda x,y: x+y, res_pipe_map)
	prob_pub = rospy.Publisher("/results/probabilities", RobotProbabilities,
		queue_size=10)
	prob_pub.publish(float_vector)

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
	
def activate_temp():
	global temp_act_pub
	temp_act_pub = rospy.Publisher("/temp_sensor/activation",
		Bool, queue_size=10)
	rospy.sleep(1)
	temp_act_pub.publish(Bool(data=True))

def deactivate_temp():
	temp_act_pub.publish(Bool(data=False))

def fetch_all_data():

	#get temperature data from the topic
	rospy.Subscriber("/temp_sensor/data", temperatureMessage,
		temperature_callback)
	
	#rospy.sleep(2)

def load_config_file():
	global pipe_map, json_data, variance, res_pipe_map, prob_move_correct,
		prob_move_inc, num_rows, num_cols

	json_data = read_config()

	pipe_map = json_data["pipe_map"]
	variance = json_data["temp_noise_std_dev"]

	num_rows = len(pipe_map)
	num_cols = len(pipe_map[0])
	prob_move_correct = json_data["prob_move_correct"]
	prob_move_inc = (1 - prob_move_correct)/4.0
	prob_x = 1.0/(num_rows * num_cols)
	res_pipe_map= [[prob_x] * num_cols] * num_rows

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
	load_config_file()	
	activate_temp()
	fetch_all_data()

if __name__ == '__main__':
	try:
		startRobot()
	except rospy.ROSInterruptException:
		pass
	
	while not rospy.is_shutdown():
		pass



