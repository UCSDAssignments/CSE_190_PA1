#!/usr/bin/env python

from cse_190_assi_1.msg import *
from cse_190_assi_1.srv import requestTexture
from cse_190_assi_1.srv import moveService
from read_config import read_config
from std_msgs.msg import String, Float32, Bool
from copy import deepcopy
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
idx = 0
prob_pub = None
txt_pub = None
temp_pub = None

def update_temp_beliefs():
	global res_pipe_map, texture_data, idx
	move_list = json_data["move_list"]

	for idx_r, row in enumerate(res_pipe_map):
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


	if idx < len(move_list):
		pipe_map = update_move_beliefs(move_list[idx])
		idx += 1
		publish_all_data(pipe_map)
	else:
		publish_all_data(None)
		deactivate_temp()
		create_output_files()
		rospy.sleep(3)
		rospy.signal_shutdown("All Done.")
		
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

def update_move_beliefs(move):
	rospy.wait_for_service("moveService")
	moveSrvProxy = rospy.ServiceProxy("moveService", moveService)
	moveSrvProxy(move)
	isLR = True if move[0] != 0 else False
	temp_pipe_map = deepcopy(res_pipe_map)

	for idx_r in range(num_rows):
		for idx_c in range(num_cols):
			currPos = (idx_r, idx_c)
			prior_prob = 0
			if isLR:
				prior_prob= res_pipe_map[idx_r][(idx_c - move[0]) % num_cols]
			else:
				isStay = True if sum(move) == 0 else False
				if isStay:
					prior_prob = res_pipe_map[idx_r][idx_c]
				else:
					prior_prob = res_pipe_map[(idx_r - move[1]) % num_rows][idx_c]
					
			updated_belief = prior_prob * prob_move_correct		
			prob = updated_belief + getSumRestOfBeliefs(currPos, move) 
			temp_pipe_map[idx_r][idx_c] = prob

	return temp_pipe_map
				
def getSumRestOfBeliefs(curr_pos, direction_taken):
	
	directions = json_data["possible_moves"]
	temp_dir = list(directions)
	temp_dir.remove(direction_taken)
	restSum = 0
	for direct in temp_dir:
		isLR = True if direct[0] != 0 else False
		new_pos = []
		
		if isLR:
			new_pos = [curr_pos[0], (curr_pos[1] - direct[0]) % num_cols]
		else:
			isStay = True if sum(direct) == 0 else False
			if isStay:
				new_pos = curr_pos
			else:
				new_pos = [(curr_pos[0] - direct[1]) % num_rows, curr_pos[1]]
			
		prior_prob = res_pipe_map[new_pos[0]][new_pos[1]]
		updated_belief = prior_prob * prob_move_inc
		restSum += updated_belief
	return restSum

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

def publish_all_data(pipe_map):	
	if pipe_map != None:
		publish_probabilities(pipe_map)
	print temperature_data.temperature
	temp_pub.publish(temperature_data.temperature)
	txt_pub.publish(texture_data)

def publish_probabilities(pipe_map):
	float_vector = RobotProbabilities()
	float_vector.data = reduce(lambda x,y: x+y, pipe_map)
	prob_pub.publish(float_vector)
	#print float_vector

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
	global prob_pub, temp_pub, txt_pub

	prob_pub = rospy.Publisher("/results/probabilities", RobotProbabilities,
		queue_size=10)
	temp_pub = rospy.Publisher("/results/temperature_data",
		Float32, queue_size=10)
	txt_pub = rospy.Publisher("/results/texture_data", String,
		queue_size=10)
	rospy.Subscriber("/temp_sensor/data", temperatureMessage,
		temperature_callback)

def load_config_file():
	global pipe_map, json_data, variance, res_pipe_map, prob_move_correct, prob_move_inc, num_rows, num_cols

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
	
	rospy.spin()



