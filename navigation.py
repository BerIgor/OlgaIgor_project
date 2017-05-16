#!/usr/bin/env python

import subprocess
import sys
import os

"""
in order to run the route planner and complete the instructions, run this script with following arguments:
<map_path> <start_x> <start_y> <start_yaw> <goal_x> <goal_y> <map_resolution> <probability_modifier> <length_modifier>
map resolution is in [cm/pixel]
"""

if __name__ == "__main__":
	args = sys.argv
	map_path = args[1]
	start_X = args[2]
	start_Y = args[3]
	start_YAW = args[4]
	goal_X = args[5]
	goal_Y = args[6]
	resolution = args[7]
	probability_mod = args[8]
	length_mod = args[9]
	
	file_dir = os.path.dirname(os.path.realpath(__file__))
	prog = os.path.join(file_dir, "a.out")
	radius = str(17 / float(resolution))
	planner = "trrt"
	iteration_count = "3"
	time_per_iteration = "3"
	output_log_file = "log_new"
	output_ppm_file = "map_result"
	instruction_file = "instructions.txt"

	parser = os.path.join(file_dir,"src", "parser.py")

	planner_args = [prog, map_path, radius, start_X, start_Y, start_YAW, goal_X, goal_Y, planner, probability_mod,
					length_mod, iteration_count, time_per_iteration, output_log_file, output_ppm_file, instruction_file]
	subprocess.run(planner_args)

	parser_args = [parser, instruction_file, resolution]
	subprocess.run(parser_args)

