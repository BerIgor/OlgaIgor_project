import subprocess

# program parameters <map_file> <radius> <start_X> <start_Y> <start_YAW> <goal_X> <goal_Y> <planner> <probability_mod>
# <length_mod> <iteration_count> <time_per_iteration> <output_log_file> <output_ppm_file>

# constant parameters
prog = "/home/olga/OlgaIgor_project/a.out"
map_path = "/home/olga/Pictures/map_full.ppm"
radius = "5"
start_X = "1822"
start_Y = "4842"
start_YAW = "0"
goal_X = "328"
goal_Y = "1136"

# changing parametrs
planner = ["prmstar", "rrtstar", "fmtstar", "trrt"]
probability_mod = "1"
length_mod = "1"
iteration_count = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18",
                   "19", "20"]
time_per_iteration = ["0.1", "0.2", "0.5", "1", "2", "3", "4", "5"]
output_log_file = "log"
output_ppm_file = "map_result"
count = 0

for p in planner:
    for t in time_per_iteration:
        for i in iteration_count:
            args = [prog, map_path, radius, start_X, start_Y, start_YAW, goal_X, goal_Y, p, probability_mod,
                    length_mod, i, t, output_log_file, output_ppm_file + "_" + str(count)]
            subprocess.run(args)
            count += 1
