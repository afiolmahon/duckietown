#!/usr/bin/env python
'''
Author: Shih-Yuan Liu
'''
import rospkg
import yaml
from graphviz import Digraph
import sys

# TODO read from command line

config = "baseline"
# param_file = "default"
param_file = sys.argv[1]

file_path = rospkg.RosPack().get_path("duckietown") + "/config/" + config + "/fsm/fsm_node/" + param_file + ".yaml"
print "Load file: %s"%(file_path)
# Load yaml as dictionary
with file(file_path,"r") as f:
	yaml_dict = yaml.load(f)
# print yaml_dict

states_dict = yaml_dict["states"]
events_dict = yaml_dict["events"]

dot = Digraph(comment=param_file + ".yaml")

# Define state nodes
for state_name in states_dict.keys():
	dot.node(state_name,state_name)
	print "State: %s" %(state_name)

for state_name,state_dict in states_dict.items():
	transition_dict = state_dict.get("transitions")
	if transition_dict is not None:
		for event_name, next_state in transition_dict.items():
			dot.edge(state_name,next_state,label=event_name)
			print "Transition: %s -- %s --> %s " %(state_name, event_name, next_state)

dot_file_name = param_file+".dot"
with file(dot_file_name,"w") as f:
	f.write(dot.source)

print "Wrote to %s" %(dot_file_name)


