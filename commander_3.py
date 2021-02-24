#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
import math  # for sin
import os 
import time
import rosnode
import random
from collections import defaultdict
from heapq import *

block1_x_process_value = 0
block1_y_process_value = 0
List = [(0, 0), (0, -2), (-6, 0), (-7.5, 0), (-6, -2)]
list_size = 5

def dijkstra_raw(edges, from_node, to_node):
	g = defaultdict(list)
	for l,r,c in edges:
		g[l].append((c,r))
	q, seen = [(0,from_node,())], set()
	while q:
		(cost,v1,path) = heappop(q)
		if v1 not in seen:
			seen.add(v1)
			path = (v1, path)
			if v1 == to_node:
				return cost,path
			for c, v2 in g.get(v1, ()):
				if v2 not in seen:
					heappush(q, (cost+c, v2, path))
	return float("inf"),[]
 
def dijkstra(edges, from_node, to_node):
	len_shortest_path = -1
	ret_path=[]
	length,path_queue = dijkstra_raw(edges, from_node, to_node)
	
	if len(path_queue)>0:
		len_shortest_path = length		## 1. Get the length firstly;
		## 2. Decompose the path_queue, to get the passing nodes in the shortest path.
		left = path_queue[0]
		ret_path.append(left)		## 2.1 Record the destination node firstly;
		right = path_queue[1]
		while len(right)>0:
			left = right[0]
			ret_path.append(left)	## 2.2 Record other nodes, till the source-node.
			right = right[1]
		ret_path.reverse()	## 3. Reverse the list finally, to make it be normal sequence.
	return len_shortest_path,ret_path

def x_callback(subdata):
	#rospy.loginfo("process value x1 is %f", subdata.process_value)
	global block1_x_process_value
	block1_x_process_value = subdata.process_value

def y_callback(subdata):
	#rospy.loginfo("process value y1 is %f", subdata.process_value)
	global block1_y_process_value
	block1_y_process_value = subdata.process_value

def check_is_arrive(block_arg, expect_value, expect_limit):
	difference = block_arg - expect_value
	3
	
	if abs(difference) < expect_limit:
		return True
	else:
		return False

def block_mover():
	
	rospy.init_node("my_block_command", anonymous=True)
	### ==================== Given a list of nodes in the topology shown in Fig. 1.
	list_nodes_id = [0,1,2,3,4];
	### ==================== Given constants matrix of topology.
	M=99999	# This represents a large distance. It means that there is no link.
	### M_topo is the 2-dimensional adjacent matrix used to represent a topology.
	M_topo = [
	[M, 1,1,M,M],
	[1, M,M,M,M],
	[1, M,M,1,1],
	[M, M,1,M,M],
	[M, M,1,M,M]
	]	
	 
	### --- Read the topology, and generate all edges in the given topology.
	
	edges = []
	for i in range(len(M_topo)):
		for j in range(len(M_topo[0])):
			if i!=j and M_topo[i][j]!=M:
				edges.append((i,j,M_topo[i][j]))### (i,j) is a link; M_topo[i][j] here is 1, the length of link (i,j).


	robot_name = rospy.get_name()
	block1_x_publisher = rospy.Publisher(robot_name + "/x_axis_joint_position_controller/command", Float64, queue_size=20)
	block1_y_publisher = rospy.Publisher( robot_name + "/y_axis_joint_position_controller/command", Float64, queue_size=20)

	rospy.Subscriber(robot_name + '/x_axis_joint_position_controller/state', JointControllerState, x_callback)
	rospy.Subscriber(robot_name + '/y_axis_joint_position_controller/state', JointControllerState, y_callback)
	rate = rospy.Rate(10)
	current_position = 0
	time.sleep(5)
	while not rospy.is_shutdown():
		point = random.randint(0, list_size - 1)
		
		
		length,Shortest_path = dijkstra(edges, current_position, point)
		
		print 'length = ',length
		print 'The shortest path is ',Shortest_path
		for i in range(1, length+1):
			move_msg_x = Float64()
			move_msg_y = Float64()
			move_msg_x.data = List[Shortest_path[i]][0]
			move_msg_y.data = List[Shortest_path[i]][1]
			arrive = False
			block1_x_publisher.publish(move_msg_x)
			block1_y_publisher.publish(move_msg_y)
			while not arrive:
				if (check_is_arrive(block1_x_process_value, List[Shortest_path[i]][0], 0.05) and check_is_arrive(block1_y_process_value, List[Shortest_path[i]][1], 0.05)):
					arrive = True
				rate.sleep()
		current_position = point
		rate.sleep()
	rospy.spin()
	

# main
if __name__ == '__main__':


	try:
		block_mover()
	except rospy.ROSInterruptException: pass
