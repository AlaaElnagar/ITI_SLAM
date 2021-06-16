from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	ob=LaunchDescription()
	n1=Node(
		package ='rs2_project',
		executable ='control_node')
	
	n2=Node(
		package ='rs2_project',
		executable ='spawn_node')
	
	n3=Node(
		package ='turtlesim',
		executable ='turtlesim_node')
		
	ob.add_action(n1)
	ob.add_action(n2)
	ob.add_action(n3)

	return ob
	
	
