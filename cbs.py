import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from copy import deepcopy


def detect_collision(path1, path2):
	##############################
	# Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
	#           There are two types of collisions: vertex collision and edge collision.
	#           A vertex collision occurs if both robots occupy the same location at the same timestep
	#           An edge collision occurs if the robots swap their location at the same timestep.
	#           You should use "get_location(path, t)" to get the location of a robot at time t
	max_timestep = max(len(path1), len(path2))
	collision = {}
	for t in range(max_timestep):
		#   Task 3.1: Detect a vertex collision
		if get_location(path1, t) == get_location(path2, t):
			collision = {
				'a1': None,
				'a2': None,
				'loc': [get_location(path1, t)],
				'timestep': t
			}
			return collision
		#   Task 3.1: Detect an edge collision
		if get_location(path1, t - 1) == get_location(path2, t) and get_location(path1, t) == get_location(path2, t - 1) and t > 0:
			collision = {
				'a1': None,
				'a2': None,
				'loc': [get_location(path1, t-1), get_location(path1, t)],
				'timestep': t}
			return collision
	return collision


def detect_collisions(paths):
	##############################
	# Task 3.1: Return a list of first collisions between all robot pairs.
	#           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
	#           causing the collision, and the timestep at which the collision occurred.
	#           You should use your detect_collision function to find a collision between two robots.
	collisions = []
	# Traverse through paths and find a pair of agents without any overlap
	for i in range(len(paths) - 1):
		for j in range(i + 1, len(paths)):
			temp_collision = detect_collision(paths[i], paths[j])
			if temp_collision:
				collision = {
					'a1': i,
					'a2': j,
					'loc': temp_collision['loc'],
					'timestep': temp_collision['timestep']
				}
				collisions.append(collision)
	return collisions


def standard_splitting(collision):
	##############################
	# Task 3.2: Return a list of (two) constraints to resolve the given collision
	#           Vertex collision: the first constraint prevents the first agent
	#           to be at the specified location at the specified timestep,
	#           and the second constraint prevents the second agent
	#           to be at the specified location at the specified timestep.
	
	#           Edge collision: the first constraint prevents the first agent
	#           to traverse the specified edge at the specified timestep,
	#           and the second constraint prevents the second agent
	#           to traverse the specified edge at the specified timestep
	constraints = []
	#  Task 3.2: Constraints for vertex collision
	if len(collision['loc']) == 1:
		constraints.append(
			{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
		constraints.append(
			{'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
	# Task 3.2: Constraints for edge collision
	else:
		constraints.append(
			{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False})
		constraints.append(
			{'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False})
	return constraints


def disjoint_splitting(collision):
	##############################
	# Task 4.1: Return a list of (two) constraints to resolve the given collision
	#           Vertex collision: the first constraint enforces one agent to be at the specified location at the specified timestep, and the second constraint prevents the same agent to be at the
	#  same location at the timestep.
	#           Edge collision: the first constraint enforces one agent to traverse the specified edge at the specified timestep, and the second constraint prevents the same agent to traverse the
	#    specified edge at the specified timestep
	
	#  Randomly select one of the two colliding agents
	if random.randint(0, 1) == 0:
		agent = 'a1'
	else:
		agent = 'a2'
		
	constraints = []
	# Task 4.2: Convert vertex collision to vertex constraint (one positive and one negative)
	if len(collision['loc']) == 1:
		constraints.append({
			'agent': collision[agent],
			'loc': collision['loc'],
			'timestep': collision['timestep'],
			'positive': True
		})
		constraints.append({
			'agent': collision[agent],
			'loc': collision['loc'],
			'timestep': collision['timestep'],
			'positive': False
		})
	# Task 4.2: Convert edge collision to edge constraint (one positive and one negative)
	else:
		if agent == 'a1':
			constraints.append({
				'agent': collision[agent],
				'loc': collision['loc'],
				'timestep': collision['timestep'],
				'positive': True
			})
			constraints.append({
				'agent': collision[agent],
				'loc': collision['loc'],
				'timestep': collision['timestep'],
				'positive': False
			})
		else:
			constraints.append({
				'agent': collision[agent],
				'loc': [collision['loc'][1], collision['loc'][0]],
				'timestep': collision['timestep'],
				'positive': True
			})
			constraints.append({
				'agent': collision[agent],
				'loc': [collision['loc'][1], collision['loc'][0]],
				'timestep': collision['timestep'],
				'positive': False
			})
	return constraints


def paths_violate_constraint(constraint, paths):
	assert constraint['positive'] is True
	rst = []
	for i in range(len(paths)):
		if i == constraint['agent']:
			continue
		curr = get_location(paths[i], constraint['timestep'])
		prev = get_location(paths[i], constraint['timestep'] - 1)
		if len(constraint['loc']) == 1:    # vertex constraint
			if constraint['loc'][0] == curr:
				rst.append(i)
		else:  # edge constraint
			if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
					or constraint['loc'] == [curr, prev]:
				rst.append(i)
	return rst

class CBSSolver(object):
	"""The high-level search of CBS."""
	
	def __init__(self, my_map, starts, goals):
		"""my_map   - list of lists specifying obstacle positions
		starts      - [(x1, y1), (x2, y2), ...] list of start locations
		goals       - [(x1, y1), (x2, y2), ...] list of goal locations
		"""
		
		self.my_map = my_map
		self.starts = starts
		self.goals = goals
		self.num_of_agents = len(goals)
		
		self.num_of_generated = 0
		self.num_of_expanded = 0
		self.CPU_time = 0
		
		self.open_list = []
		
		# compute heuristics for the low-level search
		self.heuristics = []
		for goal in self.goals:
			self.heuristics.append(compute_heuristics(my_map, goal))
	
	def push_node(self, node):
		heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
		print("Generate node {}".format(self.num_of_generated))
		self.num_of_generated += 1
	
	def pop_node(self):
		_, _, id, node = heapq.heappop(self.open_list)
		print("Expand node {}".format(id))
		self.num_of_expanded += 1
		return node
	
	def find_solution(self, disjoint=True):
		""" Finds paths for all agents from their start locations to their goal locations

		disjoint    - use disjoint splitting or not
		"""
		
		self.start_time = timer.time()
		
		# Generate the root node
		# constraints   - list of constraints
		# paths         - list of paths, one for each agent
		#               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
		# collisions     - list of collisions in paths
		root = {
			'cost': 0,
			'constraints': [],
			'paths': [],
			'collisions': []
		}
		for i in range(self.num_of_agents):  # Find initial path for each agent
			path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
			if path is None:
				raise BaseException('No solutions')
			root['paths'].append(path)
		
		root['cost'] = get_sum_of_cost(root['paths'])
		root['collisions'] = detect_collisions(root['paths'])
		self.push_node(root)
		
		# Task 3.1: Testing
		# print(f"3.1 Testing:\n{root['collisions']}\n")
		
		# Task 3.2: Testing
		# for collision in root['collisions']:
		# 	print(f"3.2 Testing:\n{standard_splitting(collision)}")
		
		##############################
		# Task 3.3: High-Level Search
		#           Repeat the following as long as the open list is not empty:
		#             1. Get the next node from the open list (you can use self.pop_node())
		#             2. If this node has no collision, return solution
		#             3. Otherwise, choose the first collision and convert to a list of constraints (using your
		#                standard_splitting function). Add a new child node to your open list for each constraint
	
		# Task 3.3: Implementing high-level search of CBS based on the given pseudocode.
		while len(self.open_list) > 0:
			curr = self.pop_node()
			
			# Task 3.3: Debugging
			# print(f"Node Expanded: {curr}\n")

			if len(curr['collisions']) == 0:
				self.print_results(curr)
				return curr['paths']
			
			# constraints = standard_splitting(curr['collisions'][0])
			# constraints = disjoint_splitting(curr['collisions'][0])
			if disjoint:
				constraints = disjoint_splitting(curr['collisions'][0])
			else:
				constraints = standard_splitting(curr['collisions'][0])
		
			for constraint in constraints:
				new_node = {
					'constraints': deepcopy(curr['constraints']),
					'paths': deepcopy(curr['paths'])
				}
				new_node['constraints'].append(constraint)
				
				ai = constraint['agent']
				path = a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai], ai, new_node['constraints'])

				if path:
					valid = True
					new_node['paths'][ai] = path

					# Task 4.3: Adjusting the High-level search
					if constraint['positive']:
						violators = paths_violate_constraint(constraint, new_node['paths'])
						for violator in violators:
							if violator != ai:
								new_constraint = {'agent': violator, 'loc': constraint['loc'], 'timestep': constraint['timestep'], 'positive': False}
								new_node['constraints'].append(new_constraint)
							new_path = a_star(self.my_map, self.starts[violator], self.goals[violator], self.heuristics[violator], violator, new_node['constraints'])
							if new_path:
								new_node['paths'][violator] = new_path
							else:
								valid = False
								continue
					if valid:
						new_node['collisions'] = detect_collisions(new_node['paths'])
						new_node['cost'] = get_sum_of_cost(new_node['paths'])
						self.push_node(new_node)
		return None
	
	def print_results(self, node):
		print("\n Found a solution! \n")
		CPU_time = timer.time() - self.start_time
		print("CPU time (s):    {:.2f}".format(CPU_time))
		print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
		print("Expanded nodes:  {}".format(self.num_of_expanded))
		print("Generated nodes: {}".format(self.num_of_generated))
