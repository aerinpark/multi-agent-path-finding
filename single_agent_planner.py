import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = dict()
    for constraint in constraints:
        if constraint['agent'] == agent:
            timestep = constraint['timestep']
            
            # Task 4.1: Add a new key 'positive' to indicate whether the constraint is positive or not.
            positive = False
            if 'positive' in constraint.keys():
                positive = constraint['positive']
            if timestep not in constraint_table:
                constraint_table[timestep] = [{'loc': constraint['loc'], 'positive': positive}]
            else:
                constraint_table[timestep].append({'loc': constraint['loc'], 'positive': positive})

            # If positive constraint, add other negative constraints to enforce the agent to stay
            if positive:
                c_loc = constraint['loc']
                for dir in range(4):
                    neg_loc = move(c_loc[0], dir)
                    # Vertex constraint
                    if len(c_loc) == 1:
                        constraint_table[timestep].append({'loc': [neg_loc], 'positive': False})
                    # Edge constraint
                    elif len(c_loc) == 2:
                        if neg_loc != c_loc[1]:
                            constraint_table[timestep].append({'loc': [c_loc[0], neg_loc], 'positive': False})
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    # Check if any constraint exists at timestep next_time
    if next_time not in constraint_table:
        return False
    for entry in constraint_table[next_time]:
        # Task 1.2: Check for vertex constraints
        if len(entry['loc']) == 1:
            if entry['loc'][0] == next_loc:
                # print("Vertex Constraint\n")
                if entry['positive']:
                    return False
                return True
        # Task 1.3: Check for edge constraints
        elif len(entry['loc']) == 2:
            if curr_loc == entry['loc'][0] and next_loc == entry['loc'][1]:
                # print("Edge Constraint\n")
                if entry['positive']:
                    return False
                return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]

    # Task 1.2: Create a constraint table before generating a root node.
    constraint_table = build_constraint_table(constraints, agent)
    
    # Task 1.4: Add goal constraints
    earliest_goal_timestep = max(constraint['timestep'] for constraint in constraints) if constraints else 0
    agent_timestep_limit = max(constraint_table.keys()) if constraint_table else 0
    upperbound = len(my_map) * len(my_map[0])
    
    # Task 1.1.1: Add a new key/value pair for the timestep. Timestep for root is 0.
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)

    # Task 1.1.2: Use tuples (cell, timestep) for closed_list
    closed_list[(root['loc']), (root['timestep'])] = root
    
    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        
        if curr['loc'] == goal_loc:
            if curr['timestep'] >= agent_timestep_limit or curr['timestep'] >= earliest_goal_timestep:
                return get_path(curr)
        if curr['timestep'] >= 500:
            return None
    
            #############################
        for dir in range(4):
            child_loc = move(curr['loc'], dir)
            
            if child_loc[0] >= len(my_map) or child_loc[0] < 0 or child_loc[1] >= len(my_map[0]) or child_loc[1] < 0:
                continue
            
            if my_map[child_loc[0]][child_loc[1]]:
                continue
                
            # Task 1.1.1: The timestep of each node is 1 larger than of its parent node.
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1}

            # Task 1.2: Check whether the new node satisfies the constraints passed to the a_star function
            # and prune it if it does not.
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc']), child['timestep']]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc']), (child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc']), (child['timestep'])] = child
                push_node(open_list, child)

        # Task 1.1.3: When generating child nodes, ensure to add a child node where the agent waits in its
        # current cell instead of moving to a neighbouring cell
        child_wait = {'loc': curr['loc'],
                      'g_val': curr['g_val'] + 1,
                      'h_val': h_values[curr['loc']],
                      'parent': curr,
                      'timestep': curr['timestep'] + 1
                      }
        if not is_constrained(child_wait['loc'], child_wait['loc'], child_wait['timestep'], constraint_table):
            if (child_wait['loc'], child_wait['timestep']) in closed_list:
                existing_node = closed_list[(child_wait['loc']), child_wait['timestep']]
                if compare_nodes(child_wait, existing_node):
                    closed_list[(child_wait['loc']), (child_wait['timestep'])] = child_wait
                    push_node(open_list, child_wait)
            else:
                closed_list[(child_wait['loc']), (child_wait['timestep'])] = child_wait
                push_node(open_list, child_wait)
    return None  # Failed to find solutions
