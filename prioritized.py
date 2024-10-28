import math
import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        constraints = []
        # Task 1.2: Handling Vertex Constraint
        # constraints = [
        #     {'agent': 0,
        #      'loc': [(1, 5)],
        #      'timestep': 4
        #      }
        # ]

        # Task 1.3: Handling Edge Constraint
        # constraints = [
        #     {'agent': 1,
        #      'loc': [(1, 2), (1, 3)],
        #      'timestep': 1
        #      }
        # ]

        # Task 1.4: Handling Goal Constraint
        # constraints = [
        #     {
        #         'agent': 0,
        #         'loc': [(1, 5)],
        #         'timestep': 10
        #     }
        # ]

        # Task 1.5: Designing Constraints
        # constraints = [
        #     {
        #         'agent': 1,
        #         'loc': [(1, 4)],
        #         'timestep': 2
        #     },
        #     {
        #         'agent': 1,
        #         'loc': [(1, 3)],
        #         'timestep': 2
        #     },
        #     {
        #         'agent': 1,
        #         'loc': [(1, 2)],
        #         'timestep': 2
        #     }
        # ]
        
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            
            result.append(path)
            # print(f"path for a{i}\n{path}\n")

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            
            # Task 2: Create nested loops
            # for each agent, check if its path conflicts with the path of other agents
            for agent in range(self.num_of_agents):
                for timestep in range(len(path)):
                    # Task 2.1: Add vertex constraints
                    if agent != i:
                        constraints.append(
                            {
                                'agent': agent,
                                'loc': [path[timestep]],
                                'timestep': timestep,
                                'positive': False
                            }
                        )
                        # Task 2.2: Add edge constraints
                        if timestep > 0:
                            constraints.append(
                                {
                                    'agent': agent,
                                    'loc': [path[timestep], path[timestep - 1]],
                                    'timestep': timestep,
                                    'positive': False
                                }
                            )
                # Task 2.3: Add additional constraints
                upperbound = sum(result[0:len(result)-2]) + (len(self.my_map) + len(self.my_map[0]))
                if i != agent:
                    for goal_constraint in range(len(path), upperbound):
                        constraints.append(
                            {
                                'agent': agent,
                                'loc': [path[-1]],
                                'timestep': goal_constraint,
                                'positive': False
                            })
            ##############################
        

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
