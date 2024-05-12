from tqdm import tqdm

import numpy as np
import random
import time

from .environment import Environment
from .tree_basics import Node, Tree

def make_path(last_node: Node):
    path = []
    while last_node is not None:
        path.append(last_node.state)
        last_node = last_node.parent
    return path[::-1]


def RRTConnect(env: Environment, max_iters=3000, 
               goal_bias_prob=.1, goal_gens=10, treshold=.1,
               verbose=False, time_limit=40, sampler=None):
    start_time = time.time()

    start_node = Node(np.array(env.get_state()))
    tree = Tree([start_node])
    
    goal_nodes = []
    for _ in range(goal_gens):
        res, state = env.inverse_kinematics()
        if res:
            goal_nodes.append(np.array(state))

    if not goal_nodes:
        return False, 0, 0, None

    num_steps = 0

    for iter in tqdm(range(max_iters)) if verbose else range(max_iters):
        if np.random.rand() < goal_bias_prob:
            new_state = random.choice(goal_nodes)
        else:
            new_state = np.array(env.get_random_state(sampler))

        closest_node = tree.get_nearest(new_state)
        
        closest_state = closest_node.state


        env.set_state(closest_state)
        _, steps, _, path = env.step(new_state)
        num_steps += steps
        prev = closest_node

        for state in path:
            new_node = Node(state, prev)
            tree.add(new_node)
            prev = new_node

        if env.dist_to_goal() < treshold:
            return True, iter + 1, num_steps, make_path(new_node)

        if time.time() - start_time > time_limit:
            return False, iter + 1, num_steps, None

    return False, max_iters, num_steps, None
