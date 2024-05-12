from .environment import Environment
from .tree_basics import Node, Tree
from tqdm import tqdm

import numpy as np
import time

def make_bipath(last_node1, last_node2):
    path = []
    last_node2 = last_node2.parent
    while last_node2 is not None:
        path.append(last_node2.state)
        last_node2 = last_node2.parent
    path = path[::-1]
    while last_node1 is not None:
        path.append(last_node1.state)
        last_node1 = last_node1.parent
    
    return path[::-1]


def BiRRT(env: Environment, max_iters=3000, goal_gens=10, 
          near_radius=2, verbose=False, time_limit=90,
          sampler=None):
    start_time = time.time()
    
    start = np.array(env.get_state())
    start_node = Node(start)
    
    goal_nodes = []
    for _ in range(goal_gens):
        res, state = env.inverse_kinematics()
        if res:
            goal_nodes.append(Node(np.array(state)))
    
    if len(goal_nodes) == 0:
        return False, 0, 0, None
    
    trees = [Tree([start_node]), Tree(goal_nodes)]
    
    for iter in tqdm(range(max_iters)) if verbose else range(max_iters): 
        new_state = np.array(env.get_random_state(sampler))

        connection = False
        num_steps = 0
        added = None
        for id in range(2):
            closest_node = trees[id].get_nearest(new_state)
            closest_state = closest_node.state
            
            diff = new_state - closest_state
            norm = np.linalg.norm(diff, 1)
            target_state = new_state
            if norm > near_radius:
                target_state = closest_state + diff / norm * near_radius
            
            env.set_state(closest_state)
            result, steps, last_state, path = env.step(target_state)

            connection = (added is not None) and np.allclose(last_state, added)
            added = last_state
            
            prev = closest_node
            for state in path:
                new_node = Node(state, prev)
                trees[id].add(new_node)
                prev = new_node

            num_steps += steps

        if connection:
            return True, iter, num_steps, make_bipath(trees[0].get_last(), trees[1].get_last())

        if time.time() - start_time > time_limit:
            return False, iter, num_steps, None
        
    return False, max_iters, num_steps, None
