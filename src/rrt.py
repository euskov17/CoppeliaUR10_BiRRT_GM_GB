from dataclasses import dataclass
from ur10_wrapper import UR10
from tqdm import tqdm
import numpy as np
import random
import time

class Node:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent

class Tree:
    def __init__(self, nodes):
        self.nodes = nodes

    def get_nearest(self, state):
        closest_node = self.nodes[0]
        best_dist = np.linalg.norm(state - closest_node.state, 1)
        for i in range(1, len(self.nodes)):
            dist = np.linalg.norm(state - self.nodes[i].state, 1)
            if dist < best_dist:
                best_dist = dist
                closest_node = self.nodes[i]
        return closest_node
    
    def add(self, node):
        self.nodes.append(node)

    def get_last(self):
        return self.nodes[-1]

def make_path(last_node):
    path = []
    while last_node is not None:
        path.append(last_node.state)
        last_node = last_node.parent
    return path[::-1]


def RRTConnect(manip: UR10, max_iters=3000, goal_bias_prob=.1, goal_gens=10, treshold=.1,
               verbose=False, sampler=None):
    start_node = Node(np.array(manip.get_state()))
    tree = Tree([start_node])
    
    goal_nodes = []
    for _ in range(goal_gens):
        res, state = manip.inverse_kinematics()
        if not res:
            return False, 0, 0, None
        goal_nodes.append(np.array(state))


    num_steps = 0
    for iter in tqdm(range(max_iters)) if verbose else range(max_iters):
        if np.random.rand() < goal_bias_prob:
            new_state = random.choice(goal_nodes)
            is_goal = True
            # print("Goal")
            # print(new_state)
        else:
            new_state = np.array(manip.get_random_state(sampler))
            is_goal = False
            # print("Uniform") 

        closest_node = tree.get_nearest(new_state)
        
        closest_state = closest_node.state
        
        # manip.set_state(closest_state)
        # result, last_state = manip.step(new_state)

        manip.set_state(closest_state)
        result, steps, last_state, path = manip.step(new_state)
        num_steps += steps
        prev = closest_node
        for state in path:
            # print(state)
            new_node = Node(state, prev)
            tree.add(new_node)
            prev = new_node

        if is_goal and result:
            if manip.dist_to_goal() >= treshold:
                print("Error")
        if manip.dist_to_goal() < treshold:
            return True, iter + 1, num_steps, make_path(new_node)
    
    return False, max_iters, num_steps, None

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


def BiRRT(manip: UR10, max_iters=3000, goal_bias_prob=.1, goal_gens=10, 
          near_radius=2, treshold=.05, verbose=False, time_limit=40,
          sampler=None):
    # start_time = time.process_time()
    start_time = time.time()
    
    start = np.array(manip.get_state())
    start_node = Node(start)
    
    goal_nodes = []
    for _ in range(goal_gens):
        res, state = manip.inverse_kinematics()
        if res:
            goal_nodes.append(Node(np.array(state)))
    
    if len(goal_nodes) == 0:
        return False, 0, 0, None
    
    # goal_nodes = [Node(np.array(manip.inverse_kinematics())) for _ in range(goal_gens)]
    trees = [Tree([start_node]), Tree(goal_nodes)]
    
    for iter in tqdm(range(max_iters)) if verbose else range(max_iters): 
        new_state = np.array(manip.get_random_state(sampler))

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
            
            manip.set_state(closest_state)
            result, steps, last_state, path = manip.step(target_state)

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