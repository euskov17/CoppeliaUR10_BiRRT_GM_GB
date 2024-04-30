import numpy as np

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