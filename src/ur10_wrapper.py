import numpy as np
import math

from .rrt_versions.environment import Environment
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class UR10(Environment):
    def __init__(self, world):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.joint_names = ["/joint{" + str(id) + "}" for id in range(0, 6)]
        self.joints = [self.sim.getObject(joint) for joint in self.joint_names]
        
        self.world = self.sim.createCollection(0)
        for object in world:
            item = self.sim.getObject('/' + object)
            # print(item, object)
            self.sim.addItemToCollection(self.world, self.sim.handle_tree, item, 0)

        self.ur10 = self.sim.createCollection(0)
        self.sim.addItemToCollection(self.ur10, self.sim.handle_tree, self.sim.getObject('/UR10/joint'), 0)

        self.sim_ik = self.client.require('simIK')
        self.ik_env = self.sim_ik.createEnvironment()
        self.ik_group = self.sim_ik.createGroup(self.ik_env)
        self.sim_ik.setGroupCalculation(self.ik_env, self.ik_group, self.sim_ik.method_pseudo_inverse, 0, 100)
        
        self.robot_base = self.sim.getObject('/UR10')
        self.end_effector = self.sim.getObject('/UR10/Dummy')
        self.target = self.sim.getObject('/Cuboid/Dummy')
        self.sim_ik.addElementFromScene(self.ik_env, self.ik_group, self.robot_base, self.end_effector, self.target,
                                self.sim_ik.constraint_pose)
        
        self.collistion_pair = tuple([self.sim.getObject('/UR10/link7_visible'), self.sim.getObject('/RG2/baseVisible')])
        self.delta = np.pi / 30
        self.target_position = np.array(self.sim.getObjectPosition(self.target))
        self.collision_checks = 0

    def reset_collision_check(self):
        self.collision_checks = 0

    def get_collision_checks_cnt(self):
        return self.collision_checks

    def get_state(self):
        return [self.sim.getJointPosition(joint) for joint in self.joints]
    
    def update_target(self):
        self.target_position = self.sim.getObjectPosition(self.target)
    
    def get_target(self):
        return self.target_position
    
    def dist_to_goal(self):
        end_position = np.array(self.sim.getObjectPosition(self.end_effector))
        return np.linalg.norm(end_position - self.target_position, 2)

    def set_state(self, state):
        for joint, value in zip(self.joints, state):
            self.sim.setJointPosition(joint, value)

    def get_random_state(self, sampler=None):
        if sampler is None:
            state = np.random.uniform(-np.pi, np.pi, len(self.joints))
        else:
            state = sampler.sample()
        self.set_state(state)
        while self.has_collision():
            if sampler is None:
                state = np.random.uniform(-np.pi, np.pi, len(self.joints))
                state[1] /= 2
            else:
                state = sampler.sample()
            self.set_state(state)
        return self.get_state()
    
    def set_zero_state(self, noise=False):
        state = np.zeros(len(self.joints))
        if noise:
            state += np.random.normal(loc=0, scale=.1, size=len(self.joints))
        self.set_state(state)

    def has_collision(self):
        self.collision_checks += 1
        if (tuple(self.sim.checkCollision(self.ur10, self.ur10)[1]) != self.collistion_pair):
            return True
        if (self.sim.checkCollision(self.ur10, self.world)[0] != 0):
            return True
        return False

    def step(self, new_state):
        prev = np.array(self.get_state())
        diff = new_state - prev
        norm = np.linalg.norm(diff, 1)

        # if norm <= self.delta:
        #     self.set_state(new_state)
        #     if self.has_collision():
        #         return False, 0, prev, []
        #     return True, 1, new_state, [new_state]
        
        # diff /= norm * self.delta

        # new_state = prev + diff
        # self.set_state(new_state)
        # if self.has_collision():
        #     return False, 0, prev, []
        # return False, 1, new_state, [new_state]

        steps = math.ceil(norm / self.delta)

        step = diff / steps
        path = []
        for step_num in range(steps):
            state = prev + step
            self.set_state(state)
            if self.has_collision():

                self.set_state(prev)
                return False, step_num, prev, path
            path.append(state)
            prev = state

        return True, steps, state, path

    def inverse_kinematics(self, max_iter=100):
        self.get_random_state()
        res, *_ = self.sim_ik.handleGroup(self.ik_env, self.ik_group, {'syncWorlds': True})
        while res != self.sim_ik.result_success or self.has_collision():
            self.get_random_state()
            res, *_ = self.sim_ik.handleGroup(self.ik_env, self.ik_group, {'syncWorlds': True})
            self.sim.addLog(self.sim.verbosity_scriptwarnings, "IK solvere failed.")
            max_iter -= 1
            if not max_iter:
                return False, self.get_state()
        return True, self.get_state()
