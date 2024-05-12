from rrt_versions.sampler import Sampler
from rrt_versions.environment import Environment
from sklearn.mixture import GaussianMixture

import numpy as np

def compute_weights(means, env: Environment):
    goal_pos = np.array(env.get_target_position())
    dists = np.linalg.norm(goal_pos[None, :] - means, axis=1)
    dists = 1 / (.1 + np.log(np.array(dists) + 1))
    return dists


def get_means_positions(means, env: Environment):
    means_pos = []
    for mean in means:
        env.set_state(mean)
        means_pos.append(env.get_end_pos())
    return np.array(means_pos)

class GB_GM_Sampler(Sampler):
    def __init__(self, gm: GaussianMixture, env: Environment):
        self.gm = gm
        self.gm_weights = gm.weights_
        self.positions = get_means_positions(self.gm.means_)
        self.env = env

    def sample(self):
        return self.gm.sample()[0][0]
    
    def reboot(self):
        self.gm.weights_ = self.gm_weights

    def reweigth(self):
        weights = compute_weights(self.positions, self.env)
        self.gm.weights_ = self.gm_weights * weights
        self.gm.weights_ /= self.gm.weights_,sum()