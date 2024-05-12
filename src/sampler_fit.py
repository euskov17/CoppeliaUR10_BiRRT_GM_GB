from tqdm import tqdm
from .rrt_versions.environment import Environment
from .tests_maker import DEFAULT_START
from .rrt_versions.bi_rrt import BiRRT
from .rrt_versions.rrt import RRTConnect
from .rrt_versions.rrt_connect import RRTConnect
from .ur10_wrapper import UR10
from .tests_maker import generate_tests
from sklearn.mixture import GaussianMixture

import numpy as np

def collect_data(env: UR10, start_state=DEFAULT_START, algo=BiRRT, num_runs=100,
                  algo_params = dict(), savefile="collected_data/collected_tmp.txt"):
    tests = generate_tests(env, num_runs, None)
    data = []
    for test in tqdm(tests):
        if start_state is not None:
            env.set_state(start_state)
        else:
            env.get_random_state()



        result, num_iters, steps, path = algo(env, sampler=None, **algo_params)          
        data.extend(path)

    data = np.array(data)
    if savefile is not None:
        np.savetxt(savefile, data)
    
    return data


def sampler_fit(datafile: str = "../collected_data/fixed_start_ur10_100_runs", n_components=300):
    data = np.loadtxt(datafile)
    gm = GaussianMixture(n_components=n_components).fit(data)
    return gm
    