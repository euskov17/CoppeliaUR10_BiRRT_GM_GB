import time 

from tqdm import tqdm
from .rrt_versions.environment import Environment
from .tests_maker import DEFAULT_START
from .rrt_versions.bi_rrt import BiRRT
from .rrt_versions.rrt import RRTConnect
from .rrt_versions.rrt_connect import RRTConnect
from .ur10_wrapper import UR10
import numpy as np

def compute_path_length(path):
    if path is None:
        return None
    length = 0.0
    for i in range(1, len(path)):
        diff = np.abs(path[i] - path[i - 1])
        diff = np.minimum(diff, 2 * np.pi - diff)
        length += np.linalg.norm(diff, 1)
    return length


def run_test_loop(tests, env: UR10, start_state=DEFAULT_START, algo=BiRRT, sampler=None, num_repeats=10,
                  algo_params = dict()):
    result = {
        'success' : [],
        'iters' : [],
        'times' : [],
        'collision_checks' : [],
        'paths' : [],
        'steps' : []
    }
    for testid, test in enumerate(tqdm(tests)):
        env.set_target_position(test)
        for _ in range(num_repeats):
            if start_state is not None:
                env.set_state(start_state)
            else:
                env.get_random_state()
                
            env.reset_collision_check()
            
            start = time.time()
            sampler.reweight()

            result, num_iters, steps, path = algo(env, sampler=sampler, **algo_params)          
            end = time.time()

            sampler.reboot()
            
            result['collision_checks'].append(env.get_collision_checks_cnt())
            result['success'].append(result)
            result['times'].append(end - start)
            result['iters'].append(num_iters)
            result['steps'].append(steps)
            result['paths'].append(compute_path_length(path))
    return result