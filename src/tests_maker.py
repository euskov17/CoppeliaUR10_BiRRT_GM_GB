import numpy as np
from rrt_versions.environment import Environment

DEFAULT_START = np.array([ 1.00812511, 
                          -0.94309862, 
                          -0.15667946, 
                          -0.04511813, 
                          -2.87206442,
                          -0.94151528])

def generate_tests(env: Environment, size: int = 100, savefile: str = "tests/tests_tmp.txt"):
    tests = []
    while len(tests) < size:
        x = np.random.uniform(-.3, -.1, 1)
        y = np.random.uniform(-.25, .55, 1)
        z = np.random.sample([1.15, 1.1, 0.85, 0.8, 0.55, 0.5, .25, .2])
        env.set_target_position([x, y, z])
        for _ in range(3):
            is_success, state = env.inverse_kinematics()
            if is_success:
                tests.append([x, y, z])
                break

    tests = np.array(tests)

    if savefile is not None:
        np.savetxt(savefile, tests)
    return tests

def get_tests(testfile="tests/base_tests.txt"):
    return np.loadtxt(testfile)
