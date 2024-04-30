from abc import ABC, abstractmethod


class Environment(ABC):
    @abstractmethod
    def get_state(self):
        pass
    
    @abstractmethod
    def get_target(self):
        pass
    
    @abstractmethod
    def dist_to_goal(self):
        pass

    @abstractmethod
    def set_state(self, state):
        pass

    @abstractmethod
    def get_random_state(self, sampler=None):
        pass
    
    @abstractmethod
    def has_collision(self):
        pass

    @abstractmethod
    def step(self, new_state):
        pass    

    @abstractmethod
    def inverse_kinematics(self, max_iter=100):
        pass
