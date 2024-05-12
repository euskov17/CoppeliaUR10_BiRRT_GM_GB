from abc import ABC, abstractmethod
from sklearn.mixture import GaussianMixture

class Sampler(ABC):
    @abstractmethod
    def sample(self):
        pass

    @abstractmethod
    def reboot(self):
        pass

    @abstractmethod
    def reweigth(self):
        pass

class GM_Sampler(ABC):
    def __init__(self, gm: GaussianMixture):
        self.gm = gm

    def sample(self):
        return self.gm.sample()[0][0]

