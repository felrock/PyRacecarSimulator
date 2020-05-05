from racecar_simulator_v2 import RacecarSimulator
from policy import *

import tensorflow as tf
import numpy as np

"""
    MCTS running simulatios with a NN policy
"""

class Node:

    def __init__(self, parent, state):
        self.parent = parent
        self.state = state
        self.visits = 0
        self.reward = 0.0
        self.children = []

    def setState(self, state):
        self.state = state

    def setReward(self, reward):
        self.reward = reward

    def visit(self):
        self.visits += 1

    def hasChildren(self):
        return len(self.children) >= 1

    def expand(self, exp_func):
        new_state = exp_func(self.state)
        new_child = Node(self, new_state)
        self.children.append(new_child)


class MCTS:
    """
        Driver running a MCTS with RacecarSimulator
    """

    def __init__(self, params):

        # create CarParams instance
        # load map
        # create RacecarSim instance
        pass

    def expand(self):
        pass

    def propagate(self):
        pass

    def eval(self):
        pass


if __name__ == '__name__':
    pass

