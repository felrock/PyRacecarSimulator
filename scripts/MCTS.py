from racecar_simulator_v2 import RacecarSimulator
from policy import *
from collections import defaultdict

import tensorflow as tf
import numpy as np

import math

"""
    MCTS running simulatios with a NN policy
"""

class Node:

    def __init__(self, state, action, terminal=False, parent=None):
        """
            state : Numpy array[8]
        """
        self.state = state
        self.parent = parent
        self.terminal = terminal
        self.action = action
        self.visits = 0
        self.reward = 0.0
        self.bound = (0, 0)
        self.children = []

    def getState(self):
        return self.state

    def setReward(self, reward):
        self.rewards = rewards

    def getReward(self):
        #define reward, speed - abs(steer), crash= -max_speed?
        if self.terminal:
            return -inf
        else:
            return self.state[3]

    def isTerminal(self):
        return self.terminal

    def setBound(self, center, dev):
        self.bound = (center-dev, center+dev)

    def getBound(self):
        return self.bound

    def hasChildren(self):
        return len(self.children) > 0

    def addChild(self, child):
        self.children.append(child)

    def size(self):
        return len(self.children)

    def propagate(self, reward):

        if not self.parent:
            return
        else:
            self.reward += reward
            self.visits += 1

            # walk up the tree
            return propagate(self.parent)

class MCTS:
    """
        Driver running a MCTS with RacecarSimulator
    """

    def __init__(self, simulator, budget):
        """

        """

        # create CarParams instance
        # load map
        # create RacecarSim instance
        self.simulator = simulator
        self.budget = budget
        self.budget = 1.0
        self.C = 0.1 # exp constant

    def mcts(self):
        """

        """

        t_start = time.time()
        while t_start + budget > time.time():

            mctsIteration(self.root)

        action = None
        visits = -1
        for child in self.root.children:
            if child.visits > visits:
                action = child.action
                visits = child.visits

        # return the best steering angle
        return action

    def mctsIteration(self, node, expanded=False):
        """

        """

        if node.isTerminal():
            # cannot expand from terminated node
            return 0, false

        sum_of_visits = sum(node.children, key=lambda x:x.visits)
        action = max(self.children,
                 key=lambda x:(x.reward/x.visits +
                            self.C*math.log(sum_of_visits/x.visits)))

        if math.sqrt(sum_of_visits) < node.size():
            node = action
            rv, expanded = mctsIteration(node, expanded=False)

        if not expanded:
            new_action = self.generateAction(node)
            new_state, terminal  = self.tryAction(node, new_action)
            new_child = Node(new_state, new_action, terminal, node)
            node.addChild(new_child)
            rv, steps = self.rollout(new_child)
            node = new_node

        node.propagate(rv)
        return rv, True

    def rollout(self, node):
        """
            Do a 100 iteration rollout, then pass all states to
            perform scanning on gpu. Determine wheere the collision is
            with an index
        """

        self.simulator.setState(node.state)
        self.all_sim_states = np.ndarray(dim=(100, 3), dtype=np.float32)
        rewards = np.zeros(100)

        for i in xrange(100):
            # random action?
            self.simulator.updatePose()
            new_state = self.simulator.getState()
            self.all_sim_states[i] = new_state[:3]
            rewards[i] = sim_state[3] #velocity

        index = self.simulator.checkCollisionMany(self.all_sim_states)
        return sum(reward[:index])

if __name__ == '__name__':
    pass
