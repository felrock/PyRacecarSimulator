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

    def __init__(self, state, scan, action, terminal=False, parent=None):
        """
            state : Numpy array[8]
        """
        self.state = state
        self.scan = scan
        self.parent = parent
        self.terminal = terminal
        self.action = action
        self.visits = 0
        self.reward = 0.0
        self.bound = (0, 0)
        self.children = []

    def getScan(self):
        return self.scan

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
            self = self.parent
            return self.propagate(reward)

class MCTS:
    """
        Driver running a MCTS with RacecarSimulator
    """

    def __init__(self, simulator, policy_session,  budget=1.0):
        """

        """

        self.simulator = simulator
        self.policy_session = policy_session
        self.budget = budget
        self.action = 0.0
        self.root = None
        self.C = 0.1 # exp constant
        self.crash_pen = -10

        self.speed = 1.0

    def mcts(self):
        """

        """
        root_state = self.simulator.getState()
        root_scan = self.simulator.getScan()
        self.root = Node(root_state, root_scan, self.action)

        t_start = time.time()
        while t_start + self.budget > time.time():

            self.mctsIteration(self.root)

        action = None
        visits = -1
        for child in self.root.children:
            if child.visits > visits:
                action = child.action
                visits = child.visits

        # return the best steering angle
        self.action = action
        return action, root_state

    def mctsIteration(self, node, expanded=False):
        """

        """

        if node.isTerminal():
            # cannot expand from terminated node
            return 0, False

        sum_of_visits = sum([x.visits for x in node.children])
        if node.hasChildren():
            action = max(node.children, key=lambda x:(x.reward/x.visits +\
                                self.C*math.log(sum_of_visits/x.visits)))

        if math.sqrt(sum_of_visits) < node.size():
            node = action
            rv, expanded = self.mctsIteration(node, expanded=False)

        if not expanded:
            new_action = self.generateAction(node)
            new_state, new_scan, terminal = self.act(node, new_action)
            new_child = Node(new_state, new_scan, new_action, terminal, node)
            node.addChild(new_child)
            if not terminal:
                rv, steps = self.rollout(new_child)
            else:
                rv, steps = self.crash_pen, 0
            node = new_child

        node.propagate(rv)
        return rv, True

    def act(self, node, action):

        self.simulator.setState(node.state)
        self.simulator.drive(self.speed, action)
        self.simulator.updatePose()
        self.simulator.runScan()
        scan = self.simulator.getScan()
        terminal = self.simulator.checkCollision()
        new_state = self.simulator.getState()

        return new_state, scan, terminal

    def rollout(self, node):
        """
            Do a 100 iteration rollout, then pass all states to
            perform scanning on gpu. Determine wheere the collision is
            with an index
        """

        self.simulator.setState(node.state)
        self.all_sim_states = np.ndarray((100, 3), dtype=np.float32)
        rewards = np.zeros(100)

        for i in xrange(100):
            # random action?
            self.simulator.updatePose()
            new_state = self.simulator.getState()
            self.all_sim_states[i][0] = new_state[0]
            self.all_sim_states[i][1] = new_state[1]
            self.all_sim_states[i][2] = new_state[2]
            rewards[i] = new_state[3] #velocity

        index = self.simulator.checkCollisionMany(self.all_sim_states)

        return np.sum(rewards[:index]), index

    def generateAction(self, node):

        if node.hasChildren():
            # make faster
            return self.sample(node.action, 11, 0.13*2/11)
        else:
            # make faster
            return self.policy_session.predict_action(node.getScan())


    def sample(self, prediction, length, step): #uneven length
        samples = np.arange(prediction-(step*length/2),
                    prediction + (step*length/2 + step),
                    step, dtype=float).round(decimals=2)

        interval = np.random.random_sample()

        if interval >= 0.4:
            return np.random.choice(samples[int(length/3):int(2*length/3)])
        elif interval < 0.4 and interval >= 0.2:
            return np.random.choice(samples[0:int(length/3)])
        elif interval < 0.2:
            return np.random.choice(samples[int(2*length/3):length])

if __name__ == '__name__':
    pass
