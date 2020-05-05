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

    """def __init__(self, parent, state):
        self.parent = parent
        self.state = state
        self.children = []
        self.terminal = False"""

    def setState(self, state):
        self.state = state

    def reward(self): #define reward, speed - abs(steer), crash= -max_speed?
        return 
    
    def isTerminal(self):
        return self.terminal

    def findChildren(self):#?
        return 
    
    def findRandomChild(self):#?
        return

    def __hash__(self): #needs to be hashable
        return 
    
    def __eq__(node_1, node_2):#compare
        return


class MCTS:
    """
        Driver running a MCTS with RacecarSimulator
    """

    def __init__(self, params):

        # create CarParams instance
        # load map
        # create RacecarSim instance
        self.rewards = defaultdict(int)
        self.visit_counts = defaultdict(int)
        self.children = dict()
        self.exploration_weight = 1


    def rollout(self, node):
        path = self.select(node)
        leaf = path[-1]
        self.expand(leaf)
        reward = self.simulate(leaf)
        self.backPropagate(path, reward)

    def choose(self, node):

        if node.isTerminal():
            raise RuntimeError(f"choose called on terminal node {node}")
        if node not in self.children:
            return node.findRandomChild()

        def score(n):
            if self.visit_counts[n] == 0:
                return float("-inf")
            return self.rewards[n] / self.visit_counts[n]

        return max(self.children[node], key=score)

    def uctSelect(self, node):
        "Select a child of node, balancing exploration & exploitation"

        assert all (n in self.children for n in self.children[node])

        log_n = math.log(self.visit_counts[node])

        def uct(n):
            return self.rewards[n] / self.visit_counts[n] + self.exploration_weight * math.sqrt(
                log_n / self.visit_counts[n]
            )

        return max(self.children[node], key=uct)

    def select(self, node):
        path = []

        while True:
            path.append(node)

            if node not in self.children or not self.children[node]:
                # node is either unexplored or terminal
                return path
            unexplored = self.children[node] - self.children.keys()
            if unexplored:
                n = unexplored.pop()
                path.append(n)
                return path
            node = self.uctSelect(node)



    def expand(self, node):
        if node in self.children:
            return #already expanded
        self.children[node] = node.findChildren()

    def simulate(self, node):
        invert_reward = True
        while True:
            if node.isTerminal():
                reward = node.reward()
                return 1 - reward if invert_reward else reward
            node = node.findRandomChild()
            invert_reward = not invert_reward

    def backPropagate(self, path, reward):
        for node in reversed(path):
            self.visit_counts[node] += 1
            self.rewards[node] += reward
            reward = 1 - reward 


if __name__ == '__name__':
    pass

