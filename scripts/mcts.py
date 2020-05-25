from racecar_simulator_v2 import RacecarSimulator
from policy import *
from follow_the_gap import *
import followgap

import math
from collections import defaultdict

import tensorflow as tf
import numpy as np

"""
    MCTS running simulatios with a NN policy
"""

class Node:

    def __init__(self, state, scan, action, terminal=False, parent=None):
        """
            state : Numpy array[8]
        """
        self.state = np.zeros(8)
        self.state = state
        self.scan = scan
        self.parent = parent
        self.terminal = terminal
        self.action = action
        self.visits = 1
        self.reward = 0.0
        self.children = []

        # logg
        self.ro = []

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
    def visit(self):
        self.visits += 1

    @staticmethod
    def propagate(obj, reward):

        if not obj.parent:
            return
        else:
            obj.reward += reward

            # walk up the tree
            obj = obj.parent
            Node.propagate(obj, reward)

class MCTS:
    """
        Driver running a MCTS with RacecarSimulator
    """

    def __init__(self, simulator, policy_session, recent_action, roll_out_itr,
                            budget=1.0, track_point=None, with_global=False):

        self.point_to_follow = track_point
        self.with_global = with_global
        self.simulator = simulator
        # action generators
        self.policy_session = policy_session
        #self.fg = FollowTheGap()
        self.fg = followgap.PyFollowGap(10, 15.0,
                                self.simulator.config['max_steer_ang'],
                                0.004)
        self.budget = budget
        self.max_iterations = roll_out_itr
        self.action = recent_action
        self.root = None
        self.C = 0.5 # exp constant
        self.crash_pen = - 10.0

        self.speed = 2.0

    def mcts(self):
        """

        """

        root_state = self.simulator.getState()
        root_scan = self.simulator.getScan()
        self.root = Node(root_state, root_scan, self.action)
        print "root position : %f, %f" % (root_state[0], root_state[1])
        t_start = time.time()
        count = 0
        while t_start + self.budget > time.time():
            self.mctsIteration(self.root)
            count += 1

        print "MCTS Iterations: %i" % count
        action = None
        visits = -1
        for child in self.root.children:
            print "child: %f, %f, %f" %(child.visits, child.action, child.reward)
            if child.visits > visits:
                action = child.action
                visits = child.visits

        """action_list = []
        tnode = self.root
        while tnode.hasChildren():
            vs = -1
            cd = None
            for cj in tnode.children:

                if cj.visits > vs:
                    vs = cj.visits
                    cd = cj
            action_list.append(cd.action)
            tnode = cd
        print len(action_list)"""
        # return the best steering angle
        self.action = action
        return self.action

    def mctsIteration(self, node, expanded=False):
        """

        """

        if node.isTerminal():
            # cannot expand from terminated node
            return 0, False

        #
        node.visit()

        sum_of_visits = sum([x.visits for x in node.children])

        if node.hasChildren():
            action = max(node.children, key=lambda x:(x.reward/x.visits +\
                    self.C*math.sqrt(math.log(sum_of_visits)/x.visits)))

        if math.sqrt(sum_of_visits) < node.size():
            node = action
            rv, expanded = self.mctsIteration(node, expanded=False)

        if not expanded:
            new_action = self.generateActionFromFG(node)
            new_state, new_scan, terminal = self.act(node, new_action)
            new_child = Node(new_state, new_scan, new_action, terminal,
                            parent=node)
            node.addChild(new_child)
            if not terminal:
                rv = self.rollout(new_child)
            else:
                rv = self.crash_pen
            node = new_child

        Node.propagate(node, rv)
        return rv, True

    def act(self, node, action):

        prev_state = self.simulator.getState()
        self.simulator.setState(node.state)
        self.simulator.drive(self.speed, action)
        self.simulator.updatePose()
        self.simulator.runScan()
        scan = self.simulator.getScan()
        terminal = self.simulator.checkCollision() > 0
        new_state = self.simulator.getState()

        self.simulator.setState(prev_state)

        return new_state, scan, terminal

    def rollout(self, node):
        """
            Do a batch_size iteration rollout, then pass all states to
            perform scanning on gpu. Determine wheere the collision is
            with an index
        """

        prev_state = self.simulator.getState()
        self.simulator.setState(node.state)
        self.all_sim_states = np.ndarray((self.max_iterations, 3), dtype=np.float32)
        rewards = np.zeros(self.max_iterations)

        for i in xrange(self.max_iterations):
            # random action?
            if i%10 == 0:
                rand_steer = np.random.uniform(
                                -self.simulator.config['max_steer_ang'],
                                self.simulator.config['max_steer_ang'])
                rand_speed = np.random.uniform(0,
                                self.simulator.config['max_speed'])
                self.simulator.drive(rand_speed, rand_steer)

            # step
            self.simulator.updatePose()

            # save reward from step
            new_state = self.simulator.getState()
            self.all_sim_states[i][0] = new_state[0]
            self.all_sim_states[i][1] = new_state[1]
            self.all_sim_states[i][2] = new_state[2]

            #if self.with_global:
            #rewards[i] = new_state[3] / self.calculateDistance((new_state[0],new_state[1]))
            rewards[i] = new_state[3]

        index = self.simulator.checkCollisionMany(self.all_sim_states)
        self.simulator.setState(prev_state)

        if index < 0:
            node.ro = self.all_sim_states
            return np.sum(rewards) / abs(node.action)
        else:
            node.ro = self.all_sim_states[:index]
            return np.sum(rewards[:index]) / abs(node.action)

    def calculateDistance(self, current_pose):
        return math.sqrt(
        (self.point_to_follow[0] - current_pose[0])**2 +
        (self.point_to_follow[1] - current_pose[1])**2)

    def generateActionFromNN(self, node):
        if node.hasChildren():
            return self.uniSample(node.children[0].action, 0.05)
        else:
            return self.policy_session.predict_action(node.getScan())


    def generateActionFromRandom(self, node):
        return self.uniSample(0, 0.41)

    def generateActionFromFG(self, node):
        lidar = node.getScan()
        if node.hasChildren():
            return self.uniSample(node.children[0].action, 0.05)
        else:
            return self.fg.eval(lidar, len(lidar))

    def uniSample(self, prediction, dev):
        return np.random.uniform(prediction-dev, prediction+dev)

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
