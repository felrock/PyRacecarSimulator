#!/usr/bin/env python
import sys
import numpy as np

class FollowTheGap:

    def __init__(self):
        pass
    def smooth_filter(self, x, window_len):

        s = np.r_[x[window_len-1:0:-1], x, x[-2:-window_len-1:-1]]
        w = np.ones(window_len, 'd')
        y = np.convolve(w/w.sum(), s, mode='valid')
        return y

    def preprocessLidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
                1.Setting each value to the mean over some window
                2.Rejecting high values (eg. > 3m)
        """
        WINDOW_SIZE = 10
        MAX_ACCEPTED_DISTANCE = 15.0
        proc_ranges = []
        for i in range(len(ranges)-10):
            if not ranges[i]:
                proc_ranges.append(0.0)
            elif ranges[i] > MAX_ACCEPTED_DISTANCE:
                proc_ranges.append(MAX_ACCEPTED_DISTANCE)
            else:
                proc_ranges.append(ranges[i])

        return list(self.smooth_filter(proc_ranges, WINDOW_SIZE))

    def findMaxGap(self, vector):
        """ Return the start index & end index of the max gap in vector
        """
        VECTOR_SIZE = len(vector)
        current_start = 0
        current_size = 0
        max_start = 0
        max_size = 0

        current_index = 0

        while current_index < VECTOR_SIZE:
            current_start = current_index
            current_size = 0
            while current_index < VECTOR_SIZE and vector[current_index] > 1.75:
                current_size += 1
                current_index += 1
            if current_size > max_size:
                max_start = current_start
                max_size = current_size
                current_size = 0
            current_index += 1
        if current_size > max_size:
            max_start = current_start
            max_size = current_size

        return max_start, (max_start + max_size + 1)

    def findBestPoint(self, start_i, end_i):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """

        return (start_i + end_i)/2

    def safetyBubble(self, vector, center_point, radius):

        vector[center_point] = 0.0
        current_index = center_point
        while (current_index < len(vector)-1) and current_index - radius > center_point:
            vector[current_index] = 0.0
            current_index += 1
        while current_index > 0 and current_index + radius < center_point:
            vector[current_index] = 0.0
            current_index -= 1


    def getSteerAng(self, scan, best_point, closest_distance):

        if best_point > len(scan)/2:
            angle = - 0.004 * (len(scan)/2 - best_point)
        else:
            angle = 0.004 * (best_point - len(scan)/2)

        steering_angle = 2 * angle/(scan[best_point])

        return np.clip(steering_angle, -0.4189, 0.4189)

    def getAction(self, lidar):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        # Process lidar
        proc_ranges = self.preprocessLidar(lidar)

        # Find closest point to LiDAR
        min_point = 0
        for i in range(len(proc_ranges)):
            if proc_ranges[i] != 0 and proc_ranges[i] < proc_ranges[min_point]:
                min_point = i

        # Eliminate all points inside 'bubble' (set them to zero)
        self.safetyBubble(proc_ranges, min_point, 5)
        # Find max length gap
        start, end = self.findMaxGap(proc_ranges)
        # Find the best point in the gap
        best_point = self.findBestPoint(start, end)

        # Find steering angle
        angle = self.getSteerAng(
            lidar, best_point, lidar[min_point])
        return angle

if __name__ == '__main__':
    pass
