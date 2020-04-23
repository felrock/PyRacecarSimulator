from keras.models import load_model
import numpy as np
import time

MAX_DISTANCE = 15.0

class Policy():

    def __init__(self):
        self.model = load_model('../model/mlp-small.h5', compile=True)
        print(self.model.summary())
        self.lidar_func = lambda x : x if x <= MAX_DISTANCE else MAX_DISTANCE

    def predict_action(self, lidar):
        lidar = np.array([[ self.lidar_func(i)/MAX_DISTANCE for i in lidar ]])
        start = time.time()
        output = self.model.predict(lidar)
        end = time.time()
        print("time: ", end-start)
        return output


#Example use
if __name__ == '__main__':
    policy = Policy()
    input = np.array(np.random.rand(360) * 10)
    action = policy.predict_action(input)
    print("Action: ", np.clip(action, -0.4189, 0.4189))
