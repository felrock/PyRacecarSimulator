from keras.models import load_model
import numpy as np
import time

class Policy():

    def __init__(self):
        self.model = load_model('../model/mlp.h5', compile=True)


    def predict_action(self, lidar):
        output = self.model.predict(lidar)
        return output


    def scale(self, array):
        return (array - np.amin(array)) / (np.amax(array) - np.amin(array))


#Example use
if __name__ == '__main__':
    policy = Policy()
    input = np.array([np.random.rand(1081)])
    action = policy.predict_action(input)
    print("Action: ", np.clip(action, -0.4189, 0.4189))
