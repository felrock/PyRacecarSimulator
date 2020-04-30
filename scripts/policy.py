import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
from tensorflow.python.platform import gfile
import numpy as np
import time

MAX_DISTANCE = 15.0
TRT_MODEL_PATH = '../model/TensorRT_model.pb'
GPU_MEMORY_FRACTION = 0.5

class Policy():

    def read_pb_graph(self, model):
        with gfile.FastGFile(model,'rb') as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())
        return graph_def

    def __init__(self):
        self.graph = tf.Graph().as_default()
        self.sess = tf.Session(config=tf.ConfigProto(
                        gpu_options=tf.GPUOptions(
                        per_process_gpu_memory_fraction=GPU_MEMORY_FRACTION)))
        self.trt_graph = self.read_pb_graph(TRT_MODEL_PATH)

        tf.import_graph_def(self.trt_graph, name='')
        self.input_ = self.sess.graph.get_tensor_by_name('input_1:0')
        self.output_ = self.sess.graph.get_tensor_by_name('dense_3/BiasAdd:0')

        self.lidar_proc = lambda x : x if x <= MAX_DISTANCE else MAX_DISTANCE


    def predict_action(self, lidar):
        lidar = np.array([[ self.lidar_proc(i) / MAX_DISTANCE for i in lidar ]])
        return self.sess.run(self.output_, feed_dict={ self.input_ : lidar })[0][0]


#Example use
if __name__ == '__main__':
    policy = Policy()

    lidar = np.array(np.random.rand(360) * 10)
    out_pred = policy.predict_action(lidar)

    tot_time = 0
    number_of_iter = 100
    for i in range(number_of_iter):
        lidar = np.array(np.random.rand(360) * 10)
        start = time.time()
        out_pred = policy.predict_action(lidar)
        end = time.time()
        tot_time = tot_time + (end-start)
        print("Prediction: ", out_pred[0],"time: ", (end-start))
    print("avg time: ", tot_time/number_of_iter)
