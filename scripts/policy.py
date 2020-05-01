import tensorflow as tf
import tensorflow.contrib.tensorrt as trt
from tensorflow.python.platform import gfile
import numpy as np
import time

MAX_DISTANCE = 15.0
TRT_MODEL_PATH = '../model/TensorRT_model.pb'
GPU_MEMORY_FRACTION = 0.75

class Policy():

    def read_pb_graph(self, model):
        with gfile.FastGFile(model,'rb') as f:
            graph_def = tf.compat.v1.GraphDef()
            graph_def.ParseFromString(f.read())
        return graph_def

    def __init__(self):
        self.graph = tf.Graph().as_default()
        self.sess = tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(
                        gpu_options=tf.compat.v1.GPUOptions(
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

	act = pd.read_csv('benchmark/straight_left_turn/actions.txt', sep=' ')
	lid = pd.read_csv('benchmark/straight_left_turn/actions.txt', sep=' ')

	X = lid.iloc[:,:].values
	y= act.iloc[:,0].values

	tot_time = 0
	
	for i in range(len(X)):
		start = time.time()
		
		lidar = []
		for j in range(180,900):
			if(j % 2 == 0):
				lidar.append(np.clip(X[i][j], 0, 15.0))

		pred = self.sess.run(self.output_, feed_dict={input_:lidar})

		end = time.time()
		pred_time = end-start
		tot_time = tot_time +pred_time
		print("Time: ", pred_time)

	print("Average time: ", tot_time/len(X))







