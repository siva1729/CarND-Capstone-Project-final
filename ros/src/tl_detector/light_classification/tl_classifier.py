import rospy
from styx_msgs.msg import TrafficLight
import rospkg
import os,sys
import tensorflow as tf
import numpy as np
import time
import os
import cv2

def load_graph (graph_file):
    """
    Loads the frozen inference protobuf file 
    """
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='prefix')
    return graph


def filter_results(min_score, scores, classes):
    """Return tuple (scores, classes) where score[i] >= `min_score`"""
    n = len(classes)
    idxs = []
    for i in range(n):
        if scores[i] >= min_score:
            idxs.append(i)
        
    filtered_scores = scores[idxs, ...]
    filtered_classes = classes[idxs, ...]
    return filtered_scores, filtered_classes


class TLClassifier(object):
    
    def __init__(self, is_site=False):       
        self.__model_loaded = False
        self.tf_session = None
        self.prediction = None
        self.path_to_model = '../../../deep_learning/models/frozen_graphs/'
        self.sample_image_path = '../../../deep_learning/assets/test_real.png'
       # ros_root = rospkg.get_ros_root()
       # Not using the following for now to locate the classification model
       # using absolute path since the models are in deep_learning/....
       # If we decide to put the models in 'tl_detector' then we would use the following

        self.load_model(is_site)

    def load_model(self, is_site):
        detect_path = rospkg.RosPack().get_path('tl_detector')
        if is_site:
           self.path_to_model += 'real_mobilenets_ssd_38k_epochs_frozen_inference_graph.pb'
        else:
           self.path_to_model += 'sim_mobilenets_ssd_30k_epochs_frozen_inference_graph.pb'
        rospy.loginfo('model going to be loaded from '+self.path_to_model)

        # load the graph using the path to model file
        # path_to_model has the absolute path
        self.tf_graph = load_graph(self.path_to_model)
        self.config = tf.ConfigProto(log_device_placement=False)
        # GPU video memory usage setup
        self.config.gpu_options.per_process_gpu_memory_fraction = 0.8  
        #Setup timeout for any inactive option 
        self.config.operation_timeout_in_ms = 50000 

        # Declaring our placeholders
        self.image_tensor = self.tf_graph.get_tensor_by_name('prefix/image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_scores = self.tf_graph.get_tensor_by_name('prefix/detection_scores:0')

        # Number of predictions found in the image
        self.num_detections = self.tf_graph.get_tensor_by_name('prefix/num_detections:0')

        # Classification of the object (integer id)
        self.detection_classes = self.tf_graph.get_tensor_by_name('prefix/detection_classes:0')

        with self.tf_graph.as_default():            
            # tf_start_time = time.time()    
            self.tf_session = tf.Session(graph=self.tf_graph, config=self.config)        

        self.__model_loaded = True                      
        img = cv2.imread(self.sample_image_path, cv2.IMREAD_COLOR)
        self.get_classification(img)        
        rospy.loginfo("Successfully loaded model, configured placeholders, and ran inference on sample image")



    def get_classification(self, image, confidence_cutoff=0.3):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """        
        if not self.__model_loaded:
            return TrafficLight.UNKNOWN
        
        colors = ["RED", "YELLOW", "GREEN"]
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        
        # Get the scores, classes and number of detections
        # re-use the session: it is more than 3x faster than creating a new one for every image
        (scores, classes, num) = self.tf_session.run(
            [self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np})

        # Removing unecessary dimensions
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        # And pruning results below the cutoff
        final_scores, final_classes = filter_results(confidence_cutoff, scores, classes)

        end_time = time.time()

        if len(final_classes) == 0:
            rospy.loginfo("*** Predicted color did not make the cut " + colors[classes[0] - 1] + " and score is " + str(scores[0]))     
            return TrafficLight.UNKNOWN

        #TrafficLight messages have red = 0, yellow = 1, green = 2. 
        # The model is trained to identify class red = 1, yellow = 2, green = 3. 
        # Hence, subtracting 1 to match with TrafficLight message spec.
        rospy.loginfo("Predicted color is " + colors[final_classes[0] - 1] + " and score is " + str(final_scores[0])) 
        return final_classes[0] - 1      
            