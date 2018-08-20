#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        rospy.loginfo("Inited tl_detector ")

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.__waypoints_tree = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        # sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=2*52428800) # 52 MB buffer size

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config['is_site'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0  
        # self.img_count = 0      

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
        # rospy.loginfo("Got pose")

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.__waypoints_tree:
            base_waypoints_array = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.__waypoints_tree = KDTree(base_waypoints_array)
            rospy.loginfo("Processed base waypoints")

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # rospy.loginfo("Got positions of traffic lights")

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # rospy.loginfo("Traffic light image received")
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        # LEAVE COMMENTED - only used for testing on ROSBAG
        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")        
        #predicted_state = self.light_classifier.get_classification(cv_image)
        #rospy.loginfo("*** Predicted state " + str(predicted_state))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            rospy.logdebug('publishing '+str(light_wp))
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            rospy.logdebug('publishing es '+str(self.last_wp))
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self.__waypoints_tree.query([x, y], 1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        
        # rospy.loginfo("Camera encoding is " + self.camera_image.encoding)
        # self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # cv_image = cv_image[...,::-1]

        # if self.img_count < 1000:
        #     cv2.imwrite("/home/eddie/capstone_results/images/" + str(self.img_count) + ".png" , cv_image)
        #     self.img_count +=1


        #Get classification
        predicted_state = self.light_classifier.get_classification(cv_image)
        rospy.loginfo('got state = '+str(light.state)+" == "+str(predicted_state))
        return predicted_state
        # TODO: call the claffier instead of using the one you get from simulator.
        #return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # The traffic light should be idntified here using the trained model. For now, we are using
        # data provided by the simulator.
        # We should use image_raw instead of image_color. Also, make sure to preprocess this based on
        # the requirements of trained model

        # we want to find closest light and the closest way point to that light. i.e. for a traffic light
        # we have a line at which we should stop so find a waypoint that is closest to the line
        # at which we should stop. line_wp_index is updated with that.
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        # If we have pose of the car then only.
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            # Find the closest visible traffic light (if one exist)
            # Thie diff here indicates number of waypoints that we have generated.
            # We want to find whether one of the waypoints that we have generated lies on
            # stop line of a traffic light.
            diff = len(self.waypoints.waypoints)
            # The idea is to check all the traffic lights with us. We are interested
            # in stop line of a traffic light more than the traffic light itself as we want to
            # stop at the stop line.
            # Get the closest traffic light's stop line.
            # Exhaustive search doesn't hurt as we don't have a lot of traffic lights to search.
            for i, light in enumerate(self.lights):
                # Get the line to stop at for this traffic light
                line = stop_line_positions[i]
                # Get the closest way point to the stop line
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # check whether this stop line is the closest one so far. If so, update this
                # stop line as intended stop line. If the closest one lies out of the current waypoints
                # list than don't consider it, hence d < diff condition
                d = temp_wp_idx - car_wp_idx
                # if the closest
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            # rospy.loginfo("Got closest light identified")
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
