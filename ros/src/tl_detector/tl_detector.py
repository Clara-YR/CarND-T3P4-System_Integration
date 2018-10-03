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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Set up the TL Classifier
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        # Set Callbacks
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        # KD Tree
        if not self.waypoints_2d:
            # get (x,y) for each waypoint
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # construct data structure KDTree to look up the cloest point 
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
    	"""
    	Traffic light information callback
    	"""
    	# For testing, just return the light state
        self.lights = msg.lights

        """
        if(not self.has_image):
        	self.prev_light_loc = None
        	return False

        cv_image = self.bredge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)
        """

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg 
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # If state change, update state and reset state_count
        if self.state != state:
            self.state_count = 0
            # update self.state to store the state of the closest TL
            self.state = state  
 
 		# If the TL Classifier judge 3 countinuous TL image in the same state, we should take action
        elif self.state_count >= STATE_COUNT_THRESHOLD: 
            # Update the state of the closest Traffic Light
            self.last_state = self.state
            # Update the stop line of the closest Traffic Light
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            # Update the index of the closest Traffic Light if it's RED
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        
        # Before the TF Classifier make next judgement, take the same action as previous step
        else:
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
        #TODO implement
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        # check is the closest wp is ahead of the vehicle
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_index = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.postion.x, self.pose.pose.postion.y)

        	#TODO find the closest visible traffic light (if one exists)
        	idx_diff = len(self.base_waypoints.waypoints)
        	for i, light in enumerate(self.lights):
        		# Get stop line wayopint index
        		line = stop_line_positions[i]
        		temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
        		# Find closest stop line waypoint index
        		d = temp_wp_idx - car_wp_idx
        		if d >= 0 and d < idx_diff:
        			idx_diff = d
        			closest_light = light
        			line_wp_index = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return light_wp_idx, state
        #self.waypoints = None

        # if no traffic light return -1, if can't tell traffic light state return TraffictLight.UNKNOWN
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
