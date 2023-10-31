#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from kiss_icp.srv import set_pose, set_poseRequest

# Variables to store the converted RPY values
reset_values = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'R': 0.0,
    'P': 0.0,
    'Y': 0.0
}

reset_action_in_progress = False  # Flag to track the reset action
odometry_data = None  # Store the odometry data
last_stored_pose = None  # Store the last stored pose
rtk_condition_met = False  # Flag to track the RTK condition

def call_reset_pose_service(pose_data):
    try:
        rospy.wait_for_service('/reset_pose')  # Wait for the service to become available
        reset_pose = rospy.ServiceProxy('/reset_pose', set_pose)

        if pose_data is not None:
            # Create a request object with the stored pose values
            request = set_poseRequest()
            request.x = pose_data['x']
            request.y = pose_data['y']
            request.z = pose_data['z']
            request.R = pose_data['R']
            request.P = pose_data['P']
            request.Y = pose_data['Y']

            # Call the service
            response = reset_pose(request)

            if response.done:
                rospy.loginfo("Reset Pose Service Response: {}".format(response))
            else:
                rospy.logwarn("Reset pose service reported an error.")
        else:
            rospy.logwarn("Odometry data is not available. Cannot reset pose.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(str(e)))

def update_stored_pose(event):
    global odometry_data, last_stored_pose
    if rtk_condition_met:
        last_stored_pose = odometry_data

def gps_raw_callback(data):
    global reset_action_in_progress, odometry_data, rtk_condition_met, reset_timer

    if data.fix_type == 6 and data.h_acc >= 14:
        # Ensure odometry_data is available
        if odometry_data is not None:
            
            if not rtk_condition_met:
                rtk_condition_met = True
                rospy.loginfo("RTK signal available, initiating reset...")
                rospy.sleep(3)

                # Check if there's no reset action in progress
                if not reset_action_in_progress:
                    reset_action_in_progress = True

                # Start or restart the timer for updating the stored pose every 2 seconds
                if reset_timer:
                    reset_timer.shutdown()  # Shutdown the existing timer if it's running
                reset_timer = rospy.Timer(rospy.Duration(2), update_stored_pose)
            call_reset_pose_service(odometry_data)
    else:
        # RTK condition not met
        if rtk_condition_met:
            rtk_condition_met = False
            reset_action_in_progress = False
            rospy.loginfo("RTK signal not available, initiating reset using the last stored pose...")
            call_reset_pose_service(last_stored_pose)


def odom_callback(data):
    global odometry_data

    # Collect the odometry data in each iteration
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (R, P, Y) = euler_from_quaternion(orientation_list)

    odometry_data = {
        'x': position.x,
        'y': position.y,
        'z': position.z,
        'R': R,
        'P': P,
        'Y': Y
    }

def main():
    global odometry_data, last_stored_pose, reset_timer
    try:
        rospy.init_node('gps_raw_subscriber', anonymous=True)

        # Subscribe to the GPS topic
        rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, gps_raw_callback)

        # Subscribe to the /vehicle/odom topic to get pose information
        rospy.Subscriber('/vehicle/odom', Odometry, odom_callback)

        odometry_data = None  # Initialize odometry_data
        last_stored_pose = None  # Initialize last_stored_pose

        reset_timer = None  # Initialize reset_timer

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
