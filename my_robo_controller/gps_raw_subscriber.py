#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import GPSRAW
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import asyncio
import websockets
import json

# Variables to store the converted RPY values
reset_values = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'R': 0.0,
    'P': 0.0,
    'Y': 0.0
}

def call_reset_pose_service():
    try:
        global reset_values

        rospy.loginfo("Calling external service for reset_pose")

        # Create a WebSocket connection to ROSBridge
        uri = "ws://localhost:9090"
        async def async_call_reset_pose_service():
            async with websockets.connect(uri) as websocket:
                service_msg = {
                    "op": "call_service",
                    "service": "/reset_pose",
                    "args": {
                        "x": reset_values['x'],
                        "y": reset_values['y'],
                        "z": reset_values['z'],
                        "R": reset_values['R'],
                        "P": reset_values['P'],
                        "Y": reset_values['Y']
                    }
                }

                await websocket.send(json.dumps(service_msg))
                response = await websocket.recv()
                rospy.loginfo(f"Service /reset_pose response: {response}")

        asyncio.run(async_call_reset_pose_service())
    except Exception as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def gps_raw_callback(data):
    if data.fix_type >= 5 and data.h_acc == 14:
        call_reset_pose_service()

def odom_callback(data):
    global reset_values

    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (R, P, Y) = euler_from_quaternion(orientation_list)

    reset_values['x'] = position.x
    reset_values['y'] = position.y
    reset_values['z'] = position.z
    reset_values['R'] = R
    reset_values['P'] = P
    reset_values['Y'] = Y

if __name__ == '__main__':
    try:
        rospy.init_node('gps_raw_subscriber', anonymous=True)

        # Subscribe to the GPS topic
        rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, gps_raw_callback)

        # Subscribe to the /vehicle/odom topic to get pose information
        rospy.Subscriber('/vehicle/odom', Odometry, odom_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3

# import rospy
# from mavros_msgs.msg import GPSRAW
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# import subprocess

# # Variables to store the converted RPY values
# reset_values = {
#     'x': 0.0,
#     'y': 0.0,
#     'z': 0.0,
#     'R': 0.0,
#     'P': 0.0,
#     'Y': 0.0
# }

# def launch_kiss_icp():
#     try:
#         rospy.loginfo("Launching kiss_icp odometry.launch")
#         subprocess.Popen(["roslaunch", "kiss_icp", "odometry.launch"])
#         rospy.loginfo("kiss_icp odometry.launch has been launched")
#     except Exception as e:
#         rospy.logerr("Error launching kiss_icp odometry.launch: %s", str(e))

# def call_reset_pose_service():
#     try:
#         global reset_values  # Access the global reset_values dictionary

#         rospy.loginfo("Calling external service for reset_pose")

#         # Use subprocess to call the external service using rosservice
#         reset_values_str = f"{{x: {reset_values['x']}, y: {reset_values['y']}, z: {reset_values['z']}, R: {reset_values['R']}, P: {reset_values['P']}, Y: {reset_values['Y']}}}"
#         subprocess.call(["rosservice", "call", "/reset_pose", reset_values_str])

#         rospy.loginfo("Service /reset_pose called successfully.")
#     except Exception as e:
#         rospy.logerr("Service call failed: %s", str(e))

# def gps_raw_callback(data):
#     if data.fix_type >= 5 and data.h_acc == 14:
#         call_reset_pose_service()

# def odom_callback(data):
#     global reset_values  # Access the global reset_values dictionary

#     # Extract position and orientation from Odometry message
#     position = data.pose.pose.position
#     orientation = data.pose.pose.orientation

#     # Convert orientation quaternion to roll (R), pitch (P), and yaw (Y)
#     orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
#     (R, P, Y) = euler_from_quaternion(orientation_list)

#     # Update reset_values dictionary with new values
#     reset_values['x'] = position.x
#     reset_values['y'] = position.y
#     reset_values['z'] = position.z
#     reset_values['R'] = R
#     reset_values['P'] = P
#     reset_values['Y'] = Y

# if __name__ == '__main__':
#     try:
#         rospy.init_node('gps_raw_subscriber', anonymous=True)

#         # Subscribe to the GPS topic
#         rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, gps_raw_callback)

#         # Subscribe to the /vehicle/odom topic to get pose information
#         rospy.Subscriber('/vehicle/odom', Odometry, odom_callback)

#         launch_kiss_icp()
#         rospy.sleep(10)

#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass


# #!/usr/bin/env python3

# import rospy
# from mavros_msgs.msg import GPSRAW
# import subprocess
# #from kiss_icp.srv import Reset

# def launch_kiss_icp():
#     try:
#         rospy.loginfo("Launching kiss_icp odometry.launch")
#         subprocess.Popen(["roslaunch", "kiss_icp", "odometry.launch"])
#         rospy.loginfo("kiss_icp odometry.launch has been launched")
#     except Exception as e:
#         rospy.logerr("Error launching kiss_icp odometry.launch: %s", str(e))

# def call_reset_pose_service():
#     try:

#         rospy.loginfo("Calling external service for reset_pose")

#         # Use subprocess to call the external service using rosservice
#         subprocess.call(["rosservice", "call", "/reset_pose", "{x: 0.0, y: 0.0, z: 0.0, R: 0.0, P: 0.0, Y: 0.0}"])

#     except Exception as e:
#         rospy.logerr("Service call or launch failed: %s", str(e))

# def gps_raw_callback(data):
#     if data.fix_type >= 5 and data.h_acc == 14:
#         call_reset_pose_service()

# if __name__ == '__main__':
#     try:
#         rospy.init_node('gps_raw_subscriber', anonymous=True)

#         # Subscribe to the GPS topic
#         rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, gps_raw_callback)
#         launch_kiss_icp()
#         rospy.sleep(10)

#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
