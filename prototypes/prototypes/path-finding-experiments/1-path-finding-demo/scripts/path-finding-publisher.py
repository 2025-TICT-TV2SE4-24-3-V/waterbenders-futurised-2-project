from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
import time

# Create a node
node = Node()

# Topic to publish velocity commands
cmd_vel_topic = "/model/FLIP/cmd_vel"

# Advertise the publisher for the Twist message
pub_twistmsg = node.advertise(cmd_vel_topic, Twist)

msg = Twist()

msg.linear.x = 0.5 # Forward/backward
msg.linear.z = 0.0 # Rotation

print(f"Publishing to {cmd_vel_topic}. Press Ctrl+C to exit.")

try:
    while True:
        # Publish the message
        if not pub_twistmsg.publish(msg):
            print("Publishing failed")
            break
        print(f"Published: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}")
        time.sleep(0.1) # Publish at 10 Hz

except KeyboardInterrupt:
    pass
# Stop the robot when exiting
# msg = Twist()
# msg.linear.x = 0.0
# msg.angular.z = 0.0
# pub.publish(msg)
# print("Stopped FLIP.")
