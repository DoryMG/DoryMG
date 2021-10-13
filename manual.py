import rospy
from geometry_msgs.msg import Twist

def velocity_command(turtle, x_vel=0.0, a_vel=0.3):
    cmd = Twist()
    cmd.linear.x = x_vel
    cmd.angular.z = a_vel
    turtle.publish(cmd)

def stop(turtle):
    velocity_command(turtle, 0.0, 0.0)

def go_forward(turtle, duration=2.0):
    velocity_command(turtle, 0.1, 0.0)
    rospy.sleep(duration)
    stop(turtle)

def turn(turtle, duration=2.0):
    velocity_command(turtle, 0.1, 0.3)
    stop(turtle)

if __name__ == "__main__":
    rospy.init_node("manual")
    turtle = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #go_forward(turtle)
    #turn(turtle)
    cmd = Twist()
    cmd.angular.z = 0.3
    rospy.loginfo(cmd)
    turtle.publish(cmd)
    rospy.spin()
