#!/usr/bin/env python
import random

import math

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
from rws2020_msgs.msg import MakeAPlay
# from rws2020_lib.utils import movePlayer, randomizePlayerPose, getDistanceAndAngleToTarget
import random
import math
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
import numpy as np
from visualization_msgs.msg import Marker
from rws2020_msgs.srv import Warp,WarpResponse


def getDistanceAndAngleToTarget(tf_listener, my_name, target_name,
                                time=rospy.Time(0), max_time_to_wait=1.0):
    try:
        tf_listener.waitForTransform(my_name, target_name, time, rospy.Duration(max_time_to_wait))
        (trans, rot) = tf_listener.lookupTransform(my_name, target_name, time)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
        rospy.logwarn(my_name + ': Could not get transform from ' + my_name + ' to ' + target_name)
        return None, None

    # compute distance and angle
    x, y = trans[0], trans[1]
    distance = math.sqrt(x ** 2 + y ** 2)
    angle = math.atan2(y, x)
    return distance, angle


def randomizePlayerPose(transform, arena_radius=8):
    """
    Randomizes the initial pose of a player. Based on the code by MGomes.
    :param transform: a geometry_msgs.msg.Transform() which will have the values of x,y and yaw randomized.
    :param arena_radius: the radius of the arena inside which the player can be positioned.
    """
    initial_r = arena_radius * random.random()
    initial_theta = 2 * math.pi * random.random()
    initial_x = initial_r * math.cos(initial_theta)
    initial_y = initial_r * math.sin(initial_theta)
    initial_rotation = 2 * math.pi * random.random()
    transform.translation.x = initial_x
    transform.translation.y = initial_y
    q = tf.transformations.quaternion_from_euler(0, 0, initial_rotation)
    transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
    #bla

def movePlayer(tf_broadcaster, player_name, transform_now, vel, angle, max_vel):
    """
    Moves a player given its currrent pose, a velocity, and angle, and a maximum velocity
    :param tf_broadcaster: Used to publish the new pose of the player
    :param player_name:  string with the name of the player (must coincide with the name of the tf frame_id)
    :param transform_now: a geometry_msgs.msg.Transform() containing the current pose. This variable is updated with
                          the new player pose
    :param vel: velocity of displacement to take in x axis
    :param angle: angle to turn, limited by max_angle (pi/30)
    :param max_vel: maximum velocity or displacement based on the selected animal
    """
    max_angle = math.pi / 30

    if angle > max_angle:
        angle = max_angle
    elif angle < -max_angle:
        angle = -max_angle

    if vel > max_vel:
        vel = max_vel

    T1 = transform_now

    T2 = Transform()
    T2.rotation = tf.transformations.quaternion_from_euler(0, 0, angle)
    T2.translation.x = vel
    matrix_trans = tf.transformations.translation_matrix((T2.translation.x,
                                                          T2.translation.y,
                                                          T2.translation.z))

    matrix_rot = tf.transformations.quaternion_matrix((T2.rotation[0],
                                                       T2.rotation[1],
                                                       T2.rotation[2],
                                                       T2.rotation[3]))
    matrixT2 = np.matmul(matrix_trans, matrix_rot)

    matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
                                                          T1.translation.y,
                                                          T1.translation.z))

    matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
                                                       T1.rotation.y,
                                                       T1.rotation.z,
                                                       T1.rotation.w))
    matrixT1 = np.matmul(matrix_trans, matrix_rot)

    matrix_new_transform = np.matmul(matrixT1, matrixT2)

    quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
    trans = tf.transformations.translation_from_matrix(matrix_new_transform)

    T1.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    T1.translation.x = trans[0]
    T1.translation.y = trans[1]
    T1.translation.z = trans[2]

    tf_broadcaster.sendTransform(trans, quat, rospy.Time.now(), player_name, "world")


class Player:

    def __init__(self, player_name):

        self.player_name = player_name
        self.listener = tf.TransformListener()

        self.mbocas = Marker(ns= self.player_name,id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                             )
        self.mbocas.header.frame_id = "rdcosta"
        self.mbocas.header.stamp = rospy.Time.now()
        self.mbocas.pose.position.y= 0.5
        self.mbocas.pose.orientation.w = 1
        self.mbocas.scale.z = 0.5
        self.mbocas.color.a= 1
        self.mbocas.color.r =0
        self.mbocas.color.g = 0
        self.mbocas.color.b = 0
        self.mbocas.text = "..."

        self.pub_bocas =rospy.Publisher ('/bocas', Marker, queue_size=1)

        red_team = rospy.get_param('/red_team')
        green_team = rospy.get_param('/green_team')
        blue_team = rospy.get_param('/blue_team')

        if self.player_name in red_team:
            self.my_team, self.prey_team, self.hunter_team = 'red', 'green', 'blue'
            self.my_players, self.preys, self.hunters = red_team, green_team, blue_team

        elif self.player_name in green_team:
            self.my_team, self.prey_team, self.hunter_team = 'green', 'blue', 'red'
            self.my_players, self.preys, self.hunters = green_team, blue_team, red_team

        elif self.player_name in blue_team:
            self.my_team, self.prey_team, self.hunter_team = 'blue', 'red', 'green'
            self.my_players, self.preys, self.hunters = blue_team, red_team, green_team

        else:
            rospy.logerr('My name is not in any team. I want to play!')
            exit(0)

        rospy.logwarn(self.player_name + ' starting to play ... be very afraid!!!')

        self.br = tf.TransformBroadcaster()
        self.transform = Transform()
        randomizePlayerPose(self.transform)

        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallBack)  # Subscribe make a play msg


        self.warp_server = rospy.Service('~warp', Warp, self.warpServiceCallback)

    def warpServiceCallback(self,req):
        rospy.loginfo("Someone called the service for ")


        quat = (0,0,0,1)
        trans=(req.x,req.y,0)

        self.br.sendTransform(trans, quat, rospy.Time.now(), self.player_name, "world")

        response = WarpResponse()
        response.success = True
        return response

    def makeAPlayCallBack(self, msg):

        max_vel, max_angle = msg.turtle, math.pi / 30

        if msg.blue_alive:  # PURSUIT MODE: Follow any blue player (only if there is at least one blue alive)

            self.min_dist_blue=1000000
            self.angle_blue=0
            self.idx_blue=0
            for x in range(0, len(msg.blue_alive)):
                target = msg.blue_alive[x]  # select the first alive blue player (I am hunting blue)
                distance, angle = getDistanceAndAngleToTarget(self.listener,
                                                          self.player_name, target)

                if distance< self.min_dist_blue:
                    self.min_dist_blue=distance
                    self.angle_blue=angle
                    self.idx_blue=x

            self.mbocas.text = 'Ja te apanho' + target

            for x in range(0, len(msg.red_alive)):
                targetr = msg.red_alive[x]  # select the first alive blue player (I am hunting blue)
                distancer, angler = getDistanceAndAngleToTarget(self.listener,
                                                          msg.blue_alive[self.idx_blue], targetr)

                if distancer < distance:
                    for x in range(0, len(msg.red_alive)):
                        target = msg.red_alive[x]  # select the first alive blue player (I am hunting blue)
                        distance, angle = getDistanceAndAngleToTarget(self.listener,
                                                                        self.player_name, target)
                        distance=distance
                        angle=angle
                    self.mbocas.text = 'Xauzinhooo'

            if angle is None:
                angle = 0
            vel = max_vel  # full throttle
            rospy.loginfo(self.player_name + ': Hunting ' + str(target) + '(' + str(distance) + ' away)')

            self.mbocas.header.stamp = rospy.Time.now()
            self.pub_bocas.publish(self.mbocas)

        else:  # what else to do? Lets just move towards the center
            target = 'world'
            distance, angle = getDistanceAndAngleToTarget(self.listener, self.player_name, target)
            vel = max_vel  # full throttle
            rospy.loginfo(self.player_name + ': Moving to the center of the arena.')
            rospy.loginfo('I am ' + str(distance) + ' from ' + target)
            self.mbocas.text = 'Boring'
            self.mbocas.header.stamp = rospy.Time.now()
            self.pub_bocas.publish(self.mbocas)

        # Actually move the player
        movePlayer(self.br, self.player_name, self.transform, vel, angle, max_vel)


def main():
    rospy.init_node('rdcosta', anonymous=False)
    player = Player('rdcosta')
    rospy.spin()


if __name__ == "__main__":
    main()
