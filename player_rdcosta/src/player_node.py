#!/usr/bin/env python
import math
from random import random, randrange

import numpy
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
from rws2020_msgs.msg import MakeAPlay
from std_msgs.msg import String

class Player():

    def  __init__(self, player_name):
        self.player_name = player_name

        red_team =rospy.get_param('/red_team')
        blue_team = rospy.get_param('/blue_team')
        green_team =rospy.get_param('/green_team')

        print('red_team: ' + str(red_team))
        print('green_team: ' + str(green_team))
        print('blue_team: ' + str(blue_team))

        if self.player_name in red_team:
            self.my_team, self.prey_team, self.hunter_team = 'red', 'green', 'blue'
            self.my_players, self.preys, self.hunters = red_team, green_team,blue_team
        elif self.player_name in green_team:
            self.my_team, self.prey_team, self.hunter_team =  'green', 'blue','red'
            self.my_players, self.preys, self.hunters = green_team, blue_team, red_team
        elif self.player_name in blue_team:
            self.my_team, self.prey_team, self.hunter_team = 'blue','red', 'green'
            self.my_players, self.preys, self.hunters = blue_team, red_team, green_team
        else:
            rospy.logerr('My name is not in any team. I want to play!')
            exit(0)

        rospy.logwarn('I am ' + self.player_name + ' and I am on team ' + self.my_team + '. '+ self.prey_team + ' players are all going die!')
        rospy.loginfo('I am affraid of ' + str(self.hunters))

        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallBack)
        self.br = tf.TransformBroadcaster()
        self.transform =Transform()
        self.transform.translation.x = 4
        self.transform.translation.y = -4


    def makeAPlayCallBack(self,msg):

        self.max_vel = msg.turtle
        self.max_angle = math.pi / 30
        print('Received message make a play ... my max velocity is ' + str(self.max_vel))

        # Make a play
        vel = self.max_vel  # full throttle
        angle = self.max_angle

        self.move(self.transform, vel/10, angle)

    def move(self, transform_now, vel, angle):

            if angle > self.max_angle:
                angle = self.max_angle
            elif angle < -self.max_angle:
                angle = -self.max_angle

            if vel > self.max_vel:
                vel = self.max_vel

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
            matrixT2 = numpy.matmul(matrix_trans, matrix_rot)

            matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
                                                                  T1.translation.y,
                                                                  T1.translation.z))

            matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
                                                               T1.rotation.y,
                                                               T1.rotation.z,
                                                               T1.rotation.w))
            matrixT1 = numpy.matmul(matrix_trans, matrix_rot)

            matrix_new_transform = numpy.matmul(matrixT2, matrixT1)

            quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
            trans = tf.transformations.translation_from_matrix(matrix_new_transform)

            self.transform.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
            self.transform.translation.x = trans[0]
            self.transform.translation.y = trans[1]
            self.transform.translation.z = trans[2]

            self.br.sendTransform(trans, quat, rospy.Time.now(),
                                  self.player_name, "world")



def callback(msg):
    print("Received a message containing string:" + msg.data)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    print("Hello player node!")
    rospy.init_node('rdcosta', anonymous=False)

    player = Player('rdcosta')


    rospy.spin()

if __name__=="__main__":
    main()