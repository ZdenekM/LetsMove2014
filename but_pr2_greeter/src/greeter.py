#!/usr/bin/python

import roslib
roslib.load_manifest('but_pr2_greeter')
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped
from tf import TransformListener
import tf

import actionlib

import pr2_controllers_msgs.msg

from moveit_commander import MoveGroupCommander
from sound_play.libsoundplay import SoundClient

import copy
import random
import Queue
import threading
from math import sqrt, pow, fabs

import subprocess

from speech_synthesis import SpeechSynthesisClient_NICT

#from rospeex import ROSpeexInterface

class PR2Greeter:
    
    def __init__(self, debug=False, online = True, robot_frame="odom_combined"):
        
        self._tf = TransformListener()
        
        self._online = online
        
        # self.snd_handle = SoundClient()
        
        if self._online:
        
            #self._interface = ROSpeexInterface()
            #self._interface.init()
            self._speech_client = SpeechSynthesisClient_NICT()
            
        else:
            
            self.snd_handle = SoundClient()
        
        rospy.sleep(1)
        
        self.say('Hello world!')
        
        rospy.sleep(1)
        
        self._debug = debug
        self._robot_frame = robot_frame
        
        self._point_sub = rospy.Subscriber('nearest_face', PointStamped, self.face_cb)
        
        self._head_action_cl = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', pr2_controllers_msgs.msg.PointHeadAction)
        self._torso_action_cl = actionlib.SimpleActionClient('/torso_controller/position_joint_action', pr2_controllers_msgs.msg.SingleJointPositionAction)
        
        self._left_arm = MoveGroupCommander("left_arm")
        self._right_arm = MoveGroupCommander("right_arm")
        
        print "r.f.: " + self._left_arm.get_pose_reference_frame()

        self.face = None
        # self.face_from = rospy.Time(0)
        self.face_last_dist = 0
        self.last_invited_at = rospy.Time(0)
        self._person_prob_left = 0
        
        self.l_home_pose = [0.283, 0.295, 0.537, -1.646, 0.468, -1.735]
        
        self.l_wave_1 = [-0.1, 0.6, 1.15, -1.7, -0.97, -1.6]
        self.l_wave_2 = [-0.1, 0.6, 1.15, 1.7, -0.97, 1.6]
        
        self.r_home_pose = [0.124, -0.481, 0.439, -1.548, 0.36, -0.035]
        self.r_advert = [0.521, -0.508, 0.845, -1.548, 0.36, -0.035]
        
        self.no_face_random_delay = None
        
        self._initialized = False
        
        self._timer = rospy.Timer(rospy.Duration(1.0), self.timer)
        
        
        self._move_buff = Queue.Queue()
        
        self._head_buff = Queue.Queue()
        
        self._move_thread = threading.Thread(target=self.movements)
        self._move_thread.daemon = True
        self._move_thread.start()
        
        self._head_thread = threading.Thread(target=self.head)
        self._head_thread.daemon = True
        self._head_thread.start()
        
        self.new_face = False
        self.face_last_dist = 0.0
        
        self.face_counter = 0
        
        self.actions = [self.stretchingAction,
                        self.introduceAction,
                        self.waveHandAction,
                        self.lookAroundAction,
                        self.lookAroundAction,
                        self.lookAroundAction,
                        self.advertAction,
                        self.numberOfFacesAction]
        
        self.goodbye_strings = ["Thanks for stopping by.",
                                "Enjoy the event.",
                                "See you later!",
                                "Have a nice day!"]
        
        self.invite_strings = ["Hello. It's nice to see you.",
                               "Come here and take some flyer.",
                               "I hope you are enjoying the event."]
        
        rospy.loginfo("Ready")
    
    def say(self, text):
        
        if self._online:
        
            #self._interface.say(text, 'en', 'nict')
            data = self._speech_client.request(text, 'en')
            
            try:
            
              tmp_file = open('output_tmp.dat', 'wb')
              tmp_file.write(data)
              tmp_file.close()

              # play sound
              subprocess.check_output(['ffplay', 'output_tmp.dat', '-autoexit', '-nodisp'], stderr=subprocess.STDOUT)
              
            except IOError:
              rospy.logerr('file io error.')
            except OSError:
              rospy.logerr('ffplay is not installed.')
            except subprocess.CalledProcessError:
              rospy.logerr('ffplay return error value.')
            except:
              rospy.logerr('unknown exception.')
            
        else:
            
            self.snd_handle.say(text)
    
    def getRandomFromArray(self, arr):
        
        idx = random.randint(0, len(arr) - 1)
        
        return arr[idx]
    
    def stretchingAction(self):
        
        self.say("I'm bit tired. Let's do some stretching.")
        
        self._torso_action_cl.wait_for_server()
        
        goal = pr2_controllers_msgs.msg.SingleJointPositionGoal()
        
        goal.position = 0.195
        goal.max_velocity = 0.5
        
        try:
        
          self._torso_action_cl.send_goal(goal)
          self._torso_action_cl.wait_for_result(rospy.Duration.from_sec(10.0))
          
        except:
            
            rospy.logerr("torso action error (up)")
        
        # TODO move arms
        
        self.say("I feel much better now!")
        
        goal.position = 0.0
        
        try:
        
          self._torso_action_cl.send_goal(goal)
          self._torso_action_cl.wait_for_result(rospy.Duration.from_sec(10.0))
          
        except:
            
            rospy.logerr("torso action error (down)")
    
    def numberOfFacesAction(self):
        
        string = "Today I already saw " + str(self.face_counter) + "face"
        
        if self.face_counter != 1:
            
            string = string + "s"
            
        string = string + "."
        
        self.say(string)
        rospy.sleep(1)
    
    def advertAction(self):
        
        self.say("Hello. Here are some posters for you.")
        self.go(self._right_arm, self.r_advert)
        rospy.sleep(1)
        
    def introduceAction(self):
        
        self.say("Hello. I'm PR2 robot. Come here to check me.")
        rospy.sleep(1)
        
        
    def waveHandAction(self):
        
        self.say("I'm here. Please come to see me.")
        
        rand = random.randint(1, 3)
        
        for _ in range(rand):
            
            self.wave()
            
        self.go(self._left_arm, self.l_home_pose)
        
        rospy.loginfo("Waving")
        rospy.sleep(1)
        
    def lookAroundAction(self):
        
        self.say("I'm looking for somebody. Please come closer.")
        
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "/base_link"
        
        p.point.x = 5.0
        
        sign = random.choice([-1, 1])
        
        p.point.y = sign * random.uniform(1.5, 0.5)
        p.point.z = random.uniform(1.7, 0.2)
        self._head_buff.put((copy.deepcopy(p), 0.1, True))
        
        p.point.y = -1 * sign * random.uniform(1.5, 0.5)
        p.point.z = random.uniform(1.7, 0.2)
        self._head_buff.put((copy.deepcopy(p), 0.1, True))
        
        p.point.y = sign * random.uniform(1.5, 0.5)
        p.point.z = random.uniform(1.7, 0.2)
        self._head_buff.put((copy.deepcopy(p), 0.1, True))
        
        p.point.y = -1 * sign * random.uniform(1.5, 0.5)
        p.point.z = random.uniform(1.7, 0.2)
        self._head_buff.put((copy.deepcopy(p), 0.1, True))
        
        rospy.loginfo("Looking around")
        rospy.sleep(1)
        
    def getPointDist(self, pt):
        
        assert(self.face is not None)
        
        # fist, get my position
        p = PoseStamped()
        
        p.header.frame_id = "base_link"
        p.header.stamp = rospy.Time.now() - rospy.Duration(0.5)
        
        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        
        try:
            
            self._tf.waitForTransform(p.header.frame_id, self._robot_frame, p.header.stamp, rospy.Duration(2))
            p = self._tf.transformPose(self._robot_frame, p)
            
        except:
            
            rospy.logerr("TF error!")
            return None
        
        return sqrt(pow(p.pose.position.x - pt.point.x, 2) + pow(p.pose.position.y - pt.point.y, 2) + pow(p.pose.position.z - pt.point.z, 2))
        
        
        
    def getPoseStamped(self, group, c):
        
        assert(len(c) == 6)
        
        p = PoseStamped()
        
        p.header.frame_id = "base_link"
        p.header.stamp = rospy.Time.now() - rospy.Duration(0.5)
        
        p.pose.position.x = c[0]
        p.pose.position.y = c[1]
        p.pose.position.z = c[2]
        
        quat = tf.transformations.quaternion_from_euler(c[3], c[4], c[5])
        
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]
        
        try:
            
            self._tf.waitForTransform(p.header.frame_id, group.get_pose_reference_frame(), p.header.stamp, rospy.Duration(2))
            p = self._tf.transformPose(group.get_pose_reference_frame(), p)
            
        except:
            
            rospy.logerr("TF error!")
            return None
        
        return p
    
    def go(self, group, where):
        
        self._move_buff.put((group, where))
        
    def wave(self):
        
        self.go(self._left_arm, self.l_wave_1)
        self.go(self._left_arm, self.l_wave_2)
        self.go(self._left_arm, self.l_wave_1)
        
    def head(self):
        
        self._head_action_cl.wait_for_server()
        
        while not rospy.is_shutdown():
            
            (target, vel, imp) = self._head_buff.get()
            
            # we don't need to block it here
            self._head_buff.task_done()
            
            if (not imp) and (not self._head_buff.empty()):
              
              print "skipping head goal"
              
              # head target can be skipped (follow only the latest one)  
              continue
            
            # print "head point goal"
            # print target
            
            # point PR2's head there (http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20Head)
            goal = pr2_controllers_msgs.msg.PointHeadGoal()
        
            goal.target = target
	          # goal.min_duration = rospy.Duration(3.0)
            goal.max_velocity = vel
            # goal.pointing_frame = "high_def_frame"
            goal.pointing_frame = "head_mount_kinect_rgb_link"
            goal.pointing_axis.x = 1
            goal.pointing_axis.y = 0
            goal.pointing_axis.z = 0
            
            try:
            
              self._head_action_cl.send_goal(goal)
              self._head_action_cl.wait_for_result(rospy.Duration.from_sec(5.0))

            except:

               rospy.logerr("head action error")
            
            #self._head_buff.task_done()
        
    def movements(self):
        
        while not rospy.is_shutdown():
            
            (group, where) = self._move_buff.get()
            
            group.set_start_state_to_current_state()
            p = self.getPoseStamped(group, where)
            if p is None:
                
                self._move_buff.task_done()
                continue
                
            group.set_pose_target(p)
            group.set_goal_tolerance(0.1)
            group.allow_replanning(True)
            
            self._move_buff.task_done()
            
            group.go(wait=True)
    
    def timer(self, event):
        
        if self._initialized is False:
            
            rospy.loginfo("Moving arms to home positions")
            
            self.init_head()
            self.go(self._left_arm, self.l_home_pose)
            self.go(self._right_arm, self.r_home_pose)
            self._move_buff.join()
            
            self.say("I'm ready for a great job.")
            self._initialized = True
        
        
        if self.face is None:
            
            if (self.no_face_random_delay is None):
                
                delay = random.uniform(30, 10)    
                self.no_face_random_delay = rospy.Time.now() + rospy.Duration(delay)
                
                rospy.loginfo("Random delay: " + str(delay))
                
                return
                
            else:
                
                if rospy.Time.now() < self.no_face_random_delay:
                    
                    return
            
            self.init_head()
            self.go(self._left_arm, self.l_home_pose)
            self.go(self._right_arm, self.r_home_pose)
                
            rospy.loginfo("Starting selected action")
                
            action = self.getRandomFromArray(self.actions)
            
            action()
            
            delay = random.uniform(30, 10)    
            self.no_face_random_delay = rospy.Time.now() + rospy.Duration(delay)
            
            return
        
        else:
            
            self.no_face_random_delay = None
        
        if self.new_face and (self.last_invited_at + rospy.Duration(10) < rospy.Time.now()):

            self.last_invited_at = rospy.Time.now()

            self.new_face = False
            rospy.loginfo("new person")            

            self.face_counter = self.face_counter + 1
            
            # cd = getPointDist(self.face)
            
            # TODO decide action based on distance ?
            self.go(self._left_arm, self.l_home_pose)  # if a person is too close, dont move hand?
            self.go(self._right_arm, self.r_advert)
            
            string = self.getRandomFromArray(self.invite_strings)
            self.say(string)
            
            # TODO wait some min. time + say something
        
        # after 20 seconds of no detected face, let's continue 
        if self.face.header.stamp + rospy.Duration(10) < rospy.Time.now():

            rospy.loginfo("person left")
            
            string = self.getRandomFromArray(self.goodbye_strings)
            self.say(string)
            
            self.init_head()
            
            self.go(self._left_arm, self.l_home_pose)
            self.go(self._right_arm, self.r_home_pose)
            self.face = None
            
            return
        
        self._head_buff.put((copy.deepcopy(self.face), 0.4, False))
            
     
    def init_head(self):
        
        p = PointStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "/base_link"
        
        p.point.x = 2.0
        p.point.y = 0.0
        p.point.z = 1.7
        
        self._head_buff.put((p, 0.1, True))
        
    def face_cb(self, point):
        
        # transform point
        
        try:
        
            self._tf.waitForTransform(point.header.frame_id, self._robot_frame, point.header.stamp, rospy.Duration(2))
            pt = self._tf.transformPoint(self._robot_frame, point)
            
        except:
            
            rospy.logerr("Transform error")
            return
        
        if self.face is not None:
        
            cd = self.getPointDist(pt)  # current distance
            dd = fabs(self.face_last_dist - cd)  # change in distance
            
            if dd < 0.5:
        
                self.face.header = pt.header
                
                # filter x,y,z values a bit
                self.face.point = pt.point
                #self.face.point.x = (self.face.point.x + pt.point.x) / 2
                #self.face.point.y = (self.face.point.y + pt.point.y) / 2
                #self.face.point.z = (self.face.point.z + pt.point.z) / 2
                
            else:
                
               if self._person_prob_left < 2:
                   
                   self._person_prob_left += 1 
                   
               else:
                
                   self._person_prob_left = 0
                
                   print "new face ddist: " + str(dd)

                   self.new_face = True
                   self.face = pt
               
            self.face_last_dist = cd
            
        else:
            
            self._person_prob_left = 0
            self.new_face = True
            self.face = pt
            
        
if __name__ == '__main__':
    
    rospy.init_node('but_pr2_greeter')
    rospy.loginfo("PR2 Greeter")
    
    bpg = PR2Greeter(debug=False)
    
    rospy.spin()
