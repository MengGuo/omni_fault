#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('omni_fault')
import rospy
from omni_fault.msg import pose, activity, confirmation, knowledge, status, bid, decision
from math import sqrt, cos, sin, radians
import numpy
from init import *
import sys

import time

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.omni_planner import ltl_omni_planner


confirm   = ['none', 0, 0]
header = 0
confheader = 0

def distance(pose1, pose2):
    return sqrt( (pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2 )

def confirm_callback(data):
    global confirm
    global confheader
    name = data.name
    done = data.done
    confheader = data.confheader
    confirm = [name, done, confheader]
    print "confirm", confirm

def status_callback(data):
    global op_mode
    op_mode = data.mode
    print 'operation mode', op_mode

def task_callback(data):
    global req_data
    req_data = [data.f_agent, data.task]
    print 'Agent %s request collaboration for task %s'%(req_data[0], req_data[1])

def bid_callback(data):
    global bid_data
    bid_data = [data.f_agent, data.t_agent, data.cost, data.dist]
    print 'Bid_Data received from Agent %s to Agent %s with cost %s and dist %s' %(str(bid_data[0]), str(bid_data[1]), str(bid_data[2]), str(bid_data[3]))

def decision_callback(data):
    global dec_data
    dec_data = [data.f_agent, data.t_agent, data.chosen]
    print 'Decision data received from Agent %s to Agent %s with content %s' %(str(dec_data[0]), str(dec_data[1]), str(bec_data[2]))

#----------------------------------------
# main planner starts here
#----------------------------------------
def omni_planner(letter, ts, act, task_formula, mode_alpha):
    global header
    global POSE
    global c
    global confirm
    global op_mode
    global req_data
    global bid_data
    global dec_data
    rospy.init_node('omni_planner_%s' %letter)
    print 'Agent %s: omni_planner started!' %(letter)
    ###### publish to
    activity_pub = rospy.Publisher('next_move_%s' %letter, activity, queue_size=10)
    collab_request_pub = rospy.Publisher('collab_request', task, queue_size=10)
    collab_reply_pub = rospy.Publisher('collab_reply', bid, queue_size=10)
    collab_decide_pub = rospy.Publisher('collab_decide', decision, queue_size=10)
    ###### subscribe to
    rospy.Subscriber('activity_done_%s' %letter, confirmation, confirm_callback)
    rospy.Subscriber('status_%s' %letter, status, status_callback)
    rospy.Subscriber('collab_request' %letter, task, task_callback)
    rospy.Subscriber('collab_reply' %letter, bid, bid_callback)
    rospy.Subscriber('collab_decide' %letter, decision, decision_callback)
    ####### agent information
    c = 0
    k = 0
    flag = 0
    current_mode = 'normal'
    current_alpha = mode_alpha['normal']
    full_model = MotActModel(ts, act)
    omni_planner = ltl_omni_planner(full_model, task_formula, None)
    ####### initial plan synthesis
    omni_planner.optimal(10)
    ####### main loop
    while not rospy.is_shutdown():
        next_activity = activity()
        ###############  reconfiguration due to self operation mode change
        if op_mode != current_mode:
            current_mode = op_mode
            print 'Agent %s: Operation mode changed from %s to %s' %(str(letter), str(current_mode), str(op_mode))
            # normal, type-I, type-II
            if op_mode in ['normal', 'type-I', 'type-II']
                if op_mode in mode_alpha:
                    current_alpha = mode_alpha[op_mode]
                    print 'Alpha switched to %f' %current_alpha
                else:
                    print 'Mode %s not found in mode_alpha!' %str(op_mode)
                omni_planner.update_by_alpha(current_alpha)
                print 'Agent %s: alpha in FTS updated' %str(letter)
                omni_planner.replan_simple()
                print 'Agent %s: plan updated' %str(letter)
            else:
                # type-III
                print 'Agent %s in Type-III mode, stopped moving' %str(letter)
                req_msg = task()
                req_msg.f_agent = letter
                req_msg.task = task_formula
                collab_request_pub.publish(req_msg)
                print 'Request message sent by Agent %s for task %s' %(str(letter),str(task_formula))
                # --------------------
                # wait for reply
                start = time.time()
                max_wait_time = 10 # seconds
                bid_answer = dict()
                while (time.time()-start < max_wait_time):
                    (f_agent, t_agent, cost, dist) = bid_data
                    if (t_agent == letter) and (f_agent not in bid_answer):
                        bid_answer[f_agent] = (cost, dist)
                print 'bid_answer received: %s' %str(bid_answer)
                # decide
                
        ############### send next move
        next_move = omni_planner.next_move
        next_state = omni_planner.next_state
        ############### implement next activity
        if isinstance(next_move, str):
            # next activity is action
            next_activity.header = header
            next_activity.type = next_move
            next_activity.x = -0.76
            next_activity.y = 0.30
            print 'Agent %s: next action %s!' %(letter, next_activity.type)
            while not ((confirm[0]==next_move) and (confirm[1]>0) and confirm[2] == header):
                activity_pub.publish(next_activity)
                rospy.sleep(0.06)
            rospy.sleep(1)
            confirm[1] = 0
            header = header + 1
            print 'Agent %s: action %s done!' %(letter, next_activity.type)
        else:
            print 'Agent %s: next waypoint (%.2f,%.2f,%.2f)!' %(letter, next_move[0], next_move[1], next_move[2])
            while not ((confirm[0]=='goto') and (confirm[1]>0) and confirm[2] == header):
                #relative_x = next_move[0]-POSE[0]
                #relative_y = next_move[1]-POSE[1]
                #relative_pose = [relative_x, relative_y]
                #oriented_relative_pose = rotate_2d_vector(relative_pose, -POSE[2])
                next_activity.type = 'goto'
                next_activity.header = header
                #next_activity.x = oriented_relative_pose[0]
                #next_activity.y = oriented_relative_pose[1]
                next_activity.x = next_move[0]
                next_activity.y = next_move[1]
                next_activity.psi = next_move[2]
                activity_pub.publish(next_activity)
                rospy.sleep(0.06)
            rospy.sleep(1)
            confirm[1] = 0
            header = header + 1
            print 'Agent %s: waypoint (%.2f,%.2f,%.2f) reached!' %(letter, next_move[0], next_move[1], next_move[2])
            omni_planner.pose = [next_move[0], next_move[1]]
        omni_planner.find_next_move()    

def omni_planner_agent(agent_letter):
    if agent_letter in init:
        agent_ts, agent_act, agent_task, agent_mode_alpha = init[agent_letter]
        omni_planner(agent_letter, agent_ts, agent_act, agent_task, agent_mode_alpha)
    else:
        print('Agent not specified in init.py')


if __name__ == '__main__':
    ########
    if len(sys.argv) == 2:
        agent_letter = str(sys.argv[1])
        # to run: python omni_planner.py OY
    ###############
    try:
        omni_planner_agent(agent_letter)
    except rospy.ROSInterruptException:
        pass


#=============================================
# python planner.py PY
# when manually send a message to PY
# rostopic pub -1 knowledge_PY ltl3/knowledge -- 'pyball' 'pyr3'
# after PY updates plan, it moves to 'pyr3' to execute its action 'pypoint',
# PY should publish message '['oyball', 'oyr3']' to topic 'knowledge_OY'
# rostopic pub -1 knowledge_OY ltl3/knowledge -- 'oyball' 'oyr3'
# then OY would update its plan, and moves to 'oyr3' to execute its action 'oyobsgrasp'


#=========================================
# for testing 
# rostopic pub -1 activity_done_PY ltl3/confirmation -- '0' 'goto' '10'
# rostopic pub -1 activity_done_OY ltl3/confirmation -- '0' 'goto' '10'        