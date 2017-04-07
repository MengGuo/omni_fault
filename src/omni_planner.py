#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('omni_fault')
import rospy
from omni_fault.msg import pose, activity, confirmation, status, bid, decision, request
from math import sqrt, cos, sin, radians
import numpy
from robot_model_def import robot_model
import sys

import time

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def distance(pose1, pose2):
    return sqrt( (pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2 )

def confirm_callback(data):
    global confirm
    confirm = [data.confheader, data.name, data.done]
    print "Received confirmation message", confirm

def status_callback(data):
    global op_mode
    op_mode = data.mode
    print 'Received operation mode been updated to <%s>' %str(op_mode)

def request_callback(data):
    global req_data
    req_data = [data.f_agent, data.task]
    print 'Received collaboration request from Agent <%s> for task <%s>'%(req_data[0], req_data[1])

def bid_callback(data):
    global bid_data
    bid_data = [data.f_agent, data.t_agent, data.cost, data.dist]
    print 'Bid_Data received from Agent <%s> to Agent <%s> with cost <%.2f> and <dist %.2f>' %(str(bid_data[0]), str(bid_data[1]), bid_data[2], bid_data[3])

def decision_callback(data):
    global dec_data
    dec_data = [data.f_agent, data.t_agent, data.chosen]
    print 'Decision data received from Agent <%s> to Agent <%s> with content <%s>' %(str(dec_data[0]), str(dec_data[1]), str(dec_data[2]))

    
#----------------------------------------
# main planner starts here
#----------------------------------------
def omni_planner(letter, ts, act, task_formula, mode_alpha):
    global header
    global confirm
    global op_mode
    global req_data
    global bid_data
    global dec_data
    # initialization
    header = 0
    confirm = [0, None, None]
    op_mode = 'normal'
    req_data = [None, None]
    bid_data = [None, None, None, None]    
    dec_data = [None, None, None]
    collab_flag = False
    rospy.init_node('omni_planner_%s' %letter)
    print 'Agent <%s>: omni_planner started!' %(letter)
    ###### publish to
    activity_pub = rospy.Publisher('next_move_%s' %letter, activity, queue_size=10)
    collab_request_pub = rospy.Publisher('collab_request', request, queue_size=10)
    collab_reply_pub = rospy.Publisher('collab_reply', bid, queue_size=10)
    collab_decide_pub = rospy.Publisher('collab_decide', decision, queue_size=10)
    ###### subscribe to
    rospy.Subscriber('activity_done_%s' %letter, confirmation, confirm_callback)
    rospy.Subscriber('status_%s' %letter, status, status_callback)
    rospy.Subscriber('collab_request', request, request_callback)
    rospy.Subscriber('collab_reply', bid, bid_callback)
    rospy.Subscriber('collab_decide', decision, decision_callback)
    ####### agent information
    current_mode = 'normal'
    current_alpha = mode_alpha['normal']
    full_model = MotActModel(ts, act)
    omni_planner = ltl_planner(full_model, task_formula, None)
    ####### initial plan synthesis
    omni_planner.optimal(10)
    ####### main loop
    while not rospy.is_shutdown():
        next_activity = activity()
        ###############  reconfiguration due to self operation mode change
        if op_mode != current_mode:
            print '------------------------------'
            print '**********'
            print 'Agent <%s>: Operation mode changed from <%s> to <%s>' %(str(letter), str(current_mode), str(op_mode))
            print '**********'
            current_mode = op_mode
            # normal, type-I, type-II
            if op_mode in ['normal', 'type-I', 'type-II']:
                if op_mode in mode_alpha:
                    current_alpha = mode_alpha[op_mode]
                    print 'Alpha switched to <%s>' %str(current_alpha)
                else:
                    print 'Mode <%s> not found in mode_alpha!' %str(op_mode)
                    break
                omni_planner.update_model_by_alpha(current_alpha)
                print 'Agent <%s>: alpha in FTS updated' %str(letter)
                omni_planner.replan_simple()
                header = 0
                print 'Agent <%s>: plan updated' %str(letter)
            else:
                # type-III
                print 'Agent <%s> in type-III mode, *STOPPED moving*' %str(letter)
                req_msg = request()
                req_msg.f_agent = letter
                req_msg.task = task_formula
                collab_request_pub.publish(req_msg)
                print 'Request message sent by Agent <%s> for task *%s*' %(str(letter),str(task_formula))
                # --------------------
                # wait for reply
                start = time.time()
                max_wait_time = 60 # seconds
                bid_answer = dict()
                while (time.time()-start < max_wait_time) or (not bid_answer):
                    if not rospy.is_shutdown():
                        (f_agent, t_agent, cost, dist) = bid_data
                        if (t_agent == letter) and (f_agent not in bid_answer):
                            bid_answer[f_agent] = (cost, dist)
                print 'Bid_answer received from Agent <%s>: <%s>' %(str(f_agent), str(bid_answer))
                # choose collaborator
                min_dist = min([v[1] for l,v in bid_answer.items()])
                min_dist_agent = [l for l,v in bid_answer.items() if v[1] == min_dist]
                chosen_agent = min(min_dist_agent, key= lambda l: bid_answer[l][0])
                print 'Chosen agent for collaboration: <%s>' %str(chosen_agent)
                # send decision back
                for f_agent in bid_answer.keys():
                    dec_msg = decision()
                    dec_msg.f_agent = letter
                    dec_msg.t_agent = str(f_agent)
                    if f_agent == chosen_agent:
                        dec_msg.chosen = 1
                    else:
                        dec_msg.chosen = 0
                    collab_decide_pub.publish(dec_msg)
                print 'Decision for collaboration sent from Agent <%s>!' %str(letter)
        ###############  answer to collaboration request
        (f_agent, task) = req_data
        if (f_agent != None) and (op_mode != 'type-III') and (not collab_flag):
            print '------------------------------'
            print 'Collaboration request received from <%s> for task *%s*' %(str(f_agent), str(task))
            c_cost, c_dist = omni_planner.evaluate_request(task)
            bid_msg = bid()
            bid_msg.f_agent = letter
            bid_msg.t_agent = f_agent
            bid_msg.cost = c_cost
            bid_msg.dist = c_dist
            collab_reply_pub.publish(bid_msg)
            print 'Collaboration rely sent from agent <%s> to <%s> with d_cost <%.2f> and c_cost <%.2f>' %(str(letter), str(f_agent), c_cost, c_dist)
            start = time.time()
            max_wait_time = 60 # seconds
            while (time.time()-start < max_wait_time) and (not collab_flag):
                if not rospy.is_shutdown():
                    (c_f_agent, c_t_agent, c_chosen) = dec_data
                    if (c_f_agent == f_agent) and (c_t_agent == letter) and (c_chosen == 1):
                        print 'Agent <%s> chosen for collaboration' %str(c_t_agent)
                        omni_planner.confirm_request(task)
                        header = 0
                        collab_flag = True
            if c_t_agent == None:
                print 'No confirmation received'
            elif c_t_agent != letter:
                print 'Not chosen as the collaborator'
            req_data = [None, None]
        #----------------------------------------
        ############### send next move
        if (op_mode != 'type-III'):
            print '------------------------------'
            next_move = omni_planner.next_move
            ############### implement next activity
            if isinstance(next_move, str):
                # next activity is action
                next_activity.header = header
                next_activity.type = next_move
                next_activity.x = 0.0
                next_activity.y = 0.0
                print 'Agent <%s>: next action <%s>! header: <%d>' %(letter, next_activity.type, header)
                while not ((confirm[0] == header) and (confirm[1]==next_move) and (confirm[2]>0)):
                    if (not rospy.is_shutdown()) and (op_mode != 'type-III'):
                        activity_pub.publish(next_activity)
                        rospy.sleep(0.06)
                    else:
                        break
                rospy.sleep(1)
                confirm[2] = 0
                header = header + 1
                print 'Agent <%s>: action <%s> done!' %(letter, next_activity.type)
            else:
                print 'Agent <%s>: next waypoint <(%.2f,%.2f,%.2f)>! header: <%d>' %(letter, next_move[0][0], next_move[0][1], next_move[1], header)
                while not ((confirm[0] == header) and (confirm[1]=='goto') and (confirm[2]>0)):
                    if (not rospy.is_shutdown()) and (op_mode != 'type-III'):
                        next_activity.type = 'goto'
                        next_activity.header = header
                        next_activity.x = next_move[0][0]
                        next_activity.y = next_move[0][1]
                        next_activity.psi = next_move[1]
                        activity_pub.publish(next_activity)
                        rospy.sleep(0.06)
                    else:
                        break
                rospy.sleep(1)
                confirm[2] = 0
                header = header + 1
                print 'Agent <%s>: waypoint <(%.2f,%.2f,%.2f)> reached!' %(letter, next_move[0][0], next_move[0][1], next_move[1])
                omni_planner.cur_pose = [next_move[0], next_move[1]]
            print 'omni_planner.segment', omni_planner.segment
            print 'omni_planner.index', omni_planner.index
            omni_planner.find_next_move()
 
def omni_planner_agent(agent_letter):
    if agent_letter in robot_model:
        agent_ts, agent_act, agent_task, agent_mode_alpha = robot_model[agent_letter]
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
