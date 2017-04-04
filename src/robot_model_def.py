import sys

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


from math import pi as PI

def combine_wps_angle(wps_label, angle_label):
    nodes_label = dict()
    symbols = set()
    for wp, wp_l in wps_label.iteritems():
        for angle, angle_l in angle_label.iteritems():
            # node = ((x,y), theta)
            node = (wp, angle)
            node_l = wp_l.union(angle_l)
            nodes_label[node] = node_l
            symbols.update(node_l)
    return nodes_label, symbols



# ========================================
# robot model description starts
robot_model = dict()

# angle discrete
# # [-3.1+3.1415*k/4 for k in range(8)]
# angle_label = {
#     -3.1: set(['w',]),
#     -2.3: set(['sw',]),
#     -1.5: set(['s',]),
#     -0.7: set(['se',]),
#     0.04: set(['e',]),
#     0.8: set(['ne',]),
#     1.6: set(['n',]),
#     2.3: set(['nw',]),        
# }

angle_label = {
    -3.14: set(['w',]),
    -1.57: set(['s',]),
    0.0: set(['e',]),
    1.57: set(['n',]),
}



# la = 'place to load object a'
# ua = 'place to unload object a'
# h = 'home base'
node_label = {
    (3.0, 1.0): set(['la1',]),
    (4.0, 1.0): set(['la2',]),
    (5.0, 1.0): set(['ub',]),    
    (1.0, 3.0): set(['lb1',]),
    (1.0, 4.0): set(['lb2',]),
    (1.0, 5.0): set(['ua',]),
    (4.0, 4.0): set(['h1',]),
    (5.0, 5.0): set(['h2',]),
}

#comb_nodes, symbols = combine_wps_angle(node_label, angle_label)

comb_nodes = {
    ((3.0, 1.0), -3.14): set(['la1','w']),
    ((4.0, 1.0), 0.0): set(['la2','e']),
    ((5.0, 1.0), 1.57): set(['ub','n']),    
    ((1.0, 3.0), -3.14): set(['lb1','w']),
    ((1.0, 4.0), 0.0): set(['lb2','e']),
    ((1.0, 5.0), 1.57): set(['ua','n']),
    ((4.0, 4.0), -1.57): set(['h1','s']),
    ((5.0, 5.0), -1.57): set(['h2','s']),
}

symbols = set(['la1','la2','ua','lb1','lb2','ub','w','e','n','s'])

f_edges1 = ((3.0,1.0), (4.0,1.0),(1.0,3.0), (1.0,4.0))
f_edges2 = ((5.0,1.0),(1.0,5.0))
f_edges3 = ((4.0,4.0),(5.0,5.0))
forbid_edges = [(e1,e2) for e1 in f_edges1 for e2 in f_edges1] + [(e1,e2) for e1 in f_edges2 for e2 in f_edges2] + [(e1,e2) for e1 in f_edges3 for e2 in f_edges3]


action_label={
    'load_a': (10, '1', set(['loada'])),
    'unload_a': (10, '1', set(['unloada'])),
    'load_b': (10, '1', set(['loadb'])),
    'unload_b': (10, '1', set(['unloadb'])),}

# -------------------- YoBot 1 model --------------------
Y1_init_pose = ((5.0, 6.0), -PI*0.5)
Y1_alpha = [1.0, 1.0] # alpha depends on the mode

Y1_motion = MotionFts(comb_nodes, symbols, 'Y1-workspace')
Y1_motion.set_initial(Y1_init_pose)
Y1_motion.add_edges(forbid_edges, Y1_alpha)

########### Y1 action ##########
Y1_action = ActionModel(action_label)
########### Y1 task ############
one_la = '(la1 && w) || (la2 && e)'
one_ua = '(ua && n)'
Y1_task = '[] <> ((%s && loada) && <> (%s && unloada)) && [] <> (h2 && s)' %(one_la, one_ua)
########### Y1 initialize ############
Y1_mode_alpha = {'normal': [1.0, 1.0], 'type-I': [0.1, 1.0], 'type-II': [0.1, 10.0]}
robot_model['Y1']=(Y1_motion, Y1_action, Y1_task, Y1_mode_alpha)


# -------------------- YoBot 2 model --------------------
Y2_init_pose = ((5.0, 4.0), -PI*0.5)
Y2_alpha = [1.0, 1.0] # alpha depends on the mode

Y2_motion = MotionFts(comb_nodes, symbols, 'Y2-workspace')
Y2_motion.set_initial(Y2_init_pose)
Y2_motion.add_edges(forbid_edges, Y2_alpha)

########### Y2 action ##########
Y2_action = ActionModel(action_label)
########### Y2 task ############
one_la = '(la1 && w) || (la2 && e)'
one_ua = '(ua && n)'
Y2_task = '[] <> ((%s && loada) && <> (%s && unloada)) && [] <> (h1 && s)' %(one_la, one_ua)
########### Y2 initialize ############
Y2_mode_alpha = {'normal': [1.0, 1.0], 'type-I': [0.1, 1.0], 'type-II': [0.1, 10.0]}
robot_model['Y2']=(Y2_motion, Y2_action, Y2_task, Y2_mode_alpha)


# -------------------- YoBot 3 model --------------------
Y3_init_pose = ((5.0, 5.0), -PI*0.5)
Y3_alpha = [1.0, 1.0] # alpha depends on the mode

Y3_motion = MotionFts(comb_nodes, symbols, 'Y3-workspace')
Y3_motion.set_initial(Y3_init_pose)
Y3_motion.add_edges(forbid_edges, Y3_alpha)

########### Y3 action ##########
Y3_action = ActionModel(action_label)
########### Y3 task ############
one_lb = '(lb1 && w) || (lb2 && e)'
one_ub = '(ub && n)'
Y3_task = '[] <> ((%s && loadb) && <> (%s && unloadb)) && [] <> (h1 && s)' %(one_lb, one_ub)
########### Y3 initialize ############
Y3_mode_alpha = {'normal': [1.0, 1.0], 'type-I': [0.1, 1.0], 'type-II': [0.1, 10.0]}
robot_model['Y3']=(Y3_motion, Y3_action, Y3_task, Y3_mode_alpha)


# -------------------- YoBot 4 model --------------------
Y4_init_pose = ((4.0, 5.0), -PI*0.5)
Y4_alpha = [1.0, 1.0] # alpha depends on the mode

Y4_motion = MotionFts(comb_nodes, symbols, 'Y4-workspace')
Y4_motion.set_initial(Y4_init_pose)
Y4_motion.add_edges(forbid_edges, Y4_alpha)

########### Y4 action ##########
Y4_action = ActionModel(action_label)
########### Y4 task ############
one_lb = '(lb1 && w) || (lb2 && e)'
one_ub = '(ub && n)'
Y4_task = '[] <> ((%s && loadb) && <> (%s && unloadb)) && [] <> (h2 && s)' %(one_lb, one_ub)
########### Y4 initialize ############
Y4_mode_alpha = {'normal': [1.0, 1.0], 'type-I': [0.1, 1.0], 'type-II': [0.1, 10.0]}
robot_model['Y4']=(Y4_motion, Y4_action, Y4_task, Y4_mode_alpha)




if __name__ == '__main__':
    # to run: python robot_model.py Y1
    ########
    if len(sys.argv) == 2:
        agent_name = str(sys.argv[1])
    ###############
    if agent_name in robot_model:
        ts, act, task, mode_alpha = robot_model[agent_name]
        full_model = MotActModel(ts, act)
        planner = ltl_planner(full_model, task, None)
        ####### initial plan synthesis
        planner.optimal(10)
    #######
    else:
        print('Agent not found in robot_model.py')
