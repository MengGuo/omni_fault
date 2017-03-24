import sys

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def combine_wps_angle(wps_label, angle_label):
    nodes_label = dict()
    symbols = set()
    for wp, wp_l in wps_label.iteritems():
        for angle, angle_l in angle_label.iteritems():
            for obj, obj_l in obj_label.iteritems():
                node = (wp, angle, obj)
                node_l = wp_l.union(angle_l.union(obj_l))
                nodes_label[node] = node_l
                symbols.update(node_l)
    return nodes_label, symbols



# ========================================
# robot model description starts
robot_model=dict()

# angle discrete
# [-3.1+3.1415*k/4 for k in range(8)]
angle_label = {
    -3.1: set(['w',]),
    -2.3: set(['sw',]),
    -1.5: set(['s',]),
    -0.7: set(['se',]),
    0.04: set(['e',]),
    0.8: set(['ne',]),
    1.6: set(['n',]),
    2.3: set(['nw',]),        
}

# la = 'place to load object a'
# ua = 'place to unload object a'
# h = 'home base'
node_label = {
    (2.0, 1.0): set(['ub1',]),        
    (3.0, 1.0): set(['la1',]),
    (4.0, 1.0): set(['la2',]),
    (5.0, 1.0): set(['la3',]),
    (6.0, 1.0): set(['ub2',]),    
    (1.0, 2.0): set(['ua1',]),
    (1.0, 3.0): set(['lb1',]),
    (1.0, 4.0): set(['lb2',]),
    (1.0, 5.0): set(['lb3',]),
    (1.0, 2.0): set(['ua2',]),
    (5.0, 6.0): set(['h1',]),
    (6.0, 6.0): set(['h2',]),
    (5.0, 5.0): set(['h3',]),
    (6.0, 5.0): set(['h4',]),            
}

# nobj = 'no obj'
# obj = 'has obj'
obj_label = {
    0: set(['nobj',]),
    1: set(['obj',]),
}

comb_nodes, symbols = combine_wps_angle(node_label, angle_label, obj_label)

forbid_edges = None
# -------------------- YoBot 1 model --------------------

# pose = ((x,y), theta, obj)
Y1_init_pose = ((5.0, 6.0), -1.5, 0)
Y1_alpha = [1.0, 1.0] # alpha depends on the mode

Y1_motion = MotionFts(comb_nodes, symbols, 'Y1-workspace')
Y1_motion.set_initial(Y1_init_pose)
Y1_motion.add_edges(forbid_edges, Y1_alpha)

########### Y1 action ##########
Y1_action_label={
    'load_a': (10, 'nobj', set(['loada'])),
    'unload_a': (10, 'obj', set(['unloada'])),
    'load_b': (10, 'nobj', set(['loadb'])),
    'unload_b': (10, 'obj', set(['unloadb'])),    
            }
Y1_action = ActionModel(Y1_action_label)
########### Y1 task ############
one_la = 'la1 || la2 || la3'
one_ua = 'ua1 || ua2 || ua3'
Y1_task = '[] <> ((%s && loada) && <> (%s && unloada)) && [] <> h1' %(one_la, one_ua)
########### Y1 initialize ############
Y1_mode_alpha = {'normal': [1.0, 1.0], 'type-I': [0.1, 1.0], 'type-III': [0.1, 10.0]}
robot_model['Y1']=(Y1_motion, Y1_action, Y1_task, Y1_mode_alpha)




if __name__ == '__main__':
    # to run: python robot_model.py Y1
    ########
    if len(sys.argv) == 2:
        agent_name = str(sys.argv[1])
    ###############
    if agent_name in robot_model:
        ts, act, task = robot_model[agent_name]
        full_model = MotActModel(ts, act)
        planner = ltl_planner(full_model, task, None)
        ####### initial plan synthesis
        planner.optimal(10)
    #######
    else:
        print('Agent not found in robot_model.py')
