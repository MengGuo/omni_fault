import sys

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


def combine_wps_angle(wps_label, angle_label):
    nodes_label = dict()
    symbols = set()
    for wp, wp_l in wps_label.iteritems():
        for angle, angle_l in angle_label.iteritems():
            node = (wp, angle)
            node_l = wp_l.union(angle_l)
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


# -------------------- Y1 model --------------------

Y1_wps_label = {
    (-2.0, -2.0): set(['y1r1']),
    (2.0, -2.0): set(['y1r2']),
    (0.0, 2.0): set(['y1r3']),    
}
Y1_init_pose = ((0.0, 2.0), -1.5)

Y1_label, Y1_symbols = combine_wps_angle(Y1_wps_label, angle_label)

Y1_motion = MotionFts(Y1_label, Y1_symbols, 'Y1-workspace')
Y1_motion.set_initial(Y1_init_pose)

Y1_edge=[((-2.0, -2.0), (2.0, -2.0)),
         ((2.0, -2.0), (0.0, 2.0)),
         ((0.0, 2.0), (-2.0, -2.0))]
Y1_arg_edge = [((e[0],af), (e[1],at)) for e in Y1_edge for af in angle_label.keys() for at in angle_label.keys()]
Y1_motion.add_un_edges(Y1_arg_edge, alpha=1.0) # alpha depends on the mode
########### Y1 action ##########
Y1_action_label={
             'y1act': (10, '1', set(['y1act'])),
            }
Y1_action = ActionModel(Y1_action_label)
########### Y1 task ############
Y1_task = '(<> (y1r1 && y1act)) && ([]<> (y1r2 && e)) && ([]<> (y1r3 && w))'
########### Y1 initialize ############
robot_model['Y1']=(Y1_motion, Y1_action, Y1_task)




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
