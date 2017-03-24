# -*- coding: utf-8 -*-

from boolean_formulas.parser import parse as parse_guard

from math import sqrt
from networkx.classes.digraph import DiGraph

def distance(node1, node2, alpha):
    pose1, ang1 = node1
    pose2, ang2 = node2
    wp_dif = (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)
    ang_dif = abs(ang1-ang2)
    return alpha[0]*wp_dif+alpha[1]*ang_dif

def reach_waypoint(pose, waypoint, margin):
    if distance(pose, waypoint)<=margin:
        return True
    else:
        return False

class MotionFts(DiGraph):
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        for (n, label) in node_dict.iteritems():
            self.add_node(n, label=label, status='confirmed')
        self.alpha = 1
            
    def add_un_edges(self, forbid_edges, alpha=[1.0,1.0]):
        self.alpha = alpha
        for f_node in self.nodes_iter():
            for t_node in self.nodes_iter():
                if (f_node, t_node) not in forbid_edges:
                    self.add_edge(f_node, t_node, weight=distance(f_node, t_node, alpha))
        for node in self.nodes_iter():
            self.add_edge(node, node, weight=0.01)

    def set_initial(self, state):
        init_node = self.closest_node(state)
        self.graph['initial'] = set([init_node])
        return init_node

    def closest_node(self, state):
        node = min(self.nodes_iter(), key= lambda n: distance(n,state,self.alpha))
        return node

    def update_by_alpha(self, new_alpha):
        for f_node,t_node in self.edges_iter():
            cost = distance(f_node, t_node, new_alpha)
            self.edge[f_node][t_node]['weight'] = cost
        print 'Region FTS edge cost updated due to new alpha'

        
class ActionModel(object):
    # action_dict = {act_name: (cost, guard_formula, label)}
    def __init__(self, action_dict):
        self.raw = action_dict
        self.action = dict()
        for act_name, attrib in action_dict.iteritems():
            cost = attrib[0]
            guard_formula = attrib[1]
            guard_expr = parse_guard(guard_formula)
            label = attrib[2]
            self.action[act_name] = (cost, guard_expr, label)
        self.action['None'] = (0, parse_guard('1'), set()) 

    def allowed_actions(self, ts_node_label):
        allow_action = set()
        for act_name, attrib in self.action.iteritems():
            if (attrib[1].check(ts_node_label)):
                allow_action.add(act_name)
        return allow_action



class MotActModel(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set(), type='MotActModel')

    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            new_label = self.graph['region'].node[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=reg, action=act, marker='unvisited')
            if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
                self.graph['initial'].add(prod_node)
        return prod_node

    def projection(self, prod_node):
        reg = self.node[prod_node]['region']
        act = self.node[prod_node]['action']
        return reg, act

    def build_initial(self):
        for reg_init in self.graph['region'].graph['initial']:
            init_prod_node = self.composition(reg_init, 'None')

    def build_full(self):
        for reg in self.graph['region'].nodes_iter():
            for act in self.graph['action'].action.iterkeys():
                prod_node = self.composition(reg, act)
                # actions 
                label = self.graph['region'].node[reg]['label']
                for act_to in self.graph['action'].allowed_actions(label):
                    prod_node_to = self.composition(reg, act_to)
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                # motions
                for reg_to in self.graph['region'].successors_iter(reg):
                    prod_node_to = self.composition(reg_to, 'None')
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto', marker= 'visited')
    
    def fly_successors_iter(self, prod_node): 
        reg, act = self.projection(prod_node)
        # been visited before, and hasn't changed 
        if ((self.node[prod_node]['marker'] == 'visited') and 
            (self.graph['region'].node[self.node[prod_node]['region']]['status'] == 'confirmed')):
            for prod_node_to in self.successors_iter(prod_node):
                yield prod_node_to, self.edge[prod_node][prod_node_to]['weight']
        else:
            self.remove_edges_from(self.out_edges(prod_node))
            # actions 
            label = self.graph['region'].node[reg]['label']
            for act_to in self.graph['action'].allowed_actions(label):
                prod_node_to = self.composition(reg, act_to)
                cost = self.graph['action'].action[act_to][0]
                self.add_edge(prod_node, prod_node_to, weight=cost, label= act_to)
                yield prod_node_to, cost
            # motions
            for reg_to in self.graph['region'].successors_iter(reg):
                if reg_to != reg:
                    prod_node_to = self.composition(reg_to, 'None')
                    cost = self.graph['region'][reg][reg_to]['weight']
                    self.add_edge(prod_node, prod_node_to, weight=cost, label= 'goto')         
                    yield prod_node_to, cost
            self.graph['region'].node[self.node[prod_node]['region']]['status'] = 'confirmed'
            self.node[prod_node]['marker'] = 'visited'

    def fly_predecessors_iter(self, prod_node): 
        reg, act = self.projection(prod_node)
        # actions
        label = self.graph['region'].node[reg]['label']
        if act in self.graph['action'].allowed_actions(label):    
            for f_act in self.graph['action'].action.iterkeys():
                f_prod_node = self.composition(reg, f_act)
                cost = self.graph['action'].action[act][0]
                self.add_edge(f_prod_node, prod_node, weight=cost, label= act)
                yield f_prod_node, cost
        # motions
        if act == 'None':
            for f_reg in self.graph['region'].predecessors_iter(reg):
                if f_reg !=reg:
                    for f_act in self.graph['action'].action.iterkeys():
                            f_prod_node = self.composition(f_reg, f_act)
                            cost = self.graph['region'][f_reg][reg]['weight']
                            self.add_edge(f_prod_node, prod_node, weight=cost, label= 'goto')         
                            yield f_prod_node, cost
