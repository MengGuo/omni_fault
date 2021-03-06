# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history


class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec):
                print '===================='
                print 'planner initialized'
		buchi = mission_to_buchi(hard_spec, soft_spec)
                self.hard_spec = hard_spec
                self.ts = ts
		self.product = ProdAut(ts, buchi)
		self.Time = 0
		self.cur_pose = None
		self.trace = [] # record the regions been visited
		self.traj = [] # record the full trajectory
		self.opt_log = [] 
		# record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
		self.com_log = []
		# record [(time, no_messages)]

	def optimal(self, beta=10, style='static'):
		self.beta = beta
		if style == 'static':
			# full graph construction
			self.product.graph['ts'].build_full()
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'ready':
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'on-the-fly':
			# on-the-fly construction
			self.product.build_initial()
			self.product.build_accept()
			self.run, plantime = dijkstra_plan_optimal(self.product, self.beta)
                if self.run == None:
                        print '---No valid has been found!---'
                        print '---Check you FTS or task---'
                        return 
		#print '\n'
                print '------------------------------'
                print 'the prefix of plan **sequence of states**:'
		print [n for n in self.run.line]
                print 'the suffix of plan **sequence of states**:'
		print [n for n in self.run.loop]
		#print '\n'
                print '------------------------------'
		print 'the prefix of plan **sequence of actions**:'
		print [n for n in self.run.pre_plan]
		print 'the suffix of plan **sequence of actions**:'
		print [n for n in self.run.suf_plan]
		self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
		self.last_time = self.Time
		self.acc_change = 0
		self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
		return plantime

	def find_next_move(self):
		if self.segment == 'line' and self.index < len(self.run.pre_plan)-2:
			self.trace.append(self.run.line[self.index])
			self.index += 1
			self.next_move = self.run.pre_plan[self.index]
		elif (self.segment == 'line') and ((self.index == len(self.run.pre_plan)-2) or (len(self.run.pre_plan) <= 2)):
			self.trace.append(self.run.line[self.index])
			self.index = 0
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-2:
			self.trace.append(self.run.loop[self.index])
			self.index += 1
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif (self.segment == 'loop') and ((self.index == len(self.run.suf_plan)-2) or (len(self.run.suf_plan) <= 2)):
			self.trace.append(self.run.loop[self.index])
			self.index = 0
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		return self.next_move


	def update(self,object_name):
		MotionFts = self.product.graph['ts'].graph['region']
		cur_region = MotionFts.closest_node(self.cur_pose)
		sense_info = dict()
		sense_info['label'] = set([(cur_region,set([object_name,]),set()),]) 
		changes = MotionFts.update_after_region_change(sense_info,None)
		if changes:
			return True
                        
        def update_model_by_alpha(self, new_alpha):
                self.product.graph['ts'].graph['region'].update_by_alpha(new_alpha)
                self.product.graph['ts'].graph['region'].set_initial(self.cur_pose)
                
	def replan(self):
		new_run = improve_plan_given_history(self.product, self.trace)
		if (new_run) and (new_run.pre_plan !=self.run.pre_plan[self.index:-1]):
			self.run = new_run
			self.index = 0
			self.segment = 'line'
			self.next_move = self.run.pre_plan[self.index]
			print 'Plan adapted!'

	def replan_simple(self):
		#self.product.graph['ts'].graph['region'].set_initial(self.cur_pose)
		self.optimal(10)

        def evaluate_request(self, task):
                test_buchi = mission_to_buchi(self.hard_spec, task)
                test_product = ProdAut(self.ts, test_buchi)
                test_product.build_full()
                test_run, test_time = dijkstra_plan_networkX(test_product, self.beta)
                self.test_run = test_run
                test_edges = test_run.prod_run_to_prod_edges(test_product)
                total_c = 0
                total_d = 0
                for e in test_run.pre_prod_edges:
                        total_c += test_product.edge[e[0]][e[1]]['c_weight']
                        total_d += test_product.edge[e[0]][e[1]]['d_weight']
                for e in test_run.suf_prod_edges:
                        total_c += self.beta*test_product.edge[e[0]][e[1]]['c_weight']
                        total_d += self.beta*test_product.edge[e[0]][e[1]]['d_weight']
                return total_c, total_d

        def confirm_request(self, task):
                self.__init__(self.ts, self.hard_spec, task)
                self.product.build_full()
                #self.optimal(10)
                self.run = self.test_run
                self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
                print 'New task incorporated!'









