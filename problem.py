# -*- coding: utf-8 -*-

from datetime import date

'''
FUNCTIONS (auxiliar) - simplify problem 
'''

def check_launchable(comp_list, launch):
    total_weight=0
    for comp in comp_list:
        total_weight+=comp.w
    if total_weight>launch.pl:
        return False
    elif total_weight== launch.pl:
        return -1
    else:
        return True
        
def launch_cost(comp_list,launch):
    total_cost=0
    for comp in comp_list:
        total_cost+=comp.w*launch.costv
    if total_cost !=0:
        total_cost+=launch.costf
    return total_cost

def check_list(state, node_list):
    for node in node_list:
        if sorted(node.state.comp, key=lambda x: x.id) == sorted(state.comp, key=lambda x: x.id) and node.state.l==state.l: #they have the same comp
            return node
    return False
    
def solution(path, cost=0, i=0):
    if path == False:
        
        print('\t There is no solution for the given problem.')
        return
    else:
        for launch in path:
            if launch[1] == []: # no component sent
                continue
            print('At launch: ',launch[0],'\t send: ', launch[1])
    print('\t Total launch cost:',cost,'\t iteration:',i)


'''
CLASSES (auxiliar) - read from file
    component, edge, launch
'''

class MyComponent:
    def __init__(self,id1='none',w=0):
        self.id = id1
        self.w = float(w)
    def __repr__(self):
        return 'Comp:{}({})'.format(self.id, self.w)
    def view(self):
        print('{}: ID={}, Weight={}'.format(self.__class__.__name__, self.id, self.w))
        
class MyConnect:
    def __init__(self,id1='none',id2='none'):
        self.id1 = id1
        self.id2 = id2      
    def __repr__(self):
        return 'Con:{}-{}'.format(self.id1,self.id2)
    def view(self):
        print('{}: ID1={}, ID2={}'.format(self.__class__.__name__, self.id1, self.id2))
    
class MyLaunch:
    def __init__(self,datee='310122000', payload=0, CostFixed=0, CostVar=0):
        self.date=date(year=int(datee[4:8]), month=int(datee[2:4]), day=int(datee[0:2]))
        self.pl=float(payload)
        self.costf=float(CostFixed)
        self.costv=float(CostVar)
    def __repr__(self):
        return 'Lau:{}[{}] ({}+{})'.format(self.date, self.pl, self.costf, self.costv)
    def view(self):
        print('{}: Date={}, Payload={}, Costs=(Fix={}, Var={})'.format(
            self.__class__.__name__,self.date,self.pl,self.costf,self.costv))
    def __eq__(self, other):
        if not bool(self) and not bool(other): # if both launches are set as false
            return True
        elif not bool(self) or not bool(other): #if only one launch is false
            return False
        else:
            return self.date==other.date
            
'''
CLASSES (main) - necessary for general search
    state, node, problem
'''

class MyState:
    def __init__(self,lcomp=[],nlaun=[]):
        '''components in space + next launch'''
        self.comp=lcomp
        self.l=nlaun
    def __repr__(self):
        return 'Stt:{}({})'.format(self.comp, self.l)
    def view(self):
        print('{}: Comp={}, Launch={}'.format(self.__class__.__name__, self.comp, self.l))

    def __eq__(self, other):
        return self.comp==other.comp and self.l==other.l

class MyNode:
    def __init__(self,state=MyState(),g=0,path=[],h=0):
        self.state=state
        self.path=path
        self.cost=g
        self.heur=h
        self.f=g+h
        
    def __repr__(self):
        return '{}: State={}, Cost={}, Path={}, f={}'.format(self.__class__.__name__,
                                          self.state, self.cost,self.path,self.f)
    def __lt__(self, other): 
        return self.f < other.f #order nodes by value of f function
        
class MyProblem:
    def __init__(self,Components=[],Edges=[],Launches=[],GoalState=[],InitialState=[]):
        self.InitialState=InitialState
        self.GoalState=GoalState
        self.CompL=Components
        self.EdgeL=Edges
        self.LaunL=Launches
    
    def __repr__(self):
        return '{}:\n Components... {}\n Edges... {}\n Launches... {}\n   InitialState={}\n   GoalState={}\n'.format(self.__class__.__name__,
                                          self.CompL, self.EdgeL, self.LaunL, self.InitialState, self.GoalState)    
    
    def goaltest(self,state):
        if self.GoalState.comp==state.comp: #states must be sorted
            return True
        else:
            return False
    
    # Reads information from file and stores in problem lists
    def read_file(self, txt): 
        file = open(txt,'r')  # Opens the .txt file with all the information
        l1 = file.readlines()  # Reads all lines from file to a list, one line per entry
        V_list = list()  # Initiallize empty lists
        E_list = list()  # Initiallize empty lists
        L_list = list()  # Initiallize empty lists
        
        for n in l1:  # Groups the elements of l1 into 3 categories: V, E and L
            s1 = str(n)
            aux = s1.split()
            if(s1[0]=='V'):
                V_list.append(MyComponent(aux[0],aux[1]))
            elif(s1[0]=='E'):
                E_list.append(MyConnect(aux[1],aux[2]))
            elif(s1[0]=='L'):
                L_list.append(MyLaunch(aux[1],aux[2],aux[3],aux[4]))
                L_list.sort(key=lambda x: x.date)
                
        self.CompL=V_list
        self.EdgeL=E_list
        self.LaunL=L_list
    
    # Receives one component and returns a list of components directly conected to it
    def adjacent(self, comp):
        adj_list=[] # list with adjacent components (to be returned)
        if comp==[]: # no componnets in space
            for comp in self.CompL:
                adj_list.append(comp)
            return adj_list
        for edge in self.EdgeL:
            if comp.id==edge.id1: # check if component is on this edge
                aux=edge.id2
            elif comp.id==edge.id2:
                aux=edge.id1
            else: continue
            for comp_adj in self.CompL: # identidy conected component in component list
                if aux==comp_adj.id:
                    break
            adj_list.append(comp_adj)
        return adj_list
    
    # Recieves the current state and returns all possible actions (respecting problem constrains)
    def action_func(self, state):
        if not state.l[0]: # if there are no more launches
            return []
        actions=[[]] # send nothing is always an option
        act_imp = list()  # Initiallize empty lists
        act_unk = list()
        act_aux = list()
        if state.comp != []: # if we have components in space we should mark them as impossible to re-launch
            for comp_x in state.comp:
                act_imp.append([comp_x])
        
        if state.comp==[]: # calculate adjacent components when nothing is in space
            act_aux=self.adjacent(state.comp)
        else: # calculate adjacent components to each component already in space
            for comp in state.comp:
                act_aux+=self.adjacent(comp)
                
        for action in act_aux: # save each adjacent component as a list of one component
            act_unk.append([action])
        
        while act_unk!=[]: # do until all possible possible actions are explored
            action = act_unk.pop(0)
            mask=[set(action).issuperset(aa) for aa in act_imp]
            if True in mask: # if action contains a group of imp components, do nothing
                pass
            elif not check_launchable(action,state.l[0]): # if action is too heavy for launch add to impossible list
                act_imp.append(action)
            else: # if action is possible add to actions list and add adjacent nodes to actions unknown
                actions.append(action)
                for comp in action+state.comp: # calculate adjacent components for all components to be launched AND already in space
                    sub_act = self.adjacent(comp)
                    for sub in sub_act:
                        if not sub in action: # if adjacent isn't already set to be launched 
                            action_sub=action+[sub]
                            mask=[sorted(action_sub, key=lambda x: x.id) == sorted(aa, key=lambda x: x.id) for aa in actions+act_unk]
                            if not True in mask: # check if the same group of components was already set to be launched before (in another order)
                                act_unk.append(action_sub)
        return actions
    
    # Recieves the current state and estimates consistent value to achieve goal
    def heuristic(self, state):
        heur_cost=0 # variable to be returned    

        remain_comp = [item for item in self.CompL if item not in state.comp] # list of components still to be launched
        remain_w = sum(comp.w for comp in remain_comp) # total weight of all components to be launched    
        
        if state.l[0]==False: # if there are no more launches remaining
            if remain_comp == []:
                return 0
            else:
                return 1995 # act as a flag to be sent when it is impossible to achieve goal
        
        mask=[state.l[0]==launch for launch in self.LaunL] # compare launch with launch list and flag index
        remain_laun=self.LaunL[mask.index(True):] # create list with remaining launcehs
        
        if remain_w > sum(l.pl for l in remain_laun): # if there is more weight to be launched than total available payload
            return 1995
        
        min_cost_list = sorted(remain_laun, key= lambda x: (x.costf+(x.costv*x.pl))/x.pl) # order by specific cost (total cost per weight)
    
        while remain_w > 0:
            launch=min_cost_list.pop(0)
            sp_cost = (launch.costf+(launch.costv*launch.pl))/launch.pl # specific cost
            if remain_w <= launch.pl: # if all components can be send in this launch
                heur_cost += sp_cost*remain_w
                remain_w = 0
            else:
                heur_cost += sp_cost*launch.pl
                remain_w -= launch.pl # subtract launched payload from total weight to be launcehd
        return heur_cost
    
    # unused alternative heuristic
    def heuristic2(self, state):
        heur_cost=0 # variable to be returned    
        
        remain_comp = [item for item in self.CompL if item not in state.comp] # list of components still to be launched
        remain_w = sum(comp.w for comp in remain_comp) # total weight of all components to be launched    
        
        if state.l[0]==False: # if there are no more launches remaining
            if remain_comp == []:
                return 0
            else:
                return 1995 # define a flag to be sent when it is impossible to achieve goal
        
        mask=[state.l[0]==launch for launch in self.LaunL] # compare launch with launch list and flag index
        remain_laun=self.LaunL[mask.index(True):]
        
        if remain_w > sum(l.pl for l in remain_laun): # if there is more weight to be launched than total available payload
            return 1995
            
        min_costv_list = sorted(remain_laun, key= lambda x: x.costv) # order remaining launches by variable cost
        min_costf_list = sorted(remain_laun, key= lambda x: x.costf/x.pl) # order by fixed cost per weight
    
        aux_w = remain_w
        while remain_w > 0:
            launchV = min_costv_list.pop(0) # pop launch with lowest variable cost
            if remain_w <= launchV.pl: # if all components can be send in this launch
                heur_cost += remain_w*launchV.costv # variable cost of sending remaining comp
                remain_w = 0
            else:
                heur_cost += launchV.pl*launchV.costv # variable cost of sending maximung payload
                remain_w -= launchV.pl # subtract launched payload from total weight to be launcehd
        
        remain_w=aux_w
        while remain_w > 0:
            launchF = min_costf_list.pop(0) # pop launch with lowest fix cost
            if remain_w <= launchF.pl: # if all components can be send in this launch
                heur_cost += remain_w/launchF.pl*launchF.costf
                remain_w = 0
            else:
                heur_cost += launchF.costf # fix cost of using one launch
                remain_w -= launchF.pl
        return heur_cost
    
    # Receives a node, a action and a search method and computes the child node
    def childnode(self,node,action,search):
        nextstateC = node.state.comp + action # next state comp = comp already in space + comp launched
        nextstateC.sort(key= lambda x: x.id) # always store state with sorted components
        mask=[node.state.l[0]==launch for launch in self.LaunL]
        if mask[-1]: # if next launch is the last define next launch as False
            nextstateL=[False]
        else: # if there are more launches set next launch as the next in time
            nextstateL=[self.LaunL[mask.index(True)+1]]
        nextState = MyState(nextstateC,nextstateL)
        nextpath = node.path+[[node.state.l[0],action]] # update path with launch used and what was sent
        nextcost = node.cost + launch_cost(action,node.state.l[0])
        if search == '-i':
            next_h = self.heuristic(nextState)
        elif search == '-i2':
            next_h = self.heuristic2(nextState)
        else:
            next_h = 0
        NextNode = MyNode(nextState, nextcost, nextpath, next_h)
        return NextNode