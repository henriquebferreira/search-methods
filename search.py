# -*- coding: utf-8 -*-
"""
General search algorithm
"""

from problem import MyNode
import problem as pp

def search_alg(problem, search):
    itera=0
    node=MyNode(problem.InitialState)
    open_list=[node] #list of nodes
    closed_list=[] #list of nodes
    
    while True:
        itera+=1
        if open_list==[]:
            return pp.solution(False)
        node = open_list.pop() # pop node from open_list
        if problem.goaltest(node.state):
            return pp.solution(node.path,node.cost,itera)
        closed_list.append(node) #add current node to explored list
        
        # action is a list of components that can be launched        
        for action in problem.action_func(node.state):
            child = problem.childnode(node, action, search)
            existent = pp.check_list(child.state, open_list)
            aux=pp.check_list(child.state, closed_list)
            if not existent and not aux:
                open_list.append(child) # push into open list
            elif existent:
                if child.cost < existent.cost:
                    open_list.remove(existent)
                    open_list.append(child)
        # order open list by evaluation function f=g+h
        open_list=sorted(open_list, key=lambda MyNode: MyNode.f, reverse=True)