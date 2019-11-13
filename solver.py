# -*- coding: utf-8 -*-
"""
Solver program
"""

import sys # used for reading arguments len/str(sys.argv)

from problem import MyState
from problem import MyProblem
from search import search_alg

def read_arguments():
    search = str(sys.argv[1])
    txt = str(sys.argv[2])
    return search, txt

def ini_problem(txt):
    problem = MyProblem()
    problem.read_file(txt)
    problem.GoalState = MyState(sorted(problem.CompL, key= lambda x:x.id),[])
    problem.InitialState = MyState([],[problem.LaunL[0]])
    return problem

if __name__ == "__main__":
    search, txt = read_arguments()
    problem = ini_problem(txt)
    search_alg(problem, search)