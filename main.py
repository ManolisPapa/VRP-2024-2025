#from TSP_Model import Model
from Solver_for_tn_per_km import *

m = Model()
m.BuildModel('Instance.txt')
print(1)

s = Solver(m)
worst = s.worstSolution()
MnM_sol = s.solve()
SolDrawer.draw('Worst', worst, m.allNodes)
SolDrawer.draw('MnM', MnM_sol, m.allNodes)
print(1)

