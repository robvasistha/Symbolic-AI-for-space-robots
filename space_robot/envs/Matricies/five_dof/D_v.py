import numpy as np 
import math
def D_v (in1):
#INERTIA_D_V
#    D_V = INERTIA_D_V(IN1,IN2,IN3,IN4,IN5,IN6)

#    This function was generated by the Symbolic Math Toolbox version 8.2.
	in1 = np.insert(in1,0,0)
#    10-Jan-2019 22:43:20

	m1 = in1[1 ]
	m2 = in1[2 ]
	m3 = in1[3 ]
	m4 = in1[4 ]
	m5 = in1[5 ]
	m6 = in1[6 ]
	t2 = m1+m2+m3+m4+m5+m6
	D_v = np.reshape([t2,0.0,0.0,0.0,t2,0.0,0.0,0.0,t2],(3,3)).T 
	return 	D_v 