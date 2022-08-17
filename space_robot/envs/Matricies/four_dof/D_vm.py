import numpy as np 
import math
def D_vm (in1,in2,in3,in4,in5,in6,in7,in8,in9):
#INERTIA_D_VM
#    D_VM = INERTIA_D_VM(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9)

#    This function was generated by the Symbolic Math Toolbox version 8.0.
	in1 = np.insert(in1,0,0)
	in2 = np.insert(in2,0,0)
	in3 = np.insert(in3,0,0)
	in4 = in4.T.flatten()
	in4 = np.insert(in4,0,0)
	in5 = in5.T.flatten()
	in5 = np.insert(in5,0,0)
	in6 = in6.T.flatten()
	in6 = np.insert(in6,0,0)
	in7 = np.insert(in7,0,0)
	in8 = np.insert(in8,0,0)
	in9 = in9.T.flatten()
	in9 = np.insert(in9,0,0)
#    09-May-2019 11:24:25

	b2_1 = in6[2]
	b2_2 = in6[7]
	b3_1 = in6[3]
	b3_2 = in6[8]
	b3_3 = in6[13]
	b4_1 = in6[4]
	b4_2 = in6[9]
	b4_3 = in6[14]
	b5_1 = in6[5]
	b5_2 = in6[10]
	b5_3 = in6[15]
	m2 = in1[2 ]
	m3 = in1[3 ]
	m4 = in1[4 ]
	m5 = in1[5 ]
	s2_1 = in5[2]
	s2_2 = in5[7]
	s3_1 = in5[3]
	s3_2 = in5[8]
	s3_3 = in5[13]
	s4_1 = in5[4]
	s4_2 = in5[9]
	s4_3 = in5[14]
	theta1 = in3[1]
	theta2 = in3[2]
	theta3 = in3[3]
	theta4 = in3[4]
	t2 = np.sin(theta1)
	t3 = np.cos(theta1)
	t4 = np.cos(theta2)
	t5 = np.sin(theta2)
	t6 = np.sin(theta3)
	t7 = np.cos(theta3)
	t8 = t3*t4*t6
	t9 = t3*t5*t7
	t10 = t8+t9
	t11 = t3*t4*t7
	t12 = np.sin(theta4)
	t13 = np.cos(theta4)
	t16 = t3*t5*t6
	t14 = t11-t16
	t15 = b4_1*t10
	t17 = b4_3*t2
	t18 = s3_3*t2
	t19 = s3_2*t3*t4
	t20 = s3_1*t3*t5
	t21 = b4_2*t14
	t22 = s4_1*t10
	t23 = s4_2*t14
	t24 = t2*t4*t7
	t40 = t2*t5*t6
	t25 = t24-t40
	t26 = t2*t4*t6
	t27 = t2*t5*t7
	t28 = t26+t27
	t29 = s4_3*t2
	t30 = t2*t13
	t31 = t12*t14
	t32 = t30+t31
	t33 = b5_1*t32
	t34 = t2*t12
	t41 = t13*t14
	t35 = t34-t41
	t36 = t2+t8+t9
	t37 = b3_3*t2
	t38 = b3_2*t3*t4
	t39 = b3_1*t3*t5
	t42 = b5_2*t35
	t43 = b5_3*t36
	t44 = b5_2*t13*t28*2.0
	t45 = b5_1*t12*t28*2.0
	t46 = t3*t13
	t58 = t12*t25
	t47 = t46-t58
	t48 = t3*t12
	t49 = t13*(t24-t40)
	t50 = t48+t49
	t51 = b4_1*t28
	t52 = s3_2*t2*t4
	t53 = s3_1*t2*t5
	t54 = b4_2*t25
	t55 = -t3+t26+t27
	t56 = b5_3*t55
	t57 = s4_3*t3
	t59 = b5_1*t47
	t60 = b3_2*t2*t4
	t61 = b3_1*t2*t5
	t62 = s4_1*t28
	t63 = b5_2*t50
	t64 = s4_2*t25
	t65 = t12*(t11-t16)
	t66 = t30+t65
	t67 = b5_2*t10*t13*2.0
	t68 = b5_1*t10*t12*2.0
	t69 = b5_3*t14*2.0
	t70 = t4*t6
	t71 = t5*t7
	t72 = t70+t71
	t73 = t4*t7
	t75 = t5*t6
	t74 = t73-t75
	t76 = b5_2*t13*t74*2.0
	t77 = b5_1*t12*t74*2.0
	D_vm = np.reshape([-m3*(t37+t38+t39+s2_1*t2+s2_2*t3)-m5*(t22+t23+t29+t33-b5_2*t35-b5_3*t36)-m2*(b2_1*t2+b2_2*t3)-m4*(t15+t17+t18+t19+t20+t21),m5*(t56+t57+t59-b5_2*t50-s4_2*t25-s4_1*t28)-m4*(t51+t52+t53+t54-b4_3*t3-s3_3*t3)+m2*(b2_1*t3-b2_2*t2)-m3*(t60+t61-b3_3*t3-s2_1*t3+s2_2*t2),0.0,m5*(-t22-t23-t29-t33+t42+t43+b5_3*t25-s4_1*t25+s4_2*t28+b5_1*t12*t28+b5_2*t13*t28)-m4*(t15+t17+t18+t19+t20+t21+b4_1*t25-b4_2*t28+s3_1*t2*t4-s3_2*t2*t5)-m3*(t37+t38+t39+b3_1*t2*t4-b3_2*t2*t5),-m5*(-t56-t57-t59+t62+t63+t64+b5_3*t14+s4_2*t10-s4_1*t14+b5_1*t10*t12+b5_2*t10*t13)-m4*(t51+t52+t53+t54-b4_3*t3+b4_2*t10-b4_1*t14-s3_3*t3-s3_1*t3*t4+s3_2*t3*t5)-m3*(t60+t61-b3_3*t3-b3_1*t3*t4+b3_2*t3*t5),m4*(b4_1*t72+b4_2*t74+s3_1*t5+s3_2*t4)+m5*(-b5_3*t72+s4_1*t72+s4_2*t74+b5_1*t12*t74+b5_2*t13*t74)+m3*(b3_1*t5+b3_2*t4),m5*(-t22-t23-t29-t33+t42+t43+t44+t45-s4_1*t25*2.0+s4_2*t28*2.0+b5_3*(t24-t40)*2.0)-m4*(t15+t17+t21+b4_1*t25*2.0-b4_2*t28*2.0),-m4*(t51+t54-b4_3*t3+b4_2*t10*2.0-b4_1*t14*2.0)-m5*(-t56-t57-t59+t62+t63+t64+t67+t68+t69+s4_2*t10*2.0-s4_1*t14*2.0),m4*(b4_1*t72*2.0+b4_2*t74*2.0)+m5*(t76+t77-b5_3*t72*2.0+s4_1*t72*2.0+s4_2*t74*2.0),m5*(t42+t43+t44+t45+b5_3*t25*2.0-b5_2*t47-b5_1*t50-b5_1*t66),-m5*(-t56-t59+t63+t67+t68+t69+b5_1*t35+b5_2*t66),-m5*(-t76-t77+b5_3*t72*2.0+b5_1*(t12-t13*t72)+b5_2*(t13+t12*t72))],(4,3)).T 
	return 	D_vm 