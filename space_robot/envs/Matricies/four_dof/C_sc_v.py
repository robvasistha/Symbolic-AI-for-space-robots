import numpy as np 
import math
def C_sc_v (in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11):
#BASE_C_SC_V
#    C_SC_V = BASE_C_SC_V(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11)

#    This function was generated by the Symbolic Math Toolbox version 8.0.
	in1 = np.insert(in1,0,0)
	in2 = np.insert(in2,0,0)
	in3 = np.insert(in3,0,0)
	in4 = np.insert(in4,0,0)
	in5 = np.insert(in5,0,0)
	in6 = in6.T.flatten()
	in6 = np.insert(in6,0,0)
	in7 = in7.T.flatten()
	in7 = np.insert(in7,0,0)
	in8 = in8.T.flatten()
	in8 = np.insert(in8,0,0)
	in9 = np.insert(in9,0,0)
	in10 = np.insert(in10,0,0)
	in11 = in11.T.flatten()
	in11 = np.insert(in11,0,0)
#    09-May-2019 15:38:25

	b2_1 = in8[2]
	b2_2 = in8[7]
	b2_3 = in8[12]
	b3_1 = in8[3]
	b3_2 = in8[8]
	b3_3 = in8[13]
	b4_1 = in8[4]
	b4_2 = in8[9]
	b4_3 = in8[14]
	b5_1 = in8[5]
	b5_2 = in8[10]
	b5_3 = in8[15]
	m2 = in1[2 ]
	m3 = in1[3 ]
	m4 = in1[4 ]
	m5 = in1[5 ]
	phi1 = in2[1]
	phi2 = in2[2]
	phi3 = in2[3]
	phi_d1 = in3[1]
	phi_d2 = in3[2]
	phi_d3 = in3[3]
	s1_1 = in7[1]
	s1_2 = in7[6]
	s1_3 = in7[11]
	s2_1 = in7[2]
	s2_2 = in7[7]
	s2_3 = in7[12]
	s3_1 = in7[3]
	s3_2 = in7[8]
	s3_3 = in7[13]
	s4_1 = in7[4]
	s4_2 = in7[9]
	s4_3 = in7[14]
	theta1 = in4[1]
	theta2 = in4[2]
	theta3 = in4[3]
	theta4 = in4[4]
	t2 = np.cos(phi2)
	t3 = np.sin(phi2)
	t4 = np.sin(theta1)
	t5 = np.cos(theta1)
	t6 = np.sin(phi3)
	t7 = t3*t5
	t8 = np.cos(phi3)
	t9 = t2*t8
	t10 = t2*t4*t6
	t11 = -t7+t9+t10
	t12 = t3*t4
	t13 = t2*t5*t6
	t14 = t12+t13
	t15 = np.cos(theta2)
	t16 = np.sin(theta2)
	t17 = np.cos(theta3)
	t18 = t14*t15
	t19 = t2*t8*t16
	t20 = t18+t19
	t21 = np.sin(theta3)
	t22 = t14*t16
	t24 = t2*t8*t15
	t23 = t22-t24
	t25 = t7-t10
	t26 = b2_2*t14
	t27 = b3_3*t11
	t28 = b4_3*t11
	t29 = s2_2*t14
	t30 = s3_3*t11
	t31 = t17*t23
	t32 = t20*t21
	t33 = t31+t32
	t34 = b4_1*t33
	t35 = t17*t20
	t38 = t21*t23
	t36 = t35-t38
	t37 = b4_2*t36
	t39 = b3_1*t23
	t40 = b3_2*t20
	t41 = s3_1*t23
	t42 = s3_2*t20
	t43 = np.cos(theta4)
	t44 = np.sin(theta4)
	t45 = b2_3*t2*t8
	t46 = s1_3*t2*t8
	t47 = s2_3*t2*t8
	t48 = s1_2*t2*t6
	t49 = np.cos(phi1)
	t50 = np.sin(phi1)
	t51 = t6*t49
	t58 = t3*t8*t50
	t52 = t51-t58
	t53 = t8*t49
	t54 = t3*t6*t50
	t55 = t53+t54
	t56 = t5*t55
	t59 = t2*t4*t50
	t57 = t56-t59
	t60 = s1_2*t55
	t61 = t4*t55
	t62 = t2*t5*t50
	t63 = t61+t62
	t64 = b2_1*t63
	t65 = b2_2*t57
	t66 = s1_1*t2*t50
	t67 = t16*t57
	t68 = t15*t52
	t69 = t67+t68
	t70 = b3_1*t69
	t71 = t16*t52
	t73 = t15*t57
	t72 = t71-t73
	t74 = s2_1*t63
	t75 = s2_2*t57
	t76 = -t51+t58+t61+t62
	t77 = b3_3*t76
	t78 = t17*t69
	t79 = s3_1*t69
	t80 = t17*t72
	t81 = t21*t69
	t82 = t80+t81
	t85 = t21*t72
	t83 = t78-t85
	t84 = b4_1*t83
	t86 = b4_3*t76
	t87 = s3_3*t76
	t88 = phi_d2*t49
	t89 = phi_d3*t2*t50
	t90 = t88+t89
	t91 = t8*t50
	t99 = t3*t6*t49
	t92 = t91-t99
	t93 = t5*t92
	t94 = t2*t4*t49
	t95 = t93+t94
	t96 = t6*t50
	t97 = t3*t8*t49
	t98 = t96+t97
	t100 = t15*t95
	t107 = t16*t98
	t101 = t100-t107
	t102 = t16*t95
	t103 = t15*t98
	t104 = t102+t103
	t105 = t2*t5*t49
	t114 = t4*t92
	t106 = t96+t97+t105-t114
	t108 = t17*t101
	t110 = t21*t104
	t109 = t108-t110
	t111 = t17*t104
	t112 = t21*t101
	t113 = t111+t112
	t115 = t105-t114
	t116 = b3_1*t104
	t117 = b3_2*t101
	t118 = s1_2*t92
	t119 = b2_2*t95
	t120 = s2_2*t95
	t121 = s3_1*t104
	t122 = s3_2*t101
	t123 = b4_1*t113
	t124 = b4_2*t109
	t125 = b2_3*t98
	t126 = s1_3*t98
	t127 = s2_3*t98
	t128 = b2_1*t115
	t129 = s2_1*t115
	t130 = b3_3*t106
	t131 = s1_1*t2*t49
	t132 = phi_d2*t50
	t183 = phi_d3*t2*t49
	t133 = t132-t183
	t134 = t43*t106
	t196 = t44*t109
	t135 = t134-t196
	t136 = t44*t106
	t137 = t43*t109
	t138 = t136+t137
	t139 = b5_2*t138
	t140 = s4_1*t113
	t141 = s4_2*t109
	t142 = t96+t97+t105-t111-t112-t114
	t143 = b5_3*t142
	t144 = -t116-t117-t118-t119-t120+t125+t126+t127+t128+t129+t130+t131
	t197 = b4_3*t106
	t198 = s3_3*t106
	t145 = t116+t117+t118+t119+t120+t121+t122+t123+t124-t125-t126-t127-t128-t129-t130-t131-t197-t198
	t146 = m4*t145
	t147 = -t118-t119+t125+t126+t128+t131
	t150 = b2_3*t52
	t151 = s1_3*t52
	t148 = t60+t64+t65+t66-t150-t151
	t149 = m2*t148
	t152 = t43*t76
	t191 = t44*t82
	t153 = t152-t191
	t154 = b5_1*t153
	t155 = t44*t76
	t156 = t43*t82
	t157 = t155+t156
	t158 = s4_1*t83
	t159 = s4_3*t76
	t162 = b2_1*t25
	t163 = s2_1*t25
	t167 = s1_1*t3
	t160 = t26+t27+t28+t29+t30+t34+t37+t39+t40+t41+t42+t45+t46+t47+t48-t162-t163-t167
	t161 = m4*t160
	t164 = s4_3*t11
	t165 = s4_1*t33
	t166 = s4_2*t36
	t168 = -t7+t9+t10+t31+t32
	t169 = t36*t44
	t170 = t11*t43
	t171 = t169+t170
	t172 = b5_1*t171
	t173 = t11*t44
	t175 = t36*t43
	t174 = t173-t175
	t184 = b5_3*t168
	t185 = b5_2*t174
	t176 = t26+t27+t28+t29+t30+t34+t37+t39+t40+t41+t42+t45+t46+t47+t48-t162-t163+t164+t165+t166-t167+t172-t184-t185
	t177 = m5*t176
	t178 = t26+t27+t29+t39+t40+t45+t46+t47+t48-t162-t163-t167
	t179 = m3*t178
	t180 = t26+t45+t46+t48-t162-t167
	t181 = m2*t180
	t182 = t161+t177+t179+t181
	t188 = b3_2*t72
	t189 = s2_3*t52
	t186 = t60+t64+t65+t66+t70+t74+t75+t77-t150-t151-t188-t189
	t187 = m3*t186
	t190 = -t51+t58+t61+t62+t78-t85
	t194 = s3_2*t72
	t195 = b4_2*t82
	t192 = t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87-t150-t151-t188-t189-t194-t195
	t193 = m4*t192
	t206 = b5_1*t135
	t207 = s4_3*t106
	t199 = t116+t117+t118+t119+t120+t121+t122+t123+t124-t125-t126-t127-t128-t129-t130-t131+t139+t140+t141+t143-t197-t198-t206-t207
	t200 = m5*t199
	t208 = m3*t144
	t209 = m2*t147
	t201 = t146+t200-t208-t209
	t210 = b5_3*t190
	t211 = b5_2*t157
	t212 = s4_2*t82
	t202 = t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87-t150-t151+t154+t158+t159-t188-t189-t194-t195-t210-t211-t212
	t203 = m5*t202
	t204 = t149+t187+t193+t203
	t216 = phi_d3*t3
	t205 = phi_d1-t216
	t213 = phi_d1*t2*t50
	t214 = phi_d2*t3*t49
	t215 = t213+t214
	t217 = t133*t182
	t218 = t201*t205
	t219 = t217+t218
	t220 = phi_d1*t2*t49
	t221 = t220-phi_d2*t3*t50
	t222 = t218-t90*t204
	C_sc_v = np.reshape([-t133*(t161+m3*(t26+t27+t29+t39+t40+t45+t46+t47+t48-b2_1*t25-s1_1*t3-s2_1*t25)+m2*(t26+t45+t46+t48-b2_1*t25-s1_1*t3)+m5*(t26+t27+t28+t29+t30+t34+t37+t39+t40+t41+t42+t45+t46+t47+t48+t164+t165+t166+t172-b2_1*t25-b5_3*t168-b5_2*t174-s1_1*t3-s2_1*t25))+t90*(t149+m4*(t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87-b2_3*t52-b3_2*t72-b4_2*t82-s1_3*t52-s2_3*t52-s3_2*t72)+m3*(t60+t64+t65+t66+t70+t74+t75+t77-b2_3*t52-b3_2*t72-s1_3*t52-s2_3*t52)+m5*(t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87+t154+t158+t159-b5_3*(-t51+t58+t61+t62+t78-t21*t72)-b2_3*t52-b3_2*t72-b4_2*t82-b5_2*t157-s1_3*t52-s2_3*t52-s3_2*t72-s4_2*t82)),-t204*t205,-t182*t205,phi_d1*t49*(t149+t187+m4*(t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87-t150-t151-b3_2*t72-b4_2*t82-s2_3*t52-s3_2*t72)+m5*(t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87-t150-t151+t154+t158+t159-b3_2*t72-b4_2*t82-b5_2*t157-b5_3*t190-s2_3*t52-s3_2*t72-s4_2*t82))+t50*t133*(t146-m3*t144-m2*t147+m5*(t116+t117+t118+t119+t120+t121+t122+t123+t124-t125-t126-t127-t128-t129-t130-t131+t139+t140+t141+t143-b4_3*t106-b5_1*t135-s3_3*t106-s4_3*t106))-phi_d1*t50*t182+t49*t90*(t146+m5*(t116+t117+t118+t119+t120+t121+t122+t123+t124+t139+t140+t141+t143-b2_3*t98-b2_1*t115-b3_3*t106-b4_3*t106-b5_1*t135-s1_3*t98-s2_3*t98-s2_1*t115-s3_3*t106-s4_3*t106-s1_1*t2*t49)-m3*t144-m2*t147),-t49*t219+phi_d1*t49*t201-t50*t133*t204,t50*t222-phi_d1*t50*t201-t49*t90*t182,t182*t221+t204*t215-t3*(t90*(t149+t187+t193+m5*(t60+t64+t65+t66+t70+t74+t75+t77+t79+t84+t86+t87-t150-t151+t154+t158+t159-t188-t189-b4_2*t82-b5_2*t157-b5_3*t190-s3_2*t72-s4_2*t82))-t133*t182)+t2*t50*t90*t201-t2*t49*t133*t201,t201*t215+phi_d2*t2*t182-t2*t50*t219+t3*t204*t205+t2*t49*t133*t204,t201*t221-phi_d2*t2*t204-t2*t49*t222+t3*t182*t205-t2*t50*t90*t182],(3,3)).T 
	return 	C_sc_v 