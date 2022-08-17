import numpy as np 
import math
def C_wm (in1,in2,in3,in4,in5,in6,in7,in8,in9,in10):
#NONLINEAR_C_WM
#    C_WM = NONLINEAR_C_WM(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10)

#    This function was generated by the Symbolic Math Toolbox version 8.0.
	in1 = np.insert(in1,0,0)
	in2 = np.insert(in2,0,0)
	in3 = np.insert(in3,0,0)
	in4 = np.insert(in4,0,0)
	in5 = in5.T.flatten()
	in5 = np.insert(in5,0,0)
	in6 = in6.T.flatten()
	in6 = np.insert(in6,0,0)
	in7 = in7.T.flatten()
	in7 = np.insert(in7,0,0)
	in8 = np.insert(in8,0,0)
	in9 = np.insert(in9,0,0)
	in10 = in10.T.flatten()
	in10 = np.insert(in10,0,0)
#    14-May-2019 21:45:30

	b2_1 = in7[2]
	b2_2 = in7[9]
	b2_3 = in7[16]
	b3_1 = in7[3]
	b3_2 = in7[10]
	b3_3 = in7[17]
	b4_1 = in7[4]
	b4_2 = in7[11]
	b4_3 = in7[18]
	b5_1 = in7[5]
	b5_2 = in7[12]
	b5_3 = in7[19]
	m2 = in1[2 ]
	m3 = in1[3 ]
	m4 = in1[4 ]
	m5 = in1[5 ]
	phi1 = in2[1]
	phi2 = in2[2]
	phi3 = in2[3]
	s1_1 = in6[1]
	s1_2 = in6[8]
	s1_3 = in6[15]
	s2_1 = in6[2]
	s2_2 = in6[9]
	s2_3 = in6[16]
	s3_1 = in6[3]
	s3_2 = in6[10]
	s3_3 = in6[17]
	s4_1 = in6[4]
	s4_2 = in6[11]
	s4_3 = in6[18]
	theta1 = in3[1]
	theta2 = in3[2]
	theta3 = in3[3]
	theta4 = in3[4]
	theta_d1 = in4[1]
	theta_d2 = in4[2]
	theta_d3 = in4[3]
	theta_d4 = in4[4]
	t2 = np.sin(theta1)
	t3 = np.cos(theta2)
	t4 = np.sin(theta3)
	t5 = np.cos(theta3)
	t6 = np.sin(theta2)
	t7 = np.sin(phi2)
	t8 = np.cos(phi2)
	t9 = np.cos(theta1)
	t10 = np.sin(phi3)
	t11 = t7*t9
	t12 = np.cos(phi3)
	t13 = t8*t12
	t14 = t2*t8*t10
	t15 = -t11+t13+t14
	t16 = t2*t7
	t17 = t8*t9*t10
	t18 = t16+t17
	t19 = t3*t18
	t20 = t6*t8*t12
	t21 = t19+t20
	t22 = t6*t18
	t24 = t3*t8*t12
	t23 = t22-t24
	t25 = t2*t3*t5
	t32 = t2*t4*t6
	t26 = t25-t32
	t27 = t2*t3*t4
	t28 = t2*t5*t6
	t29 = t27+t28
	t30 = np.cos(theta4)
	t31 = np.sin(theta4)
	t33 = t11-t14
	t34 = b2_2*t18
	t35 = b3_3*t15
	t36 = b4_3*t15
	t37 = s2_2*t18
	t38 = s3_3*t15
	t39 = t5*t23
	t40 = t4*t21
	t41 = t39+t40
	t42 = t5*t21
	t44 = t4*t23
	t43 = t42-t44
	t45 = b4_1*t41
	t46 = b4_2*t43
	t47 = b3_1*t23
	t48 = b3_2*t21
	t49 = s3_1*t23
	t50 = s3_2*t21
	t51 = b2_3*t8*t12
	t52 = s1_3*t8*t12
	t53 = s2_3*t8*t12
	t54 = s1_2*t8*t10
	t55 = np.sin(phi1)
	t56 = np.cos(phi1)
	t57 = t12*t56
	t58 = t7*t10*t55
	t59 = t57+t58
	t60 = t9*t59
	t65 = t2*t8*t55
	t61 = t60-t65
	t62 = t10*t56
	t64 = t7*t12*t55
	t63 = t62-t64
	t66 = t2*t59
	t67 = t8*t9*t55
	t68 = t66+t67
	t69 = t3*t4
	t70 = t5*t6
	t71 = t69+t70
	t72 = t3*t5
	t96 = t4*t6
	t73 = t72-t96
	t74 = t6*t61
	t75 = t3*t63
	t76 = t74+t75
	t77 = b3_1*t76
	t78 = t3*t61
	t81 = t6*t63
	t79 = t78-t81
	t80 = b3_2*t79
	t82 = s1_2*t59
	t83 = t5*t76
	t84 = t4*t79
	t85 = t83+t84
	t86 = t4*t76
	t88 = t5*t79
	t87 = t86-t88
	t89 = b2_1*t68
	t90 = b2_2*t61
	t91 = s2_1*t68
	t92 = s2_2*t61
	t93 = -t62+t64+t66+t67
	t94 = b3_3*t93
	t95 = s1_1*t8*t55
	t97 = s3_1*t76
	t98 = s3_2*t79
	t99 = b4_1*t85
	t100 = b4_3*t93
	t101 = s3_3*t93
	t102 = b4_1*t26
	t103 = b3_1*t2*t3
	t117 = b2_1*t33
	t119 = s2_1*t33
	t129 = s1_1*t7
	t104 = t34+t35+t37+t47+t48+t51+t52+t53+t54-t117-t119-t129
	t105 = t3*t4*t9
	t106 = t5*t6*t9
	t107 = t105+t106
	t108 = t3*t5*t9
	t110 = t4*t6*t9
	t109 = t108-t110
	t111 = t26*t30
	t154 = t29*t31
	t112 = t111-t154
	t113 = b5_1*t112
	t114 = t29*t30
	t115 = t31*(t25-t32)
	t116 = t114+t115
	t118 = b5_3*t15
	t120 = s4_3*t15
	t121 = t30*t41
	t122 = t31*t43
	t123 = t121+t122
	t124 = b5_1*t123
	t125 = t31*t41
	t155 = t30*t43
	t126 = t125-t155
	t127 = s4_1*t41
	t128 = s4_2*t43
	t130 = t31*t71
	t157 = t30*t73
	t131 = t130-t157
	t132 = t30*t71
	t133 = t31*t73
	t134 = t132+t133
	t135 = s4_1*t85
	t136 = t30*t85
	t159 = t31*t87
	t137 = t136-t159
	t138 = b5_1*t137
	t139 = t30*t87
	t140 = t31*t85
	t141 = t139+t140
	t142 = b5_3*t93
	t143 = s4_3*t93
	t145 = b2_3*t63
	t146 = s1_3*t63
	t147 = s2_3*t63
	t148 = b4_2*t87
	t158 = s4_2*t87
	t160 = b5_2*t141
	t144 = t77+t80+t82+t89+t90+t91+t92+t94+t95+t97+t98+t99+t100+t101+t135+t138+t142+t143-t145-t146-t147-t148-t158-t160
	t149 = t30*t107
	t150 = t31*t109
	t151 = t149+t150
	t152 = t31*t107
	t161 = t30*t109
	t153 = t152-t161
	t162 = b5_2*t126
	t156 = t34+t35+t36+t37+t38+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54-t117+t118-t119+t120+t124+t127+t128-t129-t162
	t163 = t34+t35+t36+t37+t38+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54-t117-t119-t129
	t164 = b4_2*t71
	t165 = t12*t55
	t173 = t7*t10*t56
	t166 = t165-t173
	t167 = t9*t166
	t168 = t2*t8*t56
	t169 = t167+t168
	t170 = t10*t55
	t171 = t7*t12*t56
	t172 = t170+t171
	t174 = t6*t169
	t175 = t3*t172
	t176 = t174+t175
	t177 = t3*t169
	t179 = t6*t172
	t178 = t177-t179
	t180 = t2*t166
	t182 = t8*t9*t56
	t181 = t180-t182
	t183 = t170+t171-t180+t182
	t184 = t164-b4_1*t73
	t185 = b5_1*t131
	t186 = b5_2*t134
	t187 = b3_1*t176
	t188 = b3_2*t178
	t189 = s3_1*t176
	t190 = s3_2*t178
	t191 = t5*t176
	t192 = t4*t178
	t193 = t191+t192
	t194 = b4_1*t193
	t195 = t4*t176
	t197 = t5*t178
	t196 = t195-t197
	t198 = s1_2*t166
	t199 = b2_1*t181
	t200 = b2_2*t169
	t201 = s2_1*t181
	t202 = s2_2*t169
	t203 = t185+t186
	t204 = b4_2*t107
	t205 = b3_1*t3*t9
	t206 = b3_1*t3
	t207 = t206-b3_2*t6
	t208 = b5_1*t153
	t209 = t31*(t108-t110)
	t210 = t149+t209
	t211 = b5_2*t210
	t212 = b4_2*t71*2.0
	t217 = b4_2*t196
	t218 = b2_3*t172
	t220 = s1_3*t172
	t221 = s2_3*t172
	t228 = b3_3*t183
	t229 = b4_3*t183
	t230 = s3_3*t183
	t231 = s1_1*t8*t56
	t213 = t187+t188+t189+t190+t194+t198+t199+t200+t201+t202-t217-t218-t220-t221-t228-t229-t230-t231
	t214 = t212-b4_1*t73*2.0
	t215 = b5_1*t131*2.0
	t216 = b5_2*t134*2.0
	t219 = s4_1*t193
	t222 = t30*t193
	t235 = t31*t196
	t223 = t222-t235
	t224 = b5_1*t223
	t225 = t30*t196
	t226 = t31*t193
	t227 = t225+t226
	t232 = t215+t216
	t233 = b5_1*t131*3.0
	t234 = b5_2*t134*3.0
	t241 = s4_2*t196
	t242 = b5_2*t227
	t243 = b5_3*t183
	t244 = s4_3*t183
	t236 = t187+t188+t189+t190+t194+t198+t199+t200+t201+t202-t217-t218+t219-t220-t221+t224-t228-t229-t230-t231-t241-t242-t243-t244
	t237 = t233+t234
	t238 = b2_2*t2
	t239 = t238-b2_1*t9
	t259 = b3_2*t2*t6
	t240 = t103-t259
	t252 = b4_1*t109
	t245 = t204-t252
	t246 = t77+t80+t82+t89+t90+t91+t92+t94+t95+t97+t98+t99+t100+t101-t145-t146-t147-t148
	t247 = b3_2*t6*t9
	t248 = t208+t211
	t249 = b2_2*t9
	t250 = b2_1*t2
	t251 = t249+t250
	t253 = b4_1*t29
	t254 = b4_2*(t25-t32)
	t255 = b4_1*t107
	t256 = b4_2*t109
	t257 = b3_2*t3*t9
	t258 = b3_1*t6*t9
	t260 = t187+t188+t198+t199+t200+t201+t202-t218-t220-t221-t228-t231
	t261 = b5_1*t116
	t262 = b5_2*(t111-t154)
	t263 = t208+t211+t261+t262
	t264 = b3_2*t2*t3
	t265 = b3_1*t2*t6
	t266 = -t205+t247+t264+t265
	t267 = t77+t80+t82+t89+t90+t91+t92+t94+t95-t145-t146-t147
	t268 = b4_1*t29*2.0
	t269 = b4_2*(t25-t32)*2.0
	t270 = b4_1*t107*2.0
	t271 = b4_2*t109*2.0
	t272 = b5_1*t116*2.0
	t273 = b5_2*(t111-t154)*2.0
	t274 = t208+t211+t272+t273
	t275 = b5_1*t116*3.0
	t276 = b5_2*(t111-t154)*3.0
	t277 = t208+t211+t275+t276
	C_wm = np.reshape([m3*t104*t240*theta_d2+m4*theta_d3*(t102-b4_2*t29)*(t34+t35+t36+t37+t38+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54-s1_1*t7-s2_1*t33-b2_1*(t11-t2*t8*t10))+m2*t251*theta_d1*(t34+t51+t52+t54-b2_1*t33-s1_1*t7)+m5*theta_d4*(t113-b5_2*t116)*(t34+t35+t36+t37+t38+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54+t118+t120+t124+t127+t128-b2_1*t33-b5_2*t126-s1_1*t7-s2_1*t33),m5*t156*t248*theta_d4+m4*t163*t245*theta_d3-m3*t104*theta_d2*(t205-b3_2*t6*t9)+m2*t239*theta_d1*(t34+t51+t52+t54-t117-t129),-m5*t144*t248*theta_d4-m4*t245*t246*theta_d3+m3*t240*t260*theta_d2+m3*t267*theta_d2*(t205-t247)-m2*t239*theta_d1*(t82+t89+t90+t95-t145-t146)+m4*t213*theta_d3*(t102-b4_2*t29)+m5*t236*theta_d4*(t113-b5_2*t116)+m2*t251*theta_d1*(t198+t199+t200-t218-t220-t231),m3*t104*theta_d2*(t103+t257+t258-b3_2*t2*t6)+m4*theta_d3*(t102+t255+t256-b4_2*t29)*(t34+t35+t36+t37+t38+t45+t46+t47+t48+t49+t50+t51+t52+t53+t54-b2_1*t33-s1_1*t7-s2_1*t33)+m5*t156*theta_d4*(t113-b5_2*t116+b5_1*t151-b5_2*t153)-m5*t144*t203*theta_d4-m4*t184*theta_d3*(t77+t80+t82+t89+t90+t91+t92+t94+t95+t97+t98+t99+t100+t101-b2_3*t63-b4_2*t87-s1_3*t63-s2_3*t63)+m3*t207*theta_d2*(t77+t80+t82+t89+t90+t91+t92+t94+t95-b2_3*t63-s1_3*t63-s2_3*t63),m3*t104*t266*theta_d2-m4*t184*t213*theta_d3+m5*t156*t263*theta_d4+m4*t163*theta_d3*(t204+t253+t254-b4_1*t109)-m5*t203*theta_d4*(t187+t188+t189+t190+t194+t198+t199+t200+t201+t202+t219+t224-b2_3*t172-b3_3*t183-b4_3*t183-b5_3*t183-b4_2*t196-b5_2*t227-s1_3*t172-s2_3*t172-s3_3*t183-s4_3*t183-s4_2*t196-s1_1*t8*t56)+m3*t207*theta_d2*(t187+t188+t198+t199+t200+t201+t202-b2_3*t172-b3_3*t183-s1_3*t172-s2_3*t172-s1_1*t8*t56),m5*t236*theta_d4*(t113-b5_2*t116-b5_2*t153+b5_1*t210)-m5*t144*t263*theta_d4-m3*t266*t267*theta_d2+m3*t260*theta_d2*(t103+t257+t258-t259)-m4*t246*theta_d3*(t204-t252+t253+t254)+m4*t213*theta_d3*(t102+t255+t256-b4_2*t29),m5*t156*theta_d4*(t113-b5_2*t116+b5_1*t151*2.0-b5_2*t153*2.0)-m5*t144*t232*theta_d4-m4*t214*t246*theta_d3+m4*t163*theta_d3*(t102+t270+t271-b4_2*t29),-m4*t213*t214*theta_d3+m5*t156*t274*theta_d4-m5*t232*t236*theta_d4+m4*t163*theta_d3*(t204+t268+t269-b4_1*t109),m5*t236*theta_d4*(t113-b5_2*t116-b5_2*t153*2.0+b5_1*t210*2.0)-m5*t144*t274*theta_d4-m4*t246*theta_d3*(t204-t252+t268+t269)+m4*t213*theta_d3*(t102+t270+t271-b4_2*t29),m5*t156*theta_d4*(t113-b5_2*t116+b5_1*t151*3.0-b5_2*t153*3.0)-m5*t144*t237*theta_d4,m5*t156*t277*theta_d4-m5*t236*t237*theta_d4,m5*t236*theta_d4*(t113-b5_2*t116-b5_2*t153*3.0+b5_1*t210*3.0)-m5*t144*t277*theta_d4,0.0,0.0,0.0,0.0,0.0,0.0],(6,3)).T 
	return 	C_wm 