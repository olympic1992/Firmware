
float64 lat			# target position (deg * 1e7)
float64 lon			# target position (deg * 1e7) 
float32 alt			# target position

float32 vy			# target vel in y
float32 vx			# target vel in x
float32 vz			# target vel in z 

float32 roll_body				# body angle in NED frame
float32 pitch_body				# body angle in NED frame
float32 yaw_body				# body angle in NED frame

# formshape

# uint8 FORMSHAPE_NORMAL = 0
# uint8 FORMSHAPE_HORIZON1 = 1
# uint8 FORMSHAPE_VERTIAL1 = 2
# uint8 FORMSHAPE_RHOMBUS4 = 3

uint8 formshape_id                 # 飞机编队队形编号

