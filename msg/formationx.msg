# 主机位置
float64 lat			# target position (deg * 1e7)
float64 lon			# target position (deg * 1e7) 
float32 alt			# target position

float32 vy			# target vel in y
float32 vx			# target vel in x
float32 vz			# target vel in z 

# formshape

uint8 FORMSHAPE_NORMAL = 0
uint8 FORMSHAPE_HORIZON1 = 1
uint8 FORMSHAPE_VERTIAL1 = 2
uint8 FORMSHAPE_RHOMBUS4 = 3

uint8 formshape_id                 # 飞机编队队形编号


# TOPICS formationx




# 
# float32 vy			# target vel in y
# float32 vx			# target vel in x
# float32 vz			# target vel in z 


# 
# float32 roll_body				# body angle in NED frame
# float32 pitch_body				# body angle in NED frame
# float32 yaw_body				# body angle in NED frame


# float32 yaw_sp_move_rate		# rad/s (commanded by user)

# For quaternion-based attitude control
# float32[4] q_d					# Desired quaternion for quaternion control
# bool q_d_valid					# Set to true if quaternion vector is valid

# float32 thrust					# Thrust in Newton the power system should generate

# bool roll_reset_integral			# Reset roll integral part (navigation logic change)
# bool pitch_reset_integral			# Reset pitch integral part (navigation logic change)
# bool yaw_reset_integral				# Reset yaw integral part (navigation logic change)

# bool fw_control_yaw				# control heading with rudder (used for auto takeoff on runway)
# bool disable_mc_yaw_control			# control yaw for mc (used for vtol weather-vane mode)



