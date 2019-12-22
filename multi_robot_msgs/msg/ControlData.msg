#Header for timing and frames
Header header 


# #INPUTS:
# #Position that is used for controller calculations
# geometry_msgs/Vector3 position_in
# #Angle that is used for controller calculations misisng
# #Velocity that is used for controller calculations
# float64 angle_in
# geometry_msgs/Vector3 velocity_in
# #Angular velocity that is used for the controller calculations
# float64 angular_velocity_in

# #DIFFERENCES
# #Cartesian control difference that is used for controller calculations
# geometry_msgs/Vector3 control_difference_cart
# float64 control_difference_angular


# #OUTPUTS:
# #Linear velocity that is determined
# float64 linear_velocity_out
# #Angular velocity that is determined
# float64 angular_velocity_out


multi_robot_msgs/ControlState target
multi_robot_msgs/ControlState current
multi_robot_msgs/ControlDifference difference
multi_robot_msgs/ControlVector control