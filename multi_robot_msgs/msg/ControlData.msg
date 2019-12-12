#Header for timing and frames
Header header 
#Inputs:
#Position that is used for controller calculations
geometry_msgs/Vector3 position_in
#Velocity that is used for controller calculations
geometry_msgs/Vector3 velocity_in
#Angular velocity that is used for the controller calculations
float64 angular_velocity_in
#Cartesian control difference that is used for controller calculations
geometry_msgs/Vector3 control_difference_cart
#Outputs:
#Linear velocity that is determined
float64 linear_velocity_out
#Angular velocity that is determined
float64 angular_velocity_out
