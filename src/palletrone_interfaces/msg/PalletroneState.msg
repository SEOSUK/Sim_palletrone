float64 sample_time # sensor acquisition time in simulation seconds, BEFORE sensor delay
float64[3] pos
float64[3] vel
float64[3] acc
float64[3] rpy        # [roll, pitch, yaw]
float64[3] w_rpy      # [w_roll, w_pitch, w_yaw]
float64[3] a_rpy      # [a_roll, a_pitch, a_yaw]
float64[4] servo      # [Motor1_tilted_angle, Motor2_tilted_angle, Motor3_tilted_angle, Motor4_tilted_angle]
float64 common_effectiveness # common-mode eta at this sample
float64[4] actual_thrust_scale # eta_common * per-rotor relative multiplier
float64[3] truth_acc # raw MuJoCo translational acceleration before noisy finite differences
float64[3] truth_a_rpy # raw MuJoCo angular acceleration before noisy finite differences
float64[3] ou_torque_body # physical OU residual torque in body FLU [Nm]
float64[3] total_external_torque_body # configured constant plus OU torque [Nm]
float64[3] ou_force_body # physical OU residual force in body FLU [N]
float64[3] total_external_force_body # configured constant plus OU force in body FLU [N]
float64[4] motor_force # post-delay and post-lag physical rotor thrust [N]
