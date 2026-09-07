float32[3] pos_cmd
float32[3] vel_cmd # world velocity feedforward [m/s]
float32[3] acc_cmd # world acceleration feedforward [m/s^2]
float32[3] rpy_cmd # desired attitude [rad], ZYX Euler convention
float32[3] rate_cmd # desired body angular velocity feedforward [rad/s]
bool moce_enabled # runtime MOCE adaptation switch
bool moce_z_enabled # runtime Z-axis adaptation switch; X/Y follow moce_enabled
