function [Txb, Tyb, Tzb] = SelfBalancingCube_TrajectoryController(...
                                            pose_des,pose,pose_der,gains,R)

    phi_des = pose_des(1); theta_des = pose_des(2);
    phi = pose(1); theta = pose(2); 
    phi_dot = pose_der(1); theta_dot = pose_der(2);

    phi_err = phi_des - phi;
    theta_err = theta_des - theta;

    Kp_Txb_theta = gains.theta.Kp; Kp_Txb_phi = gains.phi.Kp; 
    Kp_Tyb_theta = gains.theta.Kp; Kp_Tyb_phi = gains.phi.Kp;
    Kp_Tzb_theta = gains.theta.Kp; Kp_Tzb_phi = gains.phi.Kp;

    Kd_Txb_theta = gains.theta.Kd; Kd_Txb_phi = gains.phi.Kd; 
    Kd_Tyb_theta = gains.theta.Kd; Kd_Tyb_phi = gains.phi.Kd;
    Kd_Tzb_theta = gains.theta.Kd; Kd_Tzb_phi = gains.phi.Kd;
   

    PID_Txb_theta = Kp_Txb_theta*theta_err + Kd_Txb_theta*(-theta_dot);  
    PID_Tyb_theta = Kp_Tyb_theta*theta_err + Kd_Tyb_theta*(-theta_dot);  
    PID_Tzb_theta = Kp_Tzb_theta*theta_err + Kd_Tzb_theta*(-theta_dot); 

    PID_Txb_phi = Kp_Txb_phi*phi_err + Kd_Txb_phi*(-phi_dot);  
    PID_Tyb_phi = Kp_Tyb_phi*phi_err + Kd_Tyb_phi*(-phi_dot);  
    PID_Tzb_phi = Kp_Tzb_phi*phi_err + Kd_Tzb_phi*(-phi_dot); 

    ep = [-sin(phi); cos(phi); 0];
    i = [1;0;0]; j = [0;1;0]; k = [0;0;1];
    R_const = RotationMatrixGenerator(acos(1/sqrt(3)),0,pi/4,['Z','X','Y']);
    ib = R*R_const*i; jb = R*R_const*j; kb = R*R_const*k;

    Txb = PID_Txb_theta*dot(ib,ep) ...
          + PID_Txb_phi*dot(ib,k);
    Tyb = PID_Tyb_theta*dot(jb,ep)...
          + PID_Tyb_phi*dot(jb,k);
    Tzb = PID_Tzb_theta*dot(kb,ep)...
          + PID_Tzb_phi*dot(kb,k);

end