function z_dot = SelfBalancingCube_Dynamics(t,z,M,d,g,Iref)

    z_dot = zeros(4,1);

    phi = z(1); phi_dot = z(2);
    theta = z(3); theta_dot = z(4);

    z_dot(1) = phi_dot;
    z_dot(3) = theta_dot;

    R = RotationMatrixGenerator(0,theta,phi,['Y','Z','X']);
    I = R*Iref*(R');

    phi_des = 1*t; theta_des = 0.2*cos(t);
    pose_des = [phi_des;theta_des];
    pose = [phi;theta];
    pose_der = [phi_dot;theta_dot];
    gains.theta = struct('Kp',2,'Kd',0.5,'Ki',0);
    gains.phi = struct('Kp',1,'Kd',0.2,'Ki',0);

    [T_xb, T_yb, T_zb] = SelfBalancingCube_TrajectoryController(...
                         pose_des,pose,pose_der,gains,R);

    % T_xb = 0;
    % T_yb = 0;
    % T_zb = 0;

    [A,b] = SelfBalancingCube_Acceleration(I,M,T_xb,T_yb,T_zb,d,g,...
                                           phi,phi_dot,theta,theta_dot);

    acc = A\b;

    phi_ddot = acc(1);
    theta_ddot = acc(2);

    z_dot(2) = phi_ddot;
    z_dot(4) = theta_ddot;

end