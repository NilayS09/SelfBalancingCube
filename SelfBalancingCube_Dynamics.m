function z_dot = SelfBalancingCube_Dynamics(z,M,d,g,Iref)

    z_dot = zeros(4,1);

    phi = z(1); phi_dot = z(2);
    theta = z(3); theta_dot = z(4);

    z_dot(1) = phi_dot;
    z_dot(3) = theta_dot;

    R = RotationMatrixGenerator(0,theta,phi,['Y','Z','X']);
    I = R*Iref*(R');

    T_xb = 0;
    T_yb = 0;
    T_zb = 0;

    [A,b] = SelfBalancingCube_Acceleration(I,M,T_xb,T_yb,T_zb,d,g,...
                                           phi,phi_dot,theta,theta_dot);

    acc = A\b;

    phi_ddot = acc(1);
    theta_ddot = acc(2);

    z_dot(2) = phi_ddot;
    z_dot(4) = theta_ddot;

end