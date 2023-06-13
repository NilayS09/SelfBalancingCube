function [A,b] = SelfBalancingCube_Acceleration(I,M,T_xb,T_yb,T_zb,d,g,phi,phi_dot,theta,theta_dot)
%SelfBalancingCube_Acceleration
%    [A,B] = SelfBalancingCube_Acceleration(I1_1,I1_2,I1_3,I2_1,I2_2,I2_3,I3_1,I3_2,I3_3,M,T_xb,T_yb,T_zb,D,G,PHI,PHI_DOT,THETA,THETA_DOT)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    13-Jun-2023 20:06:29

I1_1 = I(1,1); I1_2 = I(1,2); I1_3 = I(1,3);
I2_1 = I(2,1); I2_2 = I(2,2); I2_3 = I(2,3);
I3_1 = I(3,1); I3_2 = I(3,2); I3_3 = I(3,3);

t2 = cos(phi);
t3 = cos(theta);
t4 = sin(phi);
t5 = sin(theta);
t6 = d.^2;
t7 = phi.*2.0;
t8 = phi_dot.^2;
t9 = theta_dot.^2;
t13 = sqrt(2.0);
t14 = sqrt(3.0);
t15 = sqrt(6.0);
t10 = t2.^2;
t11 = t3.^2;
A = reshape([-I1_1-M.*t6+M.*t6.*t11,-I2_1-I3_1+M.*t2.*t3.*t5.*t6+M.*t3.*t4.*t5.*t6,-I1_3.*t2+I1_2.*t4,-I2_3.*t2+I2_2.*t4-I3_3.*t2+I3_2.*t4-M.*t2.*t6+M.*t4.*t6],[2,2]);
if nargout > 1
    t12 = sin(t7);
    et1 = -I1_2.*t9+I2_1.*t8-I3_1.*t8+I1_2.*t9.*t10+I1_3.*t9.*t10-(I1_2.*t9.*t12)./2.0+(I1_3.*t9.*t12)./2.0+(T_xb.*t2.*t15)./3.0-(T_xb.*t4.*t15)./3.0-(T_yb.*t2.*t15)./6.0+(T_yb.*t4.*t15)./6.0-(T_zb.*t2.*t15)./6.0+(T_zb.*t4.*t15)./6.0+I1_1.*phi_dot.*t2.*theta_dot+I1_1.*phi_dot.*t4.*theta_dot-I2_2.*phi_dot.*t2.*theta_dot+I2_3.*phi_dot.*t2.*theta_dot-I2_2.*phi_dot.*t4.*theta_dot-I2_3.*phi_dot.*t4.*theta_dot-I3_2.*phi_dot.*t2.*theta_dot-I3_3.*phi_dot.*t2.*theta_dot+I3_2.*phi_dot.*t4.*theta_dot-I3_3.*phi_dot.*t4.*theta_dot-(T_xb.*t2.*t5.*t14)./3.0-(T_xb.*t4.*t5.*t14)./3.0-(T_yb.*t2.*t3.*t13)./2.0-(T_yb.*t3.*t4.*t13)./2.0-(T_yb.*t2.*t5.*t14)./3.0-(T_yb.*t4.*t5.*t14)./3.0;
    et2 = (T_zb.*t2.*t3.*t13)./2.0+(T_zb.*t3.*t4.*t13)./2.0-(T_zb.*t2.*t5.*t14)./3.0-(T_zb.*t4.*t5.*t14)./3.0-M.*d.*g.*t2.*t5+M.*d.*g.*t4.*t5-M.*phi_dot.*t2.*t6.*t11.*theta_dot.*2.0-M.*phi_dot.*t4.*t6.*t11.*theta_dot.*2.0-M.*t2.*t3.*t5.*t6.*t8+M.*t3.*t4.*t5.*t6.*t8;
    b = [I3_2.*t9-I2_3.*t9.*t10+(I2_2.*t9.*t12)./2.0-I3_2.*t9.*t10-(I3_3.*t9.*t12)./2.0-(T_xb.*t3.*t14)./3.0-(T_yb.*t3.*t14)./3.0+(T_yb.*t5.*t13)./2.0-(T_zb.*t3.*t14)./3.0-(T_zb.*t5.*t13)./2.0-I1_2.*phi_dot.*t2.*theta_dot-I1_3.*phi_dot.*t4.*theta_dot-I2_1.*phi_dot.*t2.*theta_dot-I3_1.*phi_dot.*t4.*theta_dot+M.*phi_dot.*t6.*theta_dot.*sin(theta.*2.0);et1+et2];
end
end
