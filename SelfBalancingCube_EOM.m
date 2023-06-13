%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author : Nilay Srivastava
% Date Created : 13/06/2023
% Date Modified : 13/06/2023

% Description : This script finds Equations of Motion(EOM) for a Self 
% Balancing Cube modelled as a 3D inverted pendulum.
%
% Convention : i_cap is vertically upwards, j_cap and k_cap are then 
% according to Right Hand Rule  
% Spherical coordinates are defined according to the convention

% Usage : SelfBalancingCube_EOM.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clearing Consoles
clc; clear; close all

%% Initializations

% unit vectors
i = [1;0;0]; j = [0;1;0]; k = [0;0;1];

% lean angles
syms phi real  % yaw 
syms theta real    % angle from vertical(inertial z-axis)

% lean anglular velocity
syms phi_dot real
syms theta_dot real

% lean anglular acceleration
syms phi_ddot real
syms theta_ddot real

% spherical coordinates unit vector
er = sin(theta)*cos(phi)*j + sin(theta)*sin(phi)*k + cos(theta)*i;
et = cos(theta)*cos(phi)*j + cos(theta)*sin(phi)*k - sin(theta)*i;
ep = simplify(cross(er,et));

% System Parameters
syms M positive % cube mass
syms L positive % cube edge length
syms d positive % distance b/w COM and balance point also origin
syms I [3,3] real % cube inertia matrix

% Environment Parameters
syms g real % acceleration due to gravity

% Control Parameters
syms T_xb real % Torque in body's x direction
syms T_yb real % Torque in body's y direction
syms T_zb real % Torque in body's z direction

%% Finding COM Acceleration

rG_O = d*er;
vars = [theta; phi];
vars_dot = [theta_dot; phi_dot];

vG_O = jacobian(rG_O,vars)*vars_dot;
vars = [theta; theta_dot; phi; phi_dot];
vars_dot = [theta_dot; theta_ddot; phi_dot; phi_ddot];

aG_O = jacobian(vG_O,vars)*vars_dot;
aG_O = simplify(aG_O);

%% Solving EOM

R = [er, et, ep];
R_const = RotationMatrixGenerator(pi/4,acos(1/sqrt(3)),0,['X','Y','Z']);
Rx = RotationMatrixGenerator(phi,0,0,['X','Y','Z']);
omega = phi_dot*i + Rx*theta_dot*k;
vars = [phi;phi_dot;theta_dot];
vars_dot = [phi_dot;phi_ddot;theta_ddot];
omega_dot = jacobian(omega,vars)*vars_dot;
omega_dot = simplify(omega_dot);

% syms omega [3,1] real
%syms omega_dot [3,1] real

% Moment
M_O = cross(d*er,-M*g*i) + T_xb*R*R_const*i ...
      + T_yb*R*R_const*j + T_zb*R*R_const*k;

% Rate of Change of Angular Momentum
Hdot_O = cross(rG_O,M*aG_O) + I*omega_dot + cross(omega,I*omega);

eqn = M_O - Hdot_O; 
eqn1 = simplify(eqn(1));
eqn2 = simplify(eqn(2)+eqn(3));
eqns = [eqn1;eqn2];
vars = [phi_ddot;theta_ddot];
[A,b] = equationsToMatrix(eqns,vars);

A = simplify(A); b = simplify(b);

%% Saving Acceleration Expression as a file

matlabFunction(A,b,'File','SelfBalancingCube_Acceleration');

