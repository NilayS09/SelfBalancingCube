%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author : Nilay Srivastava
% Date Created : 13/06/2023
% Date Modified : 13/06/2023

% Description : This script finds Equations of Motion(EOM) for a Self 
% Balancing Cube modelled as a 3D inverted pendulum.

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
er = sin(theta)*cos(phi)*i + sin(theta)*sin(phi)*j + cos(theta)*k;
et = cos(theta)*cos(phi)*i + cos(theta)*sin(phi)*j - sin(theta)*k;
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

syms omega [3,1] real % current angular velocity about the rotating axis
syms omega_dot [3,1] real % current angular acceleration about the rotating axis
syms R [3,3] real

% Moment
M_O = cross(d*er,-M*g*j) + T_xb*R*i + T_yb*R*j + T_xb*R*k;

% Rate of Change of Angular Momentum
Hdot_O = cross(rG_O,M*aG_O) + I*omega_dot + cross(omega,I*omega);

eqn = M_O - Hdot_O;
omega_dot = solve(eqn,omega_dot);
omega_dot = [omega_dot.omega_dot1; omega_dot.omega_dot2; omega_dot.omega_dot3];
omega_dot = simplify(omega_dot);

%% Saving Acceleration Expression as a file

matlabFunction(omega_dot,'File','SelfBalancingCube_Acceleration');

