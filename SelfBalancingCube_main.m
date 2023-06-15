%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author : Nilay Srivastava
% Date Created : 13/06/2023
% Date Modified : 13/06/2023

% Description : Solves EOM for SelfBalancingCube, plots graph and shows
% animation

% Usage : SelfBalancingCube_main.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clearing Consoles

clc; clear; close all

%% Initializations

% System Parameter
M = 0.5;  % Mass in kg 
L = 0.15;   % Edge length in m
d = sqrt(3)*L/2; % Distance b/w COM and balance point also origin

% Environment Parameter
g = 10; % accereleration due to gravity in m/s^2;

% I = [ 1e-2, -4e-3, -4e-3;
%      -4e-3,  1e-2, -4e-3;
%      -4e-3, -4e-3,  1e-2];  % Inertia Matrix in kg m^2

I = diag([5e-3,5e-3,5e-3]);   % Inertia matrix in kg m^2

% Initial Conditions
phi0 = 0; phi_dot0 = 0;
theta0 = 0.2; theta_dot0 = 0;

z0 = [phi0;phi_dot0;theta0;theta_dot0];

%% Solving Dynamics
tend = 100; tspan = [0;tend];
f = @(t,z) SelfBalancingCube_Dynamics(t,z,M,d,g,I);

small = 1e-9;
options = odeset('AbsTol',small,'RelTol',small);
solution = ode45(f,tspan,z0,options);
toutput = linspace(solution.x(1),solution.x(end),5000);
Z = deval(solution,toutput);

phi = Z(1,:); phi_dot = Z(2,:);
theta = Z(3,:); theta_dot = Z(4,:);

%% Plotting Solutions

f1 = figure;
subplot(2,2,1)
plot(toutput,phi)
title("Yaw")
xlabel("t (seconds)")
ylabel("$\phi (radians)$",'Interpreter','latex')
subplot(2,2,2)
plot(toutput,phi_dot)
title("Yaw Rate")
xlabel("t (seconds)")
ylabel("$\dot{\phi} (radians/s)$",'Interpreter','latex')
subplot(2,2,3)
plot(toutput,theta)
title("Roll")
xlabel("t (seconds)")
ylabel("$\theta (radians)$",'Interpreter','latex')
subplot(2,2,4)
plot(toutput,theta_dot)
title("Roll Rate")
xlabel("t (seconds)")
ylabel("$\dot{\theta} (radians/s)$",'Interpreter','latex')

%% Animation

X = [0  0  L  L   0   0   L   L]';
Y = [0  L  L  0   0   L   L   0]';
Z = [0  0  0  0   L   L   L   L]';

box_vertices = [X,Y,Z];
box_faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

f2 = figure("Name","Self Balancing Cube Animation");
xlabel("X")
ylabel("Y")
zlabel("Z")
box = patch('Vertices',box_vertices,'Faces',box_faces,'FaceColor','b');
view(3)
axis equal
axis([-0.5 0.5 -0.5 0.5 -0.5 0.5])

R_const = RotationMatrixGenerator(acos(1/sqrt(3)),0,pi/4,['Z','X','Y']);
box_vertices = box_vertices*R_const';

% Animation
animspeed = 1;
tic
t = toc*animspeed;
while(t<tend)
    z = deval(solution,t);
    phi = z(1); theta = z(3);
    R = RotationMatrixGenerator(0,theta,phi,['Y','Z','X']);
    box.Vertices = box_vertices*R';
    drawnow
    t = toc*animspeed;
end

