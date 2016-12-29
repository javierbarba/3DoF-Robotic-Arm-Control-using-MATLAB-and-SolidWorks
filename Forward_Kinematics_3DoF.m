%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Forward Kinematics of a 3DoF robot using Denavit-Hartenberg      %%%
%%%                         Javier Barba, 2016-17                       %%%
%%%                    https://github.com/javierbarba                   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This function computes the final position of the robot's TCP with the
% values of the three motors. "theta(n)" represents the motor that turns
% the link "(n)". "P" represent the TCP's final cartesian cordinates in 
% "x", "y" and "z" planes.

function [ P ] = Forward_Kinematics_3DoF( degtheta )

%% ROBOT PARAMETERS
% Values of the robot's structure. Substitute this values with your robot's

% Parameters of the first beam (aqua)
alpha1 = 90; % in degrees
a1 = 0; % in mm
d1 = 17.5; % in mm

% Parameters of the second beam (green)
alpha2 = 0; % in degrees
a2 = 65; % in mm
d2 = 0; % in mm

% Parameters of the third beam (yellow)
alpha3 = 0; % in degrees
a3 = 67.5; % in mm
d3 = 0; % in mm

% Motors' angles
theta1 = degtheta(1);
theta2 = degtheta(2);
theta3 = degtheta(3);

%% LINKS' MATRICES
% Matrices using Denavit-Hartenberg Parameters for describing each beam
% of the robot.

T01 = [ cosd(theta1)  -sind(theta1)*cosd(alpha1)   sind(theta1)*sind(alpha1)  a1*cosd(theta1) ;
        sind(theta1)   cosd(theta1)*cosd(alpha1)  -cosd(theta1)*sind(alpha1)  a1*sind(theta1) ;
             0               sind(alpha1)                cosd(alpha1)               d1        ;
             0                    0                           0                      1        ];

T12 = [ cosd(theta2)  -sind(theta2)*cosd(alpha2)   sind(theta2)*sind(alpha2)  a2*cosd(theta2) ;
        sind(theta2)   cosd(theta2)*cosd(alpha2)  -cosd(theta2)*sind(alpha2)  a2*sind(theta2) ;
             0               sind(alpha2)                cosd(alpha2)               d2        ;
             0                    0                           0                      1        ];
 
T23 = [ cosd(theta3)  -sind(theta3)*cosd(alpha3)   sind(theta3)*sind(alpha3)  a3*cosd(theta3) ;
        sind(theta3)   cosd(theta3)*cosd(alpha3)  -cosd(theta3)*sind(alpha3)  a3*sind(theta3) ;
             0               sind(alpha3)                cosd(alpha3)               d3        ;
             0                    0                           0                      1        ];         

%% FORWARD KINEMATICS   
         
T03 = T01*T12*T23;

%% CALCULATED POSITION
% Final TCP position

P(1) = T03(1,4); % in mm, along x-axis
P(2) = T03(2,4); % in mm, along y-axis
P(3) = T03(3,4); % in mm, along z-axis