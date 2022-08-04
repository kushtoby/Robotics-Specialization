function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
Kp = 100;
Kd = 20;

Kp1 = 30;
Kd1 = 10;

Kp2 = 1000;
Kd2 = 50;

e_pos = des_state.pos - state.pos;
e_vel = des_state.vel - state.vel;
phi_pos = state.rot;
phi_vel = state.omega;
acc = des_state.acc;
m =  params.mass;
g = params.gravity;
Ixx = params.Ixx;

u1 = m * (g + acc(2) + Kd*e_vel(2) + Kp * e_pos(2));
phic = -1/g *(acc(1) + Kd1 *e_vel(1) + Kp1 * e_pos(1));
phic_vel = 0;
phic_acc = 0;
u2 = Ixx * (phic_acc + Kd2 *(phic_vel - phi_vel) + Kp2 * (phic - phi_pos));
% FILL IN YOUR CODE HERE

end

