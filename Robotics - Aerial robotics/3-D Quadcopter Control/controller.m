function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
m = params.mass;
g = params.gravity;
e_vel = des_state.vel - state.vel;
e_pos = des_state.pos - state.pos;
acc = des_state.acc;
Kp = [60; 60; 400; 200; 200; 200];
Kv = [20; 20; 10; 10; 10; 10];
des_psi = des_state.yaw;
psi_vel_des = des_state.yawdot;
ang_pos = state.rot;
ang_vel = state.omega;


% Thrust
F = m * ( Kp(3) * e_pos(3) + Kv(3) * e_vel(3) + g + acc(3));

%altitude control
des_phi = (1/g) * (((Kp(1) * e_pos(1) + Kv(1) * e_vel(1) + acc(1)) * sin(des_psi)) - ... 
             ((Kp(2) * e_pos(2) + Kv(2) * e_vel(2) + acc(2)) * cos(des_psi)));
des_theta = (1/g) * (((Kp(1) * e_pos(1) + Kv(1) * e_vel(1) + acc(1)) * cos(des_psi)) + ... 
             ((Kp(2) * e_pos(2) + Kv(2) * e_vel(2) + acc(2)) * sin(des_psi)));
p_des = 0;
q_des = 0;
r_des = psi_vel_des;

%desired omega and rot
des_ang_vel = [p_des; q_des; r_des];
des_ang_pos = [des_phi; des_theta; des_psi];

%error in omega and rot
e_ang_vel = des_ang_vel - ang_vel;
e_ang_pos = des_ang_pos - ang_pos;

% Moment
M = [Kp(4) * e_ang_pos(1) + Kv(4) * e_ang_vel(1);
    Kp(5) * e_ang_pos(2) + Kv(5) * e_ang_vel(2)
    Kp(6) * e_ang_pos(3) + Kv(6) * e_ang_vel(3)];

% =================== Your code ends here ===================

end
