

% Gait Scheduler

gait_period = 5; %s
t0 = 1;
R_z = eye(3);

offset_1 = 0;
offset_2 = 0.5;
offset_3 = 0;
offset_4 = 0.5;



t = 1.895;

phi = (t - t0) / gait_period;

phi_1 = mod(phi + offset_1,1)
phi_2 = mod(phi + offset_2,1)
phi_3 = mod(phi + offset_3,1)
phi_4 = mod(phi + offset_4,1)


% foot step planner

body_pos = [0;0;0.5];

L1 = [0.3;0.15;0];
L2 = [0.3;-0.15;0];
L3 = [-0.3;0.15;0];
L4 = [-0.3;-0.15;0];

p_shoulder_1 = body_pos + R_z * L1;
p_shoulder_2 = body_pos + R_z * L2;
p_shoulder_3 = body_pos + R_z * L3;
p_shoulder_4 = body_pos + R_z * L4;





