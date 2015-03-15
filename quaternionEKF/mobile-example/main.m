% Load stored data
data = load('angVelData.mat','angVel','t_angVel');
% Integrating angular velocity
q_k = 0;
q_k1 = [];
Rw = 0.01*ones(3,1);
for i=2:lastElem
    q_k = q_k1
    q_k1(end+1) = tu_qw(q_k, [], T, Rw);
end