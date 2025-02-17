close all; clc; clear all;

data_path = './training/data/2024_02_19_21_08_57/positions';

T = readtable([data_path,'.csv']);

q1 = rotm2quat(eye(3));

q2 = [T.qw_end_avg(1),T.qx_end_avg(1),T.qy_end_avg(1),T.qz_end_avg(1)];
q2 = q2 / norm(q2);

q2_conj = [q2(1), -q2(2), -q2(3), -q2(4)];

% Quaternion multiplication (q1_conj * q2)
q_rel = quatmultiply(q2_conj, q1);

% Convert the relative quaternion to a rotation matrix
R_rel = quat2rotm(q_rel);
R_2 = quat2rotm(q2);


for i = 1:size(T,1)
    qi = [T.qw_end_avg(i),T.qx_end_avg(i),T.qy_end_avg(i),T.qz_end_avg(i)];
    qi = qi / norm(qi);

    R_i = quat2rotm(qi);

    % Find new orientation
    qi_new = rotm2quat(R_rel*R_i);

    % Update table
    T.qw_end_avg(i) = qi_new(1);
    T.qx_end_avg(i) = qi_new(2);
    T.qy_end_avg(i) = qi_new(3);
    T.qz_end_avg(i) = qi_new(4);
    
end

writetable(T,[data_path,'_norm','.csv']);

