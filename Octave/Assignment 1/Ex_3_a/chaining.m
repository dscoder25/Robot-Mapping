initial = [2;3;1];
pose1 = [1;1;pi/4];
pose2 = [3;4;pi/2];
pose3 = [1;5;pi/6];
pose4 = [6;3;pi/3];

T1 = v2t(pose1);
T2 = v2t(pose2);
T3 = v2t(pose3);
T4 = v2t(pose4);

final_point = T4*T3*T2*T1*initial;
disp(final_point);
