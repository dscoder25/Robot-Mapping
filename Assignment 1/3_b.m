p1 = [1;3;pi/4];
p2 = [9;4;pi/2];

% Performing transformation from p1 to p2 for some point
point = [4;5;1]; %coordinates in p1 frame

T1 = v2t(p1);
T2 = v2t(p2);

final = T2 * inv(T1) * point; % coordinates of point in p2 frame
display(final(1:2));

