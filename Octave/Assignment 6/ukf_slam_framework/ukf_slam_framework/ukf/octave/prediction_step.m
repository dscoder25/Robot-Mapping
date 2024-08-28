function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles

for i = 1: 2*n +1
  sigma_points(1:3,i) = sigma_points(1:3,i) + [((u.t)*cos(mu(3)+u.r1));((u.t)*sin(mu(3)+u.r1));u.r1 + u.r2];
endfor
sigma_points(3,:) = normalize_angle(sigma_points(3,:));
% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% TODO: recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)
mu_f = zeros(size(mu));
sintheta = 0;
costheta = 0;
for i = 1:2*n + 1
  mu_f = mu_f + wm(i)*sigma_points(:,i);
  costheta = costheta + wm(i)*cos(sigma_points(3,1));
  sintheta = sintheta + wm(i)*sin(sigma_points(3,1));
endfor
theta_f = atan2(sintheta,costheta);
mu = mu_f;
mu(3) = theta_f;

% TODO: Recover sigma. Again, normalize the angular difference
sigma_f = zeros(size(sigma));
for i = 1:2*n + 1
  sigma_int = sigma_points;
  sigma_int(:,i) = sigma_int(:,i) - mu;
  sigma_int(3,i) = normalize_angle(sigma_int(3,i));
  sigma_f = sigma_f + wc(i)*(sigma_int(:,i)-mu)*(sigma_int(:,i)-mu)';
endfor
% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0;
     0, motionNoise, 0;
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma_f = sigma_f + R;
sigma = sigma_f;
disp(sigma);
end

