function [mu, sigma, map] = correction_step(mu, sigma, z, map);
% Updates the belief, i.e., mu and sigma after observing landmarks,
% and augments the map with newly observed landmarks.
% The employed sensor model measures the range and bearing of a landmark
% mu: state vector containing robot pose and poses of landmarks obeserved so far.
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector 'map' contains the ids of all landmarks observed so far by the robot in the order
% by which they were observed, NOT in ascending id order.

% For computing sigma
global scale;

% Number of measurements in this time step
m = size(z, 2);

% Measurement noise
Q = 0.01*eye(2);

for k = 1:m

	% If the landmark is observed for the first time:
	if (isempty(find(map == z(k).id)))
	    % Add new landmark to the map
            [mu, sigma, map] = add_landmark_to_map(mu, sigma, z(k), map, Q);
	    % The measurement has been incorporated so we quit the correction step
	    continue;
	endif

	% Compute sigma points from the predicted mean and covariance
        % This corresponds to line 6 on slide 32
  sigma_points = compute_sigma_points(mu, sigma);
        % Normalize!
  %disp(sigma_points(3,:));
	sigma_points(3,:) = normalize_angle(sigma_points(3,:));

	% Compute lambda
	n = length(mu);
	num_sig = size(sigma_points,2);
	lambda = scale - n;

        % extract the current location of the landmark for each sigma point
        % Use this for computing an expected measurement, i.e., applying the h function
	landmarkIndex = find(map==(z(k).id));
	landmarkXs = sigma_points(2*landmarkIndex + 2, :);
	landmarkYs = sigma_points(2*landmarkIndex + 3, :);

	% TODO: Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
  % This corresponds to line 7 on slide 32
  z_points = zeros(2,2*n+1);
  for j= 1:2*n + 1
    dx = landmarkXs(j) - sigma_points(1,j);
    dy = landmarkYs(j) - sigma_points(2,j);

    z_points(:,j) = [sqrt(dx*dx + dy*dy);atan2(dy,dx)-sigma_points(3,j)];
  endfor

  % setup the weight vector for mean and covariance
  wm = [lambda/scale, repmat(1/(2*scale), 1, 2*n)];
  wc = wm;

	% TODO: Compute zm, line 8 on slide 32
	% zm is the recovered expected measurement mean from z_points.
	% It will be a 2x1 vector [expected_range; expected_bearing].
        % For computing the expected_bearing compute a weighted average by
        % summing the sines/cosines of the angle
  zm = zeros(2,1);
  sintheta = 0;
  costheta =0;
  for i = 1:2*n+1
    zm = zm + wm(i)*z_points(:,i);
    sintheta = sintheta + wm(i)*sin(z_points(2,i));
    costheta = costheta + wm(i)*cos(z_points(2,i));
  endfor
  zm(2,1) = atan2(sintheta,costheta);


	% TODO: Compute the innovation covariance matrix S (2x2), line 9 on slide 32
        % Remember to normalize the bearing after computing the difference
  S = zeros(2,2);
  for i = 1:2*n+1
    diff = (z_points(:,i)-zm);
    diff(2) = normalize_angle(diff(2));
    S = S + wc(i)*(diff)*(diff)';
    S = S + Q;
  endfor

	% TODO: Compute Sigma_x_z, line 10 on slide 32
        % (which is equivalent to sigma times the Jacobian H transposed in EKF).
	% sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
        % Remember to normalize the bearing after computing the difference
  sigma_x_z = zeros(n,2);
  for i = 1:2*n+1
    z_int = z_points(:,i)-zm;
    z_int(2) = normalize_angle(z_int(2));
    sigma_int = (sigma_points(i)-mu);
    sigma_int(3) = normalize_angle(sigma_int(3));
    sigma_x_z = sigma_x_z + wc(i)*(sigma_int)*(z_int)';
  endfor

	% TODO: Compute the Kalman gain, line 11 on slide 32
  K = sigma_x_z*inv(S);

	% Get the actual measurement as a vector (for computing the difference to the observation)
	z_actual = zeros(2,1);
  z_actual = [z(k).range; z(k).bearing];

	% TODO: Update mu and sigma, line 12 + 13 on slide 32
        % normalize the relative bearing
  z_int = z_actual-zm;
  z_int(2) = normalize_angle(z_int(2));
  mu = mu + K*(z_int);
  sigma = sigma - K*S*K';
	% TODO: Normalize the robot heading mu(3)
  mu(3)= normalize_angle(mu(3));

endfor

end

