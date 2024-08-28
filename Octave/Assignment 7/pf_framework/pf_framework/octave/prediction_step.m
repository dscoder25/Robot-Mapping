function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);


for i = 1:numParticles
  u.r1 = u.r1 + normrnd(u.r1,r1Noise);
  u.t = u.t + normrnd(u.t,transNoise);
  u.r2 = u.r2 + normrnd(u.r2,r2Noise);

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;
  prev = particles(i).pose;
  % TODO: sample a new pose for the particle
  particles(i).pose = [prev(1)+(u.t*cos(prev(3)+u.r1));prev(2)+(u.t*sin(prev(3)+u.r1));normalize_angle(prev(3)+u.r1+u.r2)];
end

end
