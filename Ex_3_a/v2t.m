function M = v2t(p)
  M = [cos(p(3,1)) -sin(p(3,1)) p(1,1); sin(p(3,1)) cos(p(3,1)) p(2,1); 0 0 1];
end
