function v = t2v(M)
  v = zeros(3,1);
  v(1,1) = M(1,3);
  v(2,1) = M(2,3);
  v(3,1) = atan2(T(2,1), T(1,1));
end


