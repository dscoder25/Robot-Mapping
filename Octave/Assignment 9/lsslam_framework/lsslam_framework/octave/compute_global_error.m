% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    Z = v2t(edge.measurement);
    omega = edge.information;
    error = Z\(x1\x2);
    eij = t2v(error);
    ei = eij' * omega * eij;
    Fx = Fx + ei;


  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    X = v2t(x);
    Z = v2t([edge.measurement; 0]);
    L = v2t([l; 0]);
    omega = edge.information;;
    e_tmp = t2v(Z\(X\L));
    e_tmp = e_tmp(1:2);
    eij = e_tmp' * omega * e_tmp;
    Fx = Fx + eij;
  end

end
