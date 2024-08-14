function [pntsMap] = world_to_map_coordinates(pntsWorld, gridSize, offset)
% Convert points from the world coordinates frame to the map frame.
% pntsWorld is a matrix of N points with each column representing a point in world coordinates (meters).
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% pntsMap is a 2xN matrix containing the corresponding points in map coordinates.

% TODO: compute pntsMap
N = size(pntsWorld,2);

pntsMap = zeros(2,N);

for i = 1:N
  pntsMap(1,i) = floor((pntsWorld(1,i) - offset(1))/gridSize);
  pntsMap(2,i) = floor((pntsWorld(2,i) - offset(2))/gridSize);
endfor

end
