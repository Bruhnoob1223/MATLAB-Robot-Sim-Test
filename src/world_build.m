function world = world_build(rects, circs, bounds)
%WORLD_BUILD  Assemble a world struct from axis-aligned obstacle lists.
%
%   world = world_build(rects, circs, bounds)
%
%   Inputs
%   ------
%   rects  : Nx4 matrix, each row [cx, cy, width, height] (axis-aligned rect)
%   circs  : Mx3 matrix, each row [cx, cy, radius]
%   bounds : 1x4 vector [xmin, ymin, xmax, ymax] defining the map boundary
%
%   Output
%   ------
%   world  : struct with fields .rects, .circs, .bounds

if nargin < 1 || isempty(rects),  rects  = zeros(0,4); end
if nargin < 2 || isempty(circs),  circs  = zeros(0,3); end
if nargin < 3 || isempty(bounds), bounds = [-10,-10,10,10]; end

world.rects  = rects;
world.circs  = circs;
world.bounds = bounds;
end
