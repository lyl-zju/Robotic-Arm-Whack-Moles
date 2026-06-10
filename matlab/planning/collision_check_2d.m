function hit = collision_check_2d(p1, p2, obstacles, resolution)
if nargin < 4
    resolution = 0.005;
end

distance = norm(p2 - p1);
numSamples = max(2, ceil(distance / resolution));
alpha = linspace(0, 1, numSamples).';
points = p1 + alpha .* (p2 - p1);

hit = false;
for i = 1:size(points, 1)
    if point_in_obstacle(points(i, :), obstacles)
        hit = true;
        return;
    end
end
end

function inside = point_in_obstacle(point, obstacles)
inside = false;
for k = 1:numel(obstacles)
    obstacle = obstacles(k);
    switch obstacle.type
        case "circle"
            c = obstacle.center;
            r = obstacle.radius;
            inside = norm(point - c) <= r;
        case "rect"
            b = obstacle.bounds;
            inside = point(1) >= b(1) && point(1) <= b(2) && ...
                point(2) >= b(3) && point(2) <= b(4);
        otherwise
            error("Unknown obstacle type: %s", obstacle.type);
    end
    if inside
        return;
    end
end
end

