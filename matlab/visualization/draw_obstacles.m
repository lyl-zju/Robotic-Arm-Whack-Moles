function draw_obstacles(obstacles)
hold on;
for k = 1:numel(obstacles)
    obstacle = obstacles(k);
    switch obstacle.type
        case "circle"
            theta = linspace(0, 2*pi, 80);
            x = obstacle.center(1) + obstacle.radius * cos(theta);
            y = obstacle.center(2) + obstacle.radius * sin(theta);
            fill(x, y, [0.8 0.2 0.1], "FaceAlpha", 0.25, "EdgeColor", [0.8 0.2 0.1]);
        case "rect"
            b = obstacle.bounds;
            rectangle("Position", [b(1), b(3), b(2)-b(1), b(4)-b(3)], ...
                "FaceColor", [0.8 0.2 0.1 0.25], "EdgeColor", [0.8 0.2 0.1]);
    end
end
end
