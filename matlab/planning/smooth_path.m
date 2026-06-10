function smooth = smooth_path(path, obstacles, iterations)
if nargin < 3
    iterations = 80;
end

smooth = path;
if size(smooth, 1) <= 2
    return;
end

for k = 1:iterations
    if size(smooth, 1) <= 2
        break;
    end
    i = randi([1, size(smooth, 1) - 2]);
    j = randi([i + 2, size(smooth, 1)]);
    if ~collision_check_2d(smooth(i, :), smooth(j, :), obstacles)
        smooth = [smooth(1:i, :); smooth(j:end, :)];
    end
end
end

