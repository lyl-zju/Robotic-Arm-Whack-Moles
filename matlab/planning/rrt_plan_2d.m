function [path, info] = rrt_plan_2d(startXY, goalXY, obstacles, bounds, options)
if nargin < 5
    options = struct();
end
options = fill_defaults(options);

if isfield(options, "seed")
    rng(options.seed);
end

nodes = startXY(:).';
parents = 0;
found = false;
goalIndex = NaN;

for iter = 1:options.max_iter
    if rand < options.goal_bias
        sample = goalXY(:).';
    else
        sample = [rand_range(bounds(1), bounds(2)), rand_range(bounds(3), bounds(4))];
    end

    [~, nearIdx] = min(vecnorm(nodes - sample, 2, 2));
    near = nodes(nearIdx, :);
    direction = sample - near;
    dist = norm(direction);
    if dist < eps
        continue;
    end
    newNode = near + options.step_size * direction / dist;

    if collision_check_2d(near, newNode, obstacles, options.collision_resolution)
        continue;
    end

    nodes = [nodes; newNode]; %#ok<AGROW>
    parents = [parents; nearIdx]; %#ok<AGROW>

    if norm(newNode - goalXY) < options.goal_threshold && ...
            ~collision_check_2d(newNode, goalXY, obstacles, options.collision_resolution)
        nodes = [nodes; goalXY(:).']; %#ok<AGROW>
        parents = [parents; size(nodes, 1) - 1]; %#ok<AGROW>
        found = true;
        goalIndex = size(nodes, 1);
        break;
    end
end

if found
    path = backtrack(nodes, parents, goalIndex);
else
    path = [];
end

info.found = found;
info.iterations = iter;
info.nodes = nodes;
info.parents = parents;
end

function options = fill_defaults(options)
defaults.step_size = 0.02;
defaults.goal_threshold = 0.03;
defaults.max_iter = 2000;
defaults.goal_bias = 0.10;
defaults.collision_resolution = 0.005;
names = fieldnames(defaults);
for i = 1:numel(names)
    if ~isfield(options, names{i})
        options.(names{i}) = defaults.(names{i});
    end
end
end

function value = rand_range(low, high)
value = low + rand * (high - low);
end

function path = backtrack(nodes, parents, idx)
path = nodes(idx, :);
while parents(idx) ~= 0
    idx = parents(idx);
    path = [nodes(idx, :); path]; %#ok<AGROW>
end
end

