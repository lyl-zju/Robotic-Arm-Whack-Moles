function path = plan_cartesian_traj(pointsWorld, samplesPerSegment)
if nargin < 2
    samplesPerSegment = 20;
end

path = [];
for i = 1:(size(pointsWorld, 1) - 1)
    a = pointsWorld(i, :);
    b = pointsWorld(i + 1, :);
    alpha = linspace(0, 1, samplesPerSegment).';
    if i > 1
        alpha = alpha(2:end);
    end
    path = [path; a + alpha .* (b - a)]; %#ok<AGROW>
end
end

