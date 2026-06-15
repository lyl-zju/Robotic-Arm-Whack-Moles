function plot_impact_force_control(result, savePath)
if nargin < 2
    savePath = "";
end

t = result.traj.t;
phase = result.control.phase;

figure("Name", "Impact Force Control Results", "Color", "w");
tiledlayout(5, 1, "TileSpacing", "compact");

nexttile;
plot(t, result.ee.position, "LineWidth", 1.0);
hold on;
plot(t, result.traj.x_des, "--", "LineWidth", 0.9);
yline(result.target_world(3), ":", "target z");
title("End-effector position and commanded position");
xlabel("t / s");
ylabel("x,y,z / m");
legend("x", "y", "z", "x_{cmd}", "y_{cmd}", "z_{cmd}", "Location", "eastoutside");
grid on;

nexttile;
plot(t, result.force.normal, "LineWidth", 1.2);
hold on;
plot(t, result.force.filtered, "LineWidth", 1.0);
plot(t, result.force.desired, "--", "LineWidth", 1.0);
yline(result.force.threshold, "--r", "threshold");
title(sprintf("Contact force, peak = %.2f N", result.force.peak_force));
xlabel("t / s");
ylabel("F_n / N");
legend("raw", "filtered", "target", "Location", "eastoutside");
grid on;

nexttile;
plot(t, 1000 * result.force.penetration, "LineWidth", 1.2);
title(sprintf("Penetration, max = %.2f mm", 1000 * result.force.max_penetration));
xlabel("t / s");
ylabel("depth / mm");
grid on;

nexttile;
plot(t, result.ee.velocity(:, 3), "LineWidth", 1.1);
hold on;
plot(t, result.traj.xd_des(:, 3), "--", "LineWidth", 1.0);
title("Normal velocity");
xlabel("t / s");
ylabel("z velocity / (m/s)");
legend("actual", "commanded", "Location", "eastoutside");
grid on;

nexttile;
plot(t, vecnorm(result.control.torque, 2, 2), "LineWidth", 1.0);
hold on;
plot_phase_bands(t, phase);
title("Commanded joint-torque norm and controller phases");
xlabel("t / s");
ylabel("||tau|| / Nm");
grid on;

if strlength(string(savePath)) > 0
    exportgraphics(gcf, savePath, "Resolution", 180);
end
end

function plot_phase_bands(t, phase)
phaseNames = ["approach", "force", "retract"];
colors = [
    0.82, 0.90, 1.00;
    1.00, 0.88, 0.72;
    0.84, 0.94, 0.84
];
yl = ylim;
for i = 1:numel(phaseNames)
    mask = phase == phaseNames(i);
    if ~any(mask)
        continue;
    end
    segments = mask_to_segments(mask);
    for s = 1:size(segments, 1)
        x0 = t(segments(s, 1));
        x1 = t(segments(s, 2));
        patch([x0 x1 x1 x0], [yl(1) yl(1) yl(2) yl(2)], colors(i, :), ...
            "FaceAlpha", 0.16, "EdgeColor", "none");
    end
end
ylim(yl);
end

function segments = mask_to_segments(mask)
edges = diff([false; mask(:); false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
segments = [starts, stops];
end
