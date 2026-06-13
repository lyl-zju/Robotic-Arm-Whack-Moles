function plot_results(result, savePath)
t = result.traj.t;
qDes = result.traj.q_des;
qAct = result.tracking.q_actual;
err = result.tracking.error;
hasForce = isfield(result, "force");

figure("Name", "Joint Tracking Results", "Color", "w");
if hasForce
    tiledlayout(5, 1);
else
    tiledlayout(3, 1);
end

nexttile;
plot(t, qDes, "LineWidth", 1.0);
title("Desired joint angles");
xlabel("t / s");
ylabel("q_d / rad");
grid on;

nexttile;
plot(t, qAct, "LineWidth", 1.0);
title("Actual joint angles in simplified PD simulation");
xlabel("t / s");
ylabel("q / rad");
grid on;

nexttile;
plot(t, err, "LineWidth", 1.0);
title(sprintf("Tracking error, RMSE = %.4f rad", result.tracking.rmse));
xlabel("t / s");
ylabel("error / rad");
grid on;

if hasForce
    nexttile;
    plot(result.force.t, result.force.normal, "LineWidth", 1.2);
    hold on;
    yline(result.force.threshold, "--r");
    title(sprintf("Virtual contact force, peak = %.2f N", result.force.peak_force));
    xlabel("t / s");
    ylabel("F_n / N");
    grid on;

    nexttile;
    plot(result.force.t, 1000 * result.force.penetration, "LineWidth", 1.2);
    title(sprintf("Virtual penetration, max = %.1f mm", 1000 * result.force.max_penetration));
    xlabel("t / s");
    ylabel("depth / mm");
    grid on;
end

if nargin >= 2 && ~isempty(savePath)
    exportgraphics(gcf, savePath, "Resolution", 180);
end
end
