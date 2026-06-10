function plot_results(result, savePath)
t = result.traj.t;
qDes = result.traj.q_des;
qAct = result.tracking.q_actual;
err = result.tracking.error;

figure("Name", "Joint Tracking Results", "Color", "w");
tiledlayout(3, 1);

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

if nargin >= 2 && ~isempty(savePath)
    exportgraphics(gcf, savePath, "Resolution", 180);
end
end

