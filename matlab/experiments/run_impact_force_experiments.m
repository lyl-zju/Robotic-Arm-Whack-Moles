clc; close all;

expDir = fileparts(mfilename("fullpath"));
rootDir = fileparts(fileparts(expDir));
addpath(genpath(fullfile(rootDir, "matlab")));

resultDir = fullfile(expDir, "results");
dataDir = fullfile(resultDir, "data");
figDir = fullfile(resultDir, "figures");
ensure_dir(resultDir);
ensure_dir(dataDir);
ensure_dir(figDir);

previousFigureVisible = get(0, "DefaultFigureVisible");
cleanup = onCleanup(@() set(0, "DefaultFigureVisible", previousFigureVisible));
set(0, "DefaultFigureVisible", "off");

cfgRobot = config_robot();
cfgBoard = config_board();
cfgForce = config_force();
cfgImpactBase = config_impact_force_control();
robot = build_robot(cfgRobot);

targetBoard = [0.0, 0.0];
targetWorld = board_to_world(targetBoard, cfgBoard);

fprintf("Impact-force experiment target board=[%.3f %.3f], world=[%.3f %.3f %.3f]\n", ...
    targetBoard(1), targetBoard(2), targetWorld(1), targetWorld(2), targetWorld(3));

allMetrics = [];

% -------------------------------------------------------------------------
% Experiment 1: typical single hit under the default force-control settings.
% -------------------------------------------------------------------------
typicalCfg = cfgImpactBase;
typicalResult = simulate_impact_force_control( ...
    robot, targetWorld, cfgRobot, cfgBoard, cfgForce, typicalCfg);
typicalResult.target_mode = "experiment_center";
typicalResult.target_id = 5;
typicalResult.target_board = targetBoard;

plot_impact_force_control(typicalResult, ...
    fullfile(figDir, "exp1_typical_single_hit_detail.png"));
close(gcf);

save_result_json(typicalResult, ...
    fullfile(dataDir, "exp1_typical_single_hit_summary.json"));
writetable(result_timeseries_table(typicalResult), ...
    fullfile(dataDir, "exp1_typical_single_hit_timeseries.csv"));

typicalMetrics = result_metrics("typical_single_hit", "default_force_control", ...
    typicalResult, typicalCfg);
allMetrics = [allMetrics; typicalMetrics]; %#ok<AGROW>

% -------------------------------------------------------------------------
% Experiment 2: sweep impact speed and record peak force.
% -------------------------------------------------------------------------
impactSpeeds = [0.20, 0.26, 0.32, 0.38, 0.44].';
speedMetrics = repmat(empty_metric(), numel(impactSpeeds), 1);
speedResults = cell(numel(impactSpeeds), 1);

for i = 1:numel(impactSpeeds)
    cfg = cfgImpactBase;
    cfg.impact_speed = impactSpeeds(i);
    result = simulate_impact_force_control( ...
        robot, targetWorld, cfgRobot, cfgBoard, cfgForce, cfg);
    result.target_mode = "experiment_center";
    result.target_id = 5;
    result.target_board = targetBoard;
    speedResults{i} = result;
    speedMetrics(i) = result_metrics("impact_speed_sweep", ...
        sprintf("impact_speed_%.2f_mps", impactSpeeds(i)), result, cfg);
end

speedTable = struct2table(speedMetrics);
writetable(speedTable, fullfile(dataDir, "exp2_impact_speed_sweep_metrics.csv"));
plot_speed_sweep(speedTable, cfgForce, fullfile(figDir, "exp2_impact_speed_sweep.png"));

for i = 1:numel(speedResults)
    filename = sprintf("exp2_speed_%.2f_mps_timeseries.csv", impactSpeeds(i));
    writetable(result_timeseries_table(speedResults{i}), fullfile(dataDir, filename));
end
allMetrics = [allMetrics; speedMetrics(:)]; %#ok<AGROW>

% -------------------------------------------------------------------------
% Experiment 3: compare with and without post-contact admittance feedback.
% -------------------------------------------------------------------------
feedbackCases = [
    struct("case_name", "with_admittance_feedback", "admittance_gain", cfgImpactBase.admittance_gain)
    struct("case_name", "without_admittance_feedback", "admittance_gain", 0.0)
];
feedbackMetrics = repmat(empty_metric(), numel(feedbackCases), 1);
feedbackResults = cell(numel(feedbackCases), 1);

for i = 1:numel(feedbackCases)
    cfg = cfgImpactBase;
    cfg.admittance_gain = feedbackCases(i).admittance_gain;
    result = simulate_impact_force_control( ...
        robot, targetWorld, cfgRobot, cfgBoard, cfgForce, cfg);
    result.target_mode = "experiment_center";
    result.target_id = 5;
    result.target_board = targetBoard;
    feedbackResults{i} = result;
    feedbackMetrics(i) = result_metrics("feedback_comparison", ...
        feedbackCases(i).case_name, result, cfg);
end

feedbackTable = struct2table(feedbackMetrics);
writetable(feedbackTable, fullfile(dataDir, "exp3_feedback_comparison_metrics.csv"));
plot_feedback_comparison(feedbackResults, feedbackCases, cfgForce, ...
    fullfile(figDir, "exp3_feedback_comparison.png"));

for i = 1:numel(feedbackResults)
    filename = sprintf("exp3_%s_timeseries.csv", feedbackCases(i).case_name);
    writetable(result_timeseries_table(feedbackResults{i}), fullfile(dataDir, filename));
end
allMetrics = [allMetrics; feedbackMetrics(:)]; %#ok<AGROW>

% -------------------------------------------------------------------------
% Experiment 4: sweep knock-down force threshold. In this experiment the
% post-contact force target changes with the threshold:
%   F_des = threshold * cfgImpact.force_target_ratio.
% -------------------------------------------------------------------------
forceThresholds = [6, 8, 10, 15, 20].';
thresholdMetrics = repmat(empty_metric(), numel(forceThresholds), 1);
thresholdResults = cell(numel(forceThresholds), 1);

for i = 1:numel(forceThresholds)
    cfgThisForce = cfgForce;
    cfgThisForce.threshold = forceThresholds(i);
    result = simulate_impact_force_control( ...
        robot, targetWorld, cfgRobot, cfgBoard, cfgThisForce, cfgImpactBase);
    result.target_mode = "experiment_center";
    result.target_id = 5;
    result.target_board = targetBoard;
    thresholdResults{i} = result;
    thresholdMetrics(i) = result_metrics("force_threshold_sweep", ...
        sprintf("threshold_%.0f_N", forceThresholds(i)), result, cfgImpactBase);
end

thresholdTable = struct2table(thresholdMetrics);
writetable(thresholdTable, fullfile(dataDir, "exp4_force_threshold_sweep_metrics.csv"));
plot_threshold_sweep(thresholdResults, forceThresholds, ...
    fullfile(figDir, "exp4_force_threshold_sweep_curves.png"));
plot_threshold_summary(thresholdTable, ...
    fullfile(figDir, "exp4_force_threshold_sweep_summary.png"));

for i = 1:numel(thresholdResults)
    filename = sprintf("exp4_threshold_%.0f_N_timeseries.csv", forceThresholds(i));
    writetable(result_timeseries_table(thresholdResults{i}), fullfile(dataDir, filename));
end
allMetrics = [allMetrics; thresholdMetrics(:)]; %#ok<AGROW>

allMetricsTable = struct2table(allMetrics);
writetable(allMetricsTable, fullfile(dataDir, "all_experiment_metrics.csv"));
save(fullfile(dataDir, "all_experiment_results.mat"), ...
    "typicalResult", "speedResults", "feedbackResults", "thresholdResults", ...
    "allMetricsTable", "speedTable", "feedbackTable", "thresholdTable", ...
    "targetBoard", "targetWorld", "cfgForce", "cfgImpactBase");

fprintf("\nExperiment metrics saved to:\n%s\n", fullfile(dataDir, "all_experiment_metrics.csv"));
fprintf("Experiment figures saved to:\n%s\n", figDir);
disp(allMetricsTable(:, ["experiment", "case_name", "impact_speed_mps", ...
    "admittance_gain", "peak_force_N", "max_penetration_mm", ...
    "contact_duration_ms", "threshold_duration_ms", "knocked_down"]));

function ensure_dir(path)
if ~exist(path, "dir")
    mkdir(path);
end
end

function metric = empty_metric()
metric = struct( ...
    "experiment", "", ...
    "case_name", "", ...
    "impact_speed_mps", NaN, ...
    "admittance_gain", NaN, ...
    "peak_force_N", NaN, ...
    "peak_time_s", NaN, ...
    "threshold_N", NaN, ...
    "force_target_N", NaN, ...
    "max_penetration_mm", NaN, ...
    "contact_duration_ms", NaN, ...
    "threshold_duration_ms", NaN, ...
    "first_contact_time_s", NaN, ...
    "success_time_s", NaN, ...
    "force_rmse_in_contact_N", NaN, ...
    "mean_filtered_force_in_contact_N", NaN, ...
    "tracking_rmse_mm", NaN, ...
    "tracking_max_abs_error_mm", NaN, ...
    "min_xy_error_mm", NaN, ...
    "knocked_down", false);
end

function metric = result_metrics(experimentName, caseName, result, cfgImpact)
metric = empty_metric();
metric.experiment = string(experimentName);
metric.case_name = string(caseName);
metric.impact_speed_mps = cfgImpact.impact_speed;
metric.admittance_gain = cfgImpact.admittance_gain;
metric.peak_force_N = result.force.peak_force;
[~, peakIndex] = max(result.force.normal);
metric.peak_time_s = result.force.t(peakIndex);
metric.threshold_N = result.force.threshold;
metric.force_target_N = result.force.target;
metric.max_penetration_mm = 1000 * result.force.max_penetration;
metric.contact_duration_ms = 1000 * result.force.contact_duration;
metric.threshold_duration_ms = 1000 * result.force.threshold_duration;
metric.first_contact_time_s = finite_or_nan(result.force.first_contact_time);
metric.success_time_s = finite_or_nan(result.force.success_time);

contactMask = result.force.in_contact;
if any(contactMask)
    forceError = result.force.filtered(contactMask) - result.force.target;
    metric.force_rmse_in_contact_N = sqrt(mean(forceError .^ 2));
    metric.mean_filtered_force_in_contact_N = mean(result.force.filtered(contactMask));
end

metric.tracking_rmse_mm = 1000 * result.tracking.rmse;
metric.tracking_max_abs_error_mm = 1000 * result.tracking.max_abs_error;
metric.min_xy_error_mm = 1000 * result.force.min_xy_error;
metric.knocked_down = logical(result.force.knocked_down);
end

function value = finite_or_nan(value)
if ~isfinite(value)
    value = NaN;
end
end

function tbl = result_timeseries_table(result)
tbl = table( ...
    result.force.t(:), ...
    result.force.normal(:), ...
    result.force.filtered(:), ...
    result.force.measured(:), ...
    result.force.desired(:), ...
    1000 * result.force.penetration(:), ...
    result.ee.position(:, 3), ...
    result.traj.x_des(:, 3), ...
    result.ee.velocity(:, 3), ...
    result.force.closing_velocity(:), ...
    string(result.control.phase(:)), ...
    'VariableNames', { ...
        't_s', ...
        'force_normal_N', ...
        'force_filtered_N', ...
        'force_measured_N', ...
        'force_desired_N', ...
        'penetration_mm', ...
        'ee_z_m', ...
        'cmd_z_m', ...
        'ee_z_velocity_mps', ...
        'closing_velocity_mps', ...
        'phase'});
end

function plot_speed_sweep(speedTable, cfgForce, savePath)
fig = figure("Name", "Impact Speed Sweep", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");

nexttile;
plot(speedTable.impact_speed_mps, speedTable.peak_force_N, "-o", ...
    "LineWidth", 1.5, "MarkerSize", 6);
hold on;
yline(cfgForce.threshold, "--r", "threshold", "LineWidth", 1.0);
grid on;
xlabel("impact speed / (m/s)");
ylabel("peak contact force / N");
title("Peak force increases with commanded impact speed");

nexttile;
plot(speedTable.impact_speed_mps, speedTable.max_penetration_mm, "-s", ...
    "LineWidth", 1.5, "MarkerSize", 6);
grid on;
xlabel("impact speed / (m/s)");
ylabel("max penetration / mm");
title("Maximum mole compression under different impact speeds");

exportgraphics(fig, savePath, "Resolution", 180);
close(fig);
end

function plot_feedback_comparison(results, cases, cfgForce, savePath)
colors = [
    0.05, 0.35, 0.90;
    0.85, 0.18, 0.12
];

fig = figure("Name", "Force Feedback Comparison", "Color", "w");
tiledlayout(3, 1, "TileSpacing", "compact");

nexttile;
hold on;
for i = 1:numel(results)
    plot(results{i}.force.t, results{i}.force.filtered, ...
        "LineWidth", 1.3, "Color", colors(i, :));
end
yline(results{1}.force.target, "--", "target", "LineWidth", 1.0);
yline(cfgForce.threshold, "--r", "threshold", "LineWidth", 1.0);
grid on;
xlabel("t / s");
ylabel("filtered force / N");
title("Filtered contact force");
legend(case_labels(cases), "Location", "eastoutside");

nexttile;
hold on;
for i = 1:numel(results)
    plot(results{i}.force.t, 1000 * results{i}.force.penetration, ...
        "LineWidth", 1.3, "Color", colors(i, :));
end
grid on;
xlabel("t / s");
ylabel("penetration / mm");
title("Mole compression depth");

nexttile;
hold on;
for i = 1:numel(results)
    plot(results{i}.traj.t, results{i}.traj.x_des(:, 3), ...
        "--", "LineWidth", 1.1, "Color", colors(i, :));
    plot(results{i}.traj.t, results{i}.ee.position(:, 3), ...
        "-", "LineWidth", 1.0, "Color", colors(i, :));
end
grid on;
xlabel("t / s");
ylabel("z / m");
title("Commanded and actual end-effector height");

exportgraphics(fig, savePath, "Resolution", 180);
close(fig);
end

function plot_threshold_sweep(results, thresholds, savePath)
colors = lines(numel(results));
fig = figure("Name", "Knock-Down Force Threshold Sweep", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");

nexttile;
hold on;
for i = 1:numel(results)
    plot(results{i}.force.t, results{i}.force.normal, ...
        "LineWidth", 1.15, "Color", colors(i, :));
end
grid on;
xlabel("t / s");
ylabel("normal force / N");
title("Actual contact-force curves under different knock-down thresholds");
legend(threshold_labels(thresholds), "Location", "eastoutside");

nexttile;
hold on;
for i = 1:numel(results)
    plot(results{i}.traj.t, results{i}.ee.velocity(:, 3), ...
        "LineWidth", 1.15, "Color", colors(i, :));
end
yline(0, ":", "LineWidth", 0.8);
grid on;
xlabel("t / s");
ylabel("end-effector z velocity / (m/s)");
title("End-effector normal velocity");

exportgraphics(fig, savePath, "Resolution", 180);
close(fig);
end

function plot_threshold_summary(thresholdTable, savePath)
fig = figure("Name", "Knock-Down Force Threshold Summary", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");

nexttile;
plot(thresholdTable.threshold_N, thresholdTable.peak_force_N, "-o", ...
    "LineWidth", 1.4, "MarkerSize", 6);
hold on;
plot(thresholdTable.threshold_N, thresholdTable.force_target_N, "--s", ...
    "LineWidth", 1.2, "MarkerSize", 5);
grid on;
xlabel("knock-down threshold / N");
ylabel("force / N");
title("Force target and actual peak force");
legend({"peak force", "target force"}, "Location", "eastoutside");

nexttile;
plot(thresholdTable.threshold_N, thresholdTable.max_penetration_mm, "-o", ...
    "LineWidth", 1.4, "MarkerSize", 6);
hold on;
plot(thresholdTable.threshold_N, thresholdTable.threshold_duration_ms, "-s", ...
    "LineWidth", 1.2, "MarkerSize", 5);
grid on;
xlabel("knock-down threshold / N");
ylabel("depth / mm, duration / ms");
title("Compression depth and above-threshold duration");
legend({"max penetration / mm", "above-threshold duration / ms"}, ...
    "Location", "eastoutside");

exportgraphics(fig, savePath, "Resolution", 180);
close(fig);
end

function labels = case_labels(cases)
labels = strings(numel(cases), 1);
for i = 1:numel(cases)
    labels(i) = strrep(string(cases(i).case_name), "_", " ");
end
end

function labels = threshold_labels(thresholds)
labels = strings(numel(thresholds), 1);
for i = 1:numel(thresholds)
    labels(i) = sprintf("threshold %.0f N", thresholds(i));
end
end
