function save_result_json(result, path)
summary.hit_success = result.hit_success;
summary.hit_error = result.hit_error;
summary.response_time = result.response_time;
summary.controller = result.controller;
summary.path_type = result.path_type;
if isfield(result, "motion_mode")
    summary.motion_mode = result.motion_mode;
end
summary.target_world = result.target_world;
summary.tracking_rmse = result.tracking.rmse;
summary.tracking_max_abs_error = result.tracking.max_abs_error;
if isfield(result, "force")
    summary.knocked_down = result.force.knocked_down;
    summary.force_threshold = result.force.threshold;
    summary.peak_force = result.force.peak_force;
    summary.contact_duration = result.force.contact_duration;
    summary.threshold_duration = result.force.threshold_duration;
    summary.max_penetration = result.force.max_penetration;
    summary.min_xy_error = result.force.min_xy_error;
end

try
    text = jsonencode(summary, "PrettyPrint", true);
catch
    text = jsonencode(summary);
end
fid = fopen(path, "w");
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", text);
end
