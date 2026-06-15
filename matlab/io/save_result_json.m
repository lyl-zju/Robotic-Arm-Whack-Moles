function save_result_json(result, path)
summary.hit_success = result.hit_success;
summary.hit_error = result.hit_error;
summary.response_time = result.response_time;
summary.controller = result.controller;
summary.path_type = result.path_type;
if isfield(result, "target_mode")
    summary.target_mode = result.target_mode;
end
if isfield(result, "target_id")
    summary.target_id = result.target_id;
end
if isfield(result, "motion_mode")
    summary.motion_mode = result.motion_mode;
end
summary.target_world = result.target_world;
if isfield(result, "target_board")
    summary.target_board = result.target_board;
end
summary.tracking_rmse = result.tracking.rmse;
summary.tracking_max_abs_error = result.tracking.max_abs_error;
if isfield(result, "force")
    summary.knocked_down = result.force.knocked_down;
    summary.force_threshold = result.force.threshold;
    if isfield(result.force, "target")
        summary.force_target = result.force.target;
    end
    summary.peak_force = result.force.peak_force;
    summary.contact_duration = result.force.contact_duration;
    summary.threshold_duration = result.force.threshold_duration;
    summary.max_penetration = result.force.max_penetration;
    summary.min_xy_error = result.force.min_xy_error;
    if isfield(result.force, "first_contact_time") && isfinite(result.force.first_contact_time)
        summary.first_contact_time = result.force.first_contact_time;
    end
    if isfield(result.force, "success_time") && isfinite(result.force.success_time)
        summary.success_time = result.force.success_time;
    end
end
if isfield(result, "control") && isfield(result.control, "cfg")
    summary.impact_dt = result.control.cfg.dt;
    summary.impact_speed = result.control.cfg.impact_speed;
    summary.contact_model = result.control.cfg.contact_model;
    summary.contact_stiffness = result.control.cfg.contact_stiffness;
    summary.contact_damping = result.control.cfg.contact_damping;
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
