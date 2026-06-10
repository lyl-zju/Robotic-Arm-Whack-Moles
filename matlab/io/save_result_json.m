function save_result_json(result, path)
summary.hit_success = result.hit_success;
summary.hit_error = result.hit_error;
summary.response_time = result.response_time;
summary.controller = result.controller;
summary.path_type = result.path_type;
summary.target_world = result.target_world;
summary.tracking_rmse = result.tracking.rmse;
summary.tracking_max_abs_error = result.tracking.max_abs_error;

try
    text = jsonencode(summary, "PrettyPrint", true);
catch
    text = jsonencode(summary);
end
fid = fopen(path, "w");
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", text);
end
