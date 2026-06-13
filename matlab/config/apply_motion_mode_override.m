function cfg = apply_motion_mode_override(cfg, requestedMode)
if ~isfield(cfg, "motion_mode")
    cfg.motion_mode = normalize_motion_mode("");
end

if nargin < 2 || isempty(requestedMode) || strlength(string(requestedMode)) == 0
    cfg.motion_mode = normalize_motion_mode(cfg.motion_mode);
    return;
end

cfg.motion_mode = normalize_motion_mode(requestedMode);
end
