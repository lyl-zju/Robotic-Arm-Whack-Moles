function mode = normalize_motion_mode(mode)
if nargin < 1 || isempty(mode)
    mode = "hover_stop";
    return;
end

mode = lower(strtrim(string(mode)));

switch mode
    case {"hover_stop", "hover", "stop", "pause", "stop_hover", "悬停"}
        mode = "hover_stop";
    case {"continuous", "no_hover", "nohover", "smooth", "flow", "不悬停", "一气呵成"}
        mode = "continuous";
    otherwise
        error("Unknown hit motion mode: %s. Use hover_stop or continuous.", mode);
end
end
