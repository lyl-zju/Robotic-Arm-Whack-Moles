function target = read_target_json(path)
if ~exist(path, "file")
    error("Target file does not exist: %s", path);
end

raw = fileread(path);
target = jsondecode(raw);

if ~isfield(target, "valid")
    error("target.json is missing required field: valid");
end

if target.valid
    if ~isfield(target, "board") && ~isfield(target, "world")
        error("Valid target must contain board or world coordinates.");
    end
end
end

