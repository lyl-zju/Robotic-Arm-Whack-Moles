function target = read_latest_udp_board_target(receiver, cfgBoard)
target = [];

while receiver.NumDatagramsAvailable > 0
    datagram = read(receiver, 1, "uint8");
    raw = char(datagram.Data(:).');

    try
        payload = jsondecode(raw);
    catch err
        warning("read_latest_udp_board_target:MalformedJson", ...
            "Dropped malformed UDP packet: %s", err.message);
        continue;
    end

    try
        parsedTarget = parse_udp_board_payload(payload, cfgBoard);
    catch err
        warning("read_latest_udp_board_target:InvalidTarget", ...
            "Dropped invalid UDP target packet: %s", err.message);
        continue;
    end

    if ~isempty(parsedTarget)
        target = parsedTarget;
    end
end
end

function target = parse_udp_board_payload(payload, cfgBoard)
target = [];
if ~isfield(payload, "valid") || ~logical(payload.valid)
    return;
end

if isfield(payload, "target_id")
    target = board_target_from_id(payload.target_id, cfgBoard);
elseif isfield(payload, "id")
    target = board_target_from_id(payload.id, cfgBoard);
elseif isfield(payload, "row") && isfield(payload, "col")
    targetId = (double(payload.row) - 1) * cfgBoard.cols + double(payload.col);
    target = board_target_from_id(targetId, cfgBoard);
elseif isfield(payload, "board")
    target = nearest_board_target(numeric_vector_field(payload, "board", 2), cfgBoard);
elseif isfield(payload, "world")
    worldPoint = numeric_vector_field(payload, "world", 3);
    boardPoint = world_to_board_point(worldPoint, cfgBoard);
    target = nearest_board_target(boardPoint, cfgBoard);
else
    error("parse_udp_board_payload:MissingTarget", ...
        "Packet must contain target_id, id, row/col, board, or world.");
end

target.seq = numeric_scalar_field(payload, "seq", NaN);
target.timestamp = numeric_scalar_field(payload, "timestamp", NaN);
target.source = string_field(payload, "source", "unknown_udp_source");
target.raw_payload = payload;
end

function target = nearest_board_target(boardPoint, cfgBoard)
boardPoint = boardPoint(:).';
if numel(boardPoint) < 2 || any(~isfinite(boardPoint(1:2)))
    error("nearest_board_target:InvalidBoardPoint", ...
        "Board point must contain two finite coordinates.");
end

boardPoint = boardPoint(1:2);
d2 = sum((cfgBoard.holes_board - boardPoint).^2, 2);
[~, targetId] = min(d2);
target = board_target_from_id(targetId, cfgBoard);
target.requested_board = boardPoint;
end

function boardPoint = world_to_board_point(worldPoint, cfgBoard)
worldPoint = worldPoint(:);
if numel(worldPoint) < 3 || any(~isfinite(worldPoint(1:3)))
    error("world_to_board_point:InvalidWorldPoint", ...
        "World point must contain three finite coordinates.");
end

boardPoint3 = cfgBoard.R_world_board.' * ...
    (worldPoint(1:3) - cfgBoard.center_world(:));
boardPoint = boardPoint3(1:2).';
end

function value = numeric_scalar_field(payload, fieldName, defaultValue)
if ~isfield(payload, fieldName)
    value = defaultValue;
    return;
end

value = double(payload.(fieldName));
if ~isscalar(value) || ~isfinite(value)
    value = defaultValue;
end
end

function value = numeric_vector_field(payload, fieldName, minLength)
value = double(payload.(fieldName));
value = value(:).';
if numel(value) < minLength || any(~isfinite(value(1:minLength)))
    error("numeric_vector_field:InvalidField", ...
        "Field %s must contain at least %d finite numbers.", fieldName, minLength);
end
value = value(1:minLength);
end

function value = string_field(payload, fieldName, defaultValue)
if isfield(payload, fieldName)
    value = string(payload.(fieldName));
else
    value = string(defaultValue);
end
end
