function target = board_target_from_id(targetId, cfgBoard)
targetId = double(targetId);
numTargets = size(cfgBoard.holes_board, 1);

if ~isscalar(targetId) || ~isfinite(targetId) || ...
        targetId ~= round(targetId) || targetId < 1 || targetId > numTargets
    error("board_target_from_id:InvalidTargetId", ...
        "Target id must be an integer in [1, %d].", numTargets);
end

targetId = round(targetId);
target.id = targetId;
target.row = ceil(targetId / cfgBoard.cols);
target.col = mod(targetId - 1, cfgBoard.cols) + 1;
target.position_board = cfgBoard.holes_board(targetId, :);
target.position_world = cfgBoard.holes_world(targetId, :);
end
