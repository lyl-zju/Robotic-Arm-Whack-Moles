function pointWorld = board_to_world(pointBoard, cfgBoard)
pointBoard = pointBoard(:);
if numel(pointBoard) == 2
    pointBoard = [pointBoard; 0];
end
pointWorld = cfgBoard.R_world_board * pointBoard + cfgBoard.center_world(:);
pointWorld = pointWorld(:).';
end

