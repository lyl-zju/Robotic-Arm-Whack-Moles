function target = get_mouse_target(cfgBoard)
figure("Name", "Click target on board");
draw_board(cfgBoard, []);
title("Click a target position");
[x, y] = ginput(1);

target.id = 0;
target.position_board = [x, y];
target.position_world = board_to_world(target.position_board, cfgBoard);
target.appear_time = tic;
target.mode = "mouse";
end

