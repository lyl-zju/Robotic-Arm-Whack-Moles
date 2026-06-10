function target = generate_random_target(cfgBoard)
target.id = randi(size(cfgBoard.holes_board, 1));
target.position_board = cfgBoard.holes_board(target.id, :);
target.position_world = cfgBoard.holes_world(target.id, :);
target.appear_time = tic;
target.mode = "random";
end

