function cfg = config_board()
cfg.width = 0.60;
cfg.height = 0.40;
cfg.rows = 3;
cfg.cols = 3;
cfg.x_list = [-0.20, 0, 0.20];
cfg.y_list = [-0.13, 0, 0.13];
cfg.center_world = [0.45, 0.0, 0.05];
cfg.R_world_board = eye(3);
cfg.z_hover = cfg.center_world(3) + 0.10;
cfg.z_hit = cfg.center_world(3) + 0.01;
cfg.hit_threshold = 0.015;
cfg.holes_board = make_holes(cfg);
cfg.holes_world = zeros(size(cfg.holes_board, 1), 3);
for i = 1:size(cfg.holes_board, 1)
    cfg.holes_world(i, :) = board_to_world(cfg.holes_board(i, :), cfg);
end
end

function holes = make_holes(cfg)
holes = zeros(cfg.rows * cfg.cols, 2);
k = 1;
for iy = 1:cfg.rows
    for ix = 1:cfg.cols
        holes(k, :) = [cfg.x_list(ix), cfg.y_list(iy)];
        k = k + 1;
    end
end
end
