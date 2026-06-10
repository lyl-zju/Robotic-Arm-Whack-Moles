function draw_board(cfgBoard, targetBoard)
hold on; axis equal; grid on;
xlim([-cfgBoard.width / 2, cfgBoard.width / 2]);
ylim([-cfgBoard.height / 2, cfgBoard.height / 2]);
rectangle("Position", [-cfgBoard.width/2, -cfgBoard.height/2, cfgBoard.width, cfgBoard.height], ...
    "FaceColor", [0.92 0.92 0.92], "EdgeColor", [0.2 0.2 0.2], "LineWidth", 1.5);

for i = 1:size(cfgBoard.holes_board, 1)
    p = cfgBoard.holes_board(i, :);
    plot(p(1), p(2), "ko", "MarkerSize", 12, "LineWidth", 1.5);
    text(p(1), p(2) - 0.018, string(i), "HorizontalAlignment", "center");
end

if ~isempty(targetBoard)
    plot(targetBoard(1), targetBoard(2), "ro", "MarkerSize", 16, "LineWidth", 2.5);
end

xlabel("board x / m");
ylabel("board y / m");
end

