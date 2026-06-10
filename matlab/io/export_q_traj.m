function export_q_traj(traj, path)
q = traj.q_des;
names = ["t", "q" + string(1:size(q, 2))];
data = array2table([traj.t, q], "VariableNames", cellstr(names));
writetable(data, path);
end

