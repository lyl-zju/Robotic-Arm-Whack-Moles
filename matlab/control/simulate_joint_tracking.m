function tracking = simulate_joint_tracking(traj, cfgController)
dt = mean(diff(traj.t));
q = traj.q_des(1, :);
qd = zeros(size(q));

qActual = zeros(size(traj.q_des));
qdActual = zeros(size(traj.q_des));

for i = 1:numel(traj.t)
    qDes = traj.q_des(i, :);
    qdDes = traj.qd_des(i, :);

    qdd = cfgController.Kp .* (qDes - q) + cfgController.Kd .* (qdDes - qd);
    qd = qd + qdd * dt;
    q = q + qd * dt;

    qActual(i, :) = q;
    qdActual(i, :) = qd;
end

tracking.q_actual = qActual;
tracking.qd_actual = qdActual;
tracking.error = traj.q_des - qActual;
tracking.rmse = sqrt(mean(tracking.error(:).^2));
tracking.max_abs_error = max(abs(tracking.error(:)));
end
