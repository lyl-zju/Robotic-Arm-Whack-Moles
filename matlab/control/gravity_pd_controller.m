function tau = gravity_pd_controller(robot, q, qd, qDes, qdDes, cfgController)
tau = pd_controller(q, qd, qDes, qdDes, cfgController);
try
    tau = tau + gravityTorque(robot, q);
catch
    warning("gravityTorque failed. Falling back to plain PD.");
end
end

