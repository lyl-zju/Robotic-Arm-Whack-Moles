function tau = pd_controller(q, qd, qDes, qdDes, cfgController)
tau = cfgController.Kp .* (qDes - q) + cfgController.Kd .* (qdDes - qd);
end

