function cfg = config_force()
cfg.enabled = true;
cfg.threshold = 10.0;        % N, minimum force required to knock the mole down
cfg.k_contact = 1000.0;      % N/m, virtual mole stiffness
cfg.c_contact = 12.0;        % N/(m/s), virtual contact damping
cfg.press_depth = 0.012;     % m, planned compression below the mole top
cfg.min_contact_time = 0.02; % s, force must stay above threshold for this long
cfg.max_force = 40.0;        % N, safety clamp for the virtual force signal
end
