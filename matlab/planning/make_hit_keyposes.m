function keyposes = make_hit_keyposes(targetWorld, cfgBoard, cfgForce)
if nargin < 3 || isempty(cfgForce)
    cfgForce.press_depth = 0;
end

targetWorld = targetWorld(:).';
keyposes.hover = [targetWorld(1), targetWorld(2), cfgBoard.z_hover];
keyposes.contact = [targetWorld(1), targetWorld(2), cfgBoard.z_hit];
keyposes.press = [targetWorld(1), targetWorld(2), cfgBoard.z_hit - cfgForce.press_depth];
keyposes.hit = keyposes.press;
keyposes.back = keyposes.hover;
end
