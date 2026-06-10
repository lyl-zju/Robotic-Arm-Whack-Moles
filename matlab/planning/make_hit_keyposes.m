function keyposes = make_hit_keyposes(targetWorld, cfgBoard)
targetWorld = targetWorld(:).';
keyposes.hover = [targetWorld(1), targetWorld(2), cfgBoard.z_hover];
keyposes.hit = [targetWorld(1), targetWorld(2), cfgBoard.z_hit];
keyposes.back = keyposes.hover;
end

