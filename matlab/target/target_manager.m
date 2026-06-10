function state = target_manager(state, newTarget, mode)
if nargin < 3
    mode = "preempt";
end

if isempty(state)
    state.queue = {};
    state.active = [];
end

switch mode
    case "queue"
        state.queue{end + 1} = newTarget;
        if isempty(state.active)
            state.active = state.queue{1};
            state.queue(1) = [];
        end
    case "preempt"
        state.active = newTarget;
        state.queue = {};
    otherwise
        error("Unknown target manager mode: %s", mode);
end
end

