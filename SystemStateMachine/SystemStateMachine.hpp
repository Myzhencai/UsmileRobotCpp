#pragma once
#include <iostream>

enum class SystemState {
    IDLE = 0,
    CONNECTING = 1,
    READY = 2,
    TOOL_CHANGE = 3,
    CALIBRATING = 4,
    SCANNING = 5,
    BRUSHING = 6,
    EVALUATING = 7,
    ERROR = -1
};

class SystemStateMachine {
public:
    SystemStateMachine();
    void transition(SystemState new_state);
    SystemState getState() const;

private:
    SystemState state;
}; 