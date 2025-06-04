#include "SystemStateMachine.hpp"

SystemStateMachine::SystemStateMachine() : state(SystemState::IDLE) {}

void SystemStateMachine::transition(SystemState new_state) {
    std::cout << "[状态切换] " << static_cast<int>(state) << " -> " << static_cast<int>(new_state) << std::endl;
    state = new_state;
}

SystemState SystemStateMachine::getState() const {
    return state;
} 