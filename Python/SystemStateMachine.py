from enum import Enum
# 控制任务流程状态跳转（基于 enum 或 FSM）
class SystemState(Enum):
    IDLE = 0
    CONNECTING = 1
    READY = 2
    TOOL_CHANGE = 3
    CALIBRATING = 4
    SCANNING = 5
    BRUSHING = 6
    EVALUATING = 7
    ERROR = -1

class SystemStateMachine:
    def __init__(self):
        self.state = SystemState.IDLE

    def transition(self, new_state):
        print(f"[状态切换] {self.state.name} -> {new_state.name}")
        self.state = new_state

    def get_state(self):
        return self.state