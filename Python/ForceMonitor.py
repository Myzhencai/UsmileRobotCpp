# 实时读取六维力数据，检测过载，调整运动策略
class ForceMonitor:
    def __init__(self, force_sensor):
        self.sensor = force_sensor
        self.force_threshold = 5.0  # N

    def read_force(self):
        return self.sensor.get_force()

    def is_overloaded(self):
        force = self.read_force()
        return any(abs(f) > self.force_threshold for f in force)

    def adjust_trajectory(self, trajectory):
        # 根据力数据微调路径
        return trajectory