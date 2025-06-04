# 控制机械臂运动，包括回零、抓夹、路径执行与避障
class ArmController:
    def __init__(self, sdk):
        self.sdk = sdk  # SDK接口对象

    def move_to_home(self):
        home_pose = [0, -90, 90, 0, 90, 0]  # 示例值
        success = self.sdk.move_joints(home_pose)
        return success

    def open_gripper(self):
        return self.sdk.control_gripper(open=True)

    def close_gripper(self):
        return self.sdk.control_gripper(open=False)

    def execute_trajectory(self, path_points):
        for pt in path_points:
            if not self.sdk.move_cartesian(pt):
                print("[执行中断] 点位不可达或受限")
                return False
        return True

    def plan_safe_path(self, target_pose):
        # 伪代码：实际应接入 RRT*/A* 等避障算法
        return [target_pose]
