# 基于牙模区域与参数生成刷牙路径
class TrajectoryPlanner:
    def __init__(self, pointcloud, tool_offset):
        self.pcd = pointcloud
        self.offset = tool_offset

    def plan_brushing_path(self, region_id, frequency="中", duration=10):
        # 简化逻辑：生成沿牙模边缘扫过的路径点
        path = []
        region_points = self.extract_region_points(region_id)
        for pt in region_points:
            adjusted_pt = self.apply_offset(pt)
            path.append(adjusted_pt)
        return path

    def extract_region_points(self, region_id):
        # 示例函数：实际需在点云中提取对应区域
        return self.pcd[region_id]

    def apply_offset(self, pt):
        # 根据牙刷末端工具的几何偏移做转换
        return [pt[0], pt[1], pt[2] + self.offset]
