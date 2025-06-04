import cv2
import numpy as np
# 清洁效果评估（图像对比分析）
class CleaningEvaluator:
    def __init__(self):
        pass

    def evaluate_cleanliness(self, before_img_path, after_img_path):
        before = cv2.imread(before_img_path, 0)
        after = cv2.imread(after_img_path, 0)
        diff = cv2.absdiff(before, after)
        score = 1.0 - np.mean(diff) / 255.0
        return round(score, 3)