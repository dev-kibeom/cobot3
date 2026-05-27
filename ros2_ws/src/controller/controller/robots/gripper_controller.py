class GripperController:
    """그리퍼의 상태와 마스터 관절을 관리하는 모듈"""

    def __init__(self, joint_name="finger_joint", open_val=0.0, close_val=1.0):
        self.joint_name = joint_name
        self.open_val = open_val
        self.close_val = close_val
        self.current_val = open_val

    def open(self):
        self.current_val = self.open_val
        return self.joint_name, self.current_val

    def close(self):
        self.current_val = self.close_val
        return self.joint_name, self.current_val
