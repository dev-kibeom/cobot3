import numpy as np
import pinocchio as pin


class PinocchioCore:
    """
    Pinocchio 기반 로봇 기구학(FK/IK) 통합 코어 모듈
    """

    def __init__(self, urdf_path: str, ee_frame_name: str = "link_6"):
        # 1. 모델 및 데이터 객체 생성
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        if not self.model.existFrame(ee_frame_name):
            raise ValueError(f"URDF에 '{ee_frame_name}' 프레임이 존재하지 않습니다.")

        self.ee_frame_id = self.model.getFrameId(ee_frame_name)

    def get_fk_pose(self, q: np.ndarray, frame_id: int = None) -> pin.SE3:
        """현재 관절 각도(q)를 입력하면 해당 프레임의 3D 뼈대 위치/회전 반환 (순기구학)"""
        if frame_id is None:
            frame_id = self.ee_frame_id

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        return self.data.oMf[frame_id]

    def solve_ik(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray,
        initial_q: np.ndarray,
        eps=1e-4,
        it_max=1000,
        dt=0.1,
        damp=1e-6,
    ):
        """Newton-Raphson 수치해석적 역기구학(IK)"""
        oMdes = pin.SE3(target_rot, target_pos)
        q = np.copy(initial_q)

        for i in range(it_max):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[self.ee_frame_id]

            dMf = oMdes.actInv(oMf)
            err = pin.log(dMf).vector

            if np.linalg.norm(err) < eps:
                return True, q

            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL
            )
            v = -J.T @ np.linalg.inv(J @ J.T + damp * np.eye(6)) @ err
            q = pin.integrate(self.model, q, v * dt)

        return False, q
