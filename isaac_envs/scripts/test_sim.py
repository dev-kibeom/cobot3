import os
import numpy as np
import omni.usd
import omni.graph.core as og

from isaacsim.core.api import World
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdPhysics

def find_prim_path_by_name(root_path, link_name):
    """USD Stage에서 특정 이름을 가진 Prim의 경로를 찾는 유틸리티 함수"""

    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)

    if not root_prim.IsValid():
        print(
            f"[ERROR] 로봇 계층 구조에서 {link_name}를 찾을 수 없습니다! 스크립트를 종료합니다."
        )
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == link_name:
            print(f"[INFO] prim {link_name}의 경로 자동 탐색 성공: {str(prim.GetPath())}")
            return str(prim.GetPath())
        
    return None

def main():
    if World.instance() is not None:
        World.instance().clear_instance()

    world = World()
    world.scene.add_default_ground_plane()

    # =========================================================================
    # Nucleus 클라우드 USD 로봇 스폰
    # =========================================================================
    print("[INFO] 로봇 에셋을 다운로드 및 배치 중입니다. (최초 실행 시 시간 소요)")

    jetbot_asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

    jetbot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Jetbot",
            name="my_jetbot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jetbot_asset_path,
            position=np.array([0.0, 1.0, 0.0]),  # y축으로 1m 띄워서 스폰
        )
    )

    # =========================================================================
    # 로컬 URDF 로봇 스폰 
    # =========================================================================

    # 두산 로봇 + 그리퍼 로드
    doosan_usd_path = (
        "/home/kibeom/smart_factory_project/isaac_envs/assets/m0609_rg2.usd"
    )
    add_reference_to_stage(usd_path=doosan_usd_path, prim_path="/World/Doosan")

    ee_path = find_prim_path_by_name("/World/Doosan", "link_6")

    # [수정 2] 만약 껍데기만 로드되어 ee_path가 None이라면 여기서 스크립트를 강제 종료!
    if ee_path is None:
        print(
            "\n[ERROR] link_6를 찾지 못했습니다! 파일 경로가 정확한지 확인해주세요.\n"
        )
        return  # <--- 이 return이 없으면 아래 코드가 실행되면서 또 뻗어버립니다.
    
    # =========================================================================
    # Articulation Root 설정
    # =========================================================================
    stage = omni.usd.get_context().get_stage()

    # 1. 최상위 부모(/World/Doosan)는 순수 그룹 폴더여야 하므로 완장이 있다면 뺏습니다.
    parent_prim = stage.GetPrimAtPath("/World/Doosan")
    if parent_prim.IsValid() and parent_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        parent_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    # 2. 로봇 본체(m0609)에게 유일한 물리 대장 완장을 확정적으로 씌워줍니다!
    m0609_prim = stage.GetPrimAtPath("/World/Doosan/m0609")
    if m0609_prim.IsValid():
        UsdPhysics.ArticulationRootAPI.Apply(m0609_prim)

    # 3. 그리퍼의 완장은 충돌 방지를 위해 확실히 제거합니다.
    gripper_prim = stage.GetPrimAtPath("/World/Doosan/m0609/onrobot_rg2ft")
    if gripper_prim.IsValid() and gripper_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        gripper_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

    # 그리퍼 컨트롤러 정의 (경로는 조립 후 USD 구조에 맞게 자동 매핑됩니다)
    rg2_gripper = ParallelGripper(
        end_effector_prim_path=ee_path,
        joint_prim_names=["finger_joint", "right_inner_knuckle_joint"],
        joint_opened_positions=np.array([0.0, 0.0]),
        joint_closed_positions=np.array([0.7, -0.7]),
        action_deltas=np.array([-0.7, 0.7]),
    )

    doosan_arm = world.scene.add(
        SingleManipulator(
            prim_path="/World/Doosan/m0609",
            name="doosan_rg2",
            end_effector_prim_path=ee_path,
            gripper=rg2_gripper,
            position=np.array([0.0, -1.0, 0.0]),
        )
    )

    print("\n[SUCCESS] 모든 로봇 에셋 배치 성공! GUI 화면에서 확인하세요.\n")

main()
