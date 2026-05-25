import asyncio
import numpy as np
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.prims import delete_prim


async def dynamic_object_test():
    # 1. 이미 활성화된 World(현재 씬) 가져오기
    world = World.instance()
    if world is None:
        world = World()

    cube_path = "/World/TargetCube"
    cube_name = "my_target_cube"

    # 혹시 기존 테스트의 잔해가 남아있다면 깔끔하게 삭제
    if omni.usd.get_context().get_stage().GetPrimAtPath(cube_path):
        if world.scene.object_exists(cube_name):
            world.scene.remove_object(cube_name)
        delete_prim(cube_path)

    print("[INFO] 1단계: 허공에 빨간색 동적 큐브 생성")
    cube = DynamicCuboid(
        prim_path=cube_path,
        name=cube_name,
        position=np.array([1.0, 0.0, 1.0]),  # 허공에 스폰 (재생 중이면 바닥으로 떨어짐)
        scale=np.array([0.5, 0.5, 0.5]),  # 5cm 크기
        color=np.array([1.0, 0.0, 0.0]),  # 빨간색
        mass=0.5,
    )
    world.scene.add(cube)

    # GUI가 멈추지 않게 비동기로 3초 대기
    await asyncio.sleep(3.0)

    print("[INFO] 2단계: 큐브 위치 강제 이동 (순간이동)")
    # 로봇 팔 앞쪽 등 원하는 좌표로 강제 텔레포트
    cube.set_world_pose(position=np.array([0.5, 0.0, 0.5]))

    await asyncio.sleep(3.0)

    print("[INFO] 3단계: 씬에서 큐브 완전 삭제")
    world.scene.remove_object(cube_name)
    delete_prim(cube_path)
    print("[INFO] 동적 객체 테스트 완료!")


# Isaac Sim의 메인 이벤트 루프에 비동기 태스크 던지기
asyncio.ensure_future(dynamic_object_test())
