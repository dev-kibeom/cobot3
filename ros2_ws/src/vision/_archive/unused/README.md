# Archived (unused) scripts

이 폴더의 스크립트들은 현재 빌드/런타임 흐름에서 사용되지 않습니다.
삭제하지 않고 참고용으로 보관하기 위해 옮겨두었습니다.

## pixel_to_world.py
초기 프로토타입. `/m0609/camera/info`, `/vision/target_point` 토픽을 사용하며 현재
파이프라인(`/m0609/vision/plate_obb`, `/m0609/vision/pick_place_goal` 등)과 토픽
네이밍이 다릅니다. `setup.py` entry_points에도 등록돼 있지 않고, 다른 모듈에서
import 하지 않습니다.

## publish_fake_goal.py
수동 테스트용 fake `pick_place_goal` publisher. 어디서도 import 되지 않고
`setup.py` entry_points에도 등록돼 있지 않습니다. 문서에서 참조하는 곳도 없습니다.
필요할 때 직접 `python3 publish_fake_goal.py ...` 형태로 실행하던 보조 스크립트로
보입니다.
