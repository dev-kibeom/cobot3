"본 레포지토리를 clone한 후, 반드시 드라이브에 있는 assets.zip 파일을 다운로드하여 isaac_envs/assets/ 경로에 압축을 풀어주세요. 그렇지 않으면 시뮬레이션 환경이 로드되지 않습니다."

"isaac sim script는 import 순서 중요합니다."

"assembler 작동 안함. 직접 assemble 후 usd 파일로 만들어서 import할 것"
"vscode intergrate extension 키고 실행"

참고: settings.json
{
    "python.defaultInterpreterPath": "/home/kibeom/dev_ws2/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh", 
    "python.analysis.extraPaths": [
        "/home/kibeom/dev_ws2/isaac_sim/isaacsim/_build/linux-x86_64/release/exts",
        "/home/kibeom/dev_ws2/isaac_sim/isaacsim/source/extensions"
    ],
    "python.analysis.typeCheckingMode": "basic",
    "python.analysis.diagnosticSeverityOverrides": {
        "reportMissingModuleSource": "none"
    }
}