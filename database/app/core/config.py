import os
from dotenv import load_dotenv

# 🚀 1. 이 config.py 파일의 위치를 기준으로 database/ 폴더의 절대 경로를 계산합니다.
# __file__ = database/app/core/config.py
CORE_DIR = os.path.dirname(os.path.abspath(__file__))  # database/app/core
APP_DIR = os.path.dirname(CORE_DIR)  # database/app
BASE_DIR = os.path.dirname(APP_DIR)  # database/ (프로젝트 루트)

# 🚀 2. 절대 경로를 기반으로 .env 파일 로드
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path=dotenv_path)

# ==========================================
# 환경 변수 매핑
# ==========================================
SERVER_HOST = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.getenv("SERVER_PORT", 8001))
MQTT_BROKER_IP = os.getenv("MQTT_BROKER_IP", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))

# 🚀 3. [안전장치] database/data/ 폴더가 없으면 코드가 스스로 자동으로 생성합니다.
DATA_DIR = os.path.join(BASE_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

# 🚀 4. 업계 표준: .env의 문자열을 무시하고, 강제로 완벽한 고정 절대 경로 DB 주소를 생성합니다.
DATABASE_URL = f"sqlite:///{os.path.join(DATA_DIR, 'smart_factory.db')}"
