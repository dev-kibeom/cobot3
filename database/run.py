import uvicorn
from app.core import config

if __name__ == "__main__":
    # app 패키지의 main.py 안에 있는 app 객체를 실행하라는 의미입니다.
    uvicorn.run(
        "app.main:app", host=config.SERVER_HOST, port=config.SERVER_PORT, reload=True
    )
