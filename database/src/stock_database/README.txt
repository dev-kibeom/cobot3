# 스마트 팩토리 — 물류 DB 시스템

Isaac Sim 기반 스마트 팩토리의 재고 관리 시스템입니다.  
R plumber API 서버가 MySQL(재고 수량)과 Machbase(변동 이력)를 연동하며,  
Isaac Sim과 HTTP로 통신합니다.

---

## 시스템 구성

```
Isaac Sim (Python)
    │  POST /check_work, /update_stock
    ▼
R 서버 (factory_server_V2_1.R)  ← 이 저장소
    ├── MySQL       : 재고 현황 (현재 수량)
    └── Machbase    : 변동 이력 (시계열)
```

---

## 사전 요구사항

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 22.04 |
| R | 4.x 이상 |
| MySQL | 8.x |
| Python | 3.10 이상 |
| Machbase Neo | v8.5.2 |

---

## 설치 순서

### 1. MySQL 설치 및 설정

```bash
# MySQL 설치
sudo apt-get update
sudo apt-get install -y mysql-server

# MySQL 시작
sudo systemctl start mysql
sudo systemctl enable mysql

# DB 및 계정 생성
sudo mysql << SQL
CREATE DATABASE IF NOT EXISTS factory_db
  CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci;

CREATE USER 'rokey'@'localhost' IDENTIFIED BY 'rokey1234';
GRANT ALL PRIVILEGES ON factory_db.* TO 'rokey'@'localhost';
FLUSH PRIVILEGES;
SQL
```

### 2. MySQL 테이블 초기화

```bash
# 저장소 루트에서 실행
sudo mysql factory_db < src/factory_master_V2.sql
```

정상 실행 시 아래 테이블이 생성됩니다.

```
unit_master / material_master / part_master
material_stock / part_stock
```

### 3. Machbase 설치

```bash
# 설치 스크립트 실행
sh -c "$(curl -fsSL https://docs.machbase.com/install.sh)"

# 설치된 디렉토리로 이동 (버전명은 다를 수 있음)
cd ~/machbase-neo-v8.5.2-linux-amd64

# 서버 시작
./machbase-neo serve &

# 접속 확인
curl http://127.0.0.1:5654/db/query?q=SELECT+1
# {"success":true, ...} 가 뜨면 정상
```

### 4. Machbase 히스토리 테이블 초기화

```bash
./machbase-neo shell
```

shell 접속 후 아래 쿼리를 실행합니다.  
> ⚠️ Machbase shell에서는 `--` 주석을 사용할 수 없습니다. 쿼리만 입력하세요.

```sql
DROP TABLE IF EXISTS material_history;
DROP TABLE IF EXISTS part_history;

CREATE TAG TABLE material_history (
    name         VARCHAR(40)  PRIMARY KEY,
    time         DATETIME     BASETIME,
    mat_code     SHORT,
    consumed_qty DOUBLE,
    qty_before   DOUBLE,
    qty_after    DOUBLE
);

CREATE TAG TABLE part_history (
    name         VARCHAR(40)  PRIMARY KEY,
    time         DATETIME     BASETIME,
    part_code    SHORT,
    consumed_qty DOUBLE,
    qty_before   DOUBLE,
    qty_after    DOUBLE
);
```

확인:
```sql
show tables;
```

`MATERIAL_HISTORY`, `PART_HISTORY` 두 개가 보이면 완료입니다.

### 5. R 패키지 설치

```r
install.packages(c("plumber", "jsonlite", "DBI", "RMySQL", "later"))
```

### 6. Python 패키지 설치

```bash
pip install textual mysql-connector-python
```

---

## 실행

```bash
cd src
Rscript factory_server_V2_1.R
```

정상 기동 시 출력:

```
[ OK ]  MySQL 연결 성공
[ OK ]  Machbase 연결 성공
[ OK ]  Machbase watchdog started (interval: 10s)
================================================
  FACTORY SERVER  (MySQL + Machbase)  port:8765
================================================
  MATERIAL STOCK
  CODE  NAME         CURRENT    MAX   UNIT
  1     metal_cube   500.0   2000.0   개   [  OK]

  PART STOCK
  CODE  NAME         CURRENT    MAX   UNIT
  1     metal_part     0.0   2000.0   개   [EMPTY]
[ OK ]  Monitor launched
```

모니터링 TUI가 별도 창으로 자동 실행됩니다.

종료:
```
Ctrl+C  →  R 서버 + 모니터 창 동시 종료
```

---

## 설정값 변경

`src/factory_server_V2_1.R` 상단에서 수정합니다.

```r
MYSQL <- list(
  host     = "127.0.0.1",
  port     = 3306,
  dbname   = "factory_db",
  user     = "rokey",        # DB 계정명
  password = "rokey1234"     # DB 비밀번호
)

MACHBASE <- list(
  host = "127.0.0.1",
  port = 5654,
  exe  = "~/machbase-neo-v8.5.2-linux-amd64/machbase-neo"  # 실행 파일 경로
)
```

---

## API 엔드포인트

베이스 URL: `http://127.0.0.1:8765`

| 메서드 | 엔드포인트 | 역할 |
|---|---|---|
| GET | `/health` | 서버 생존 확인 |
| GET | `/stock` | 전체 재고 현황 조회 |
| GET | `/history` | 변동 이력 최근 50건 |
| POST | `/check_work` | 작업 허가 요청 |
| POST | `/update_stock` | 재고 차감 + 부품 생산 기록 |
| POST | `/set_stock` | 수량 직접 수정 (관리자용) |

### /check_work

Carter가 제조 설비 도착 시 호출합니다.

```json
// 요청
{ "robot_id": 1, "part_code": 1, "prod_qty": 5 }

// 응답 (허가)
{ "approved": true, "reason": "OK" }

// 응답 (거부)
{ "approved": false, "reason": "part_code not found: 99" }
```

### /update_stock

M0609 가공 완료 시 호출합니다.

```json
// 요청
{
  "robot_id":  1,
  "part_code": 1,
  "prod_qty":  5,
  "materials": [{ "mat_code": 1, "used_qty": 1.0 }]
}

// 응답
{ "ok": true, "updated_mat_codes": [1], "added_qty": 5 }
```

### /set_stock

수량 직접 수정 (관리자용, Isaac Sim Extension UI에서 사용).

```json
// 요청
{ "type": "material", "code": 1, "quantity": 500.0 }

// 응답
{ "ok": true, "type": "material", "code": 1, "quantity": 500.0 }
```

---

## 파일 구조

```
src/
├── factory_server_V2_1.R    # R plumber API 서버 (메인 실행 파일)
├── factory_master_V2.sql    # MySQL DB 초기화 스크립트
├── monitor_V2.py            # Textual TUI 실시간 모니터
├── fake_factory_sim_V2.py   # 공정 시뮬레이션 (수동 실행)
└── db_test_V2.py            # ROS2 연동 테스트 노드
```

---

## 시뮬레이션 (선택)

R 서버와 별개로 가상 공정 시뮬레이션을 실행할 수 있습니다.

```bash
python3 src/fake_factory_sim_V2.py
```

구성: 두산 M0609 × 6대, Nova Carter × 2대  
공정: `metal_cube × 1` → `metal_part × 5`

---

## 주의사항

- MySQL root 계정은 `auth_socket` 방식이므로 별도 계정(`rokey`)을 사용합니다
- Machbase shell에서 `--` 주석은 동작하지 않습니다
- Machbase 실행 파일 경로(`MACHBASE$exe`)가 실제 설치 경로와 일치해야 합니다
