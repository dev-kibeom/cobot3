-- =============================================================================
-- factory_db 전체 스키마 + 데이터 (실제 테이블 구조 기준)
-- 팀장님 PC에서: mysql -u root -p < factory_db_export.sql
-- =============================================================================

CREATE DATABASE IF NOT EXISTS factory_db
  DEFAULT CHARACTER SET utf8mb4
  DEFAULT COLLATE utf8mb4_unicode_ci;

USE factory_db;

-- =============================================================================
-- 테이블 생성
-- =============================================================================

CREATE TABLE IF NOT EXISTS unit_master (
    unit_id   INT PRIMARY KEY,
    unit_name VARCHAR(20) NOT NULL
);

CREATE TABLE IF NOT EXISTS material_master (
    mat_code  INT          PRIMARY KEY,
    mat_name  VARCHAR(50)  NOT NULL
);

CREATE TABLE IF NOT EXISTS material_stock (
    mat_code   INT         PRIMARY KEY,
    quantity   FLOAT       NOT NULL DEFAULT 0.0,
    unit_id    INT         NOT NULL,
    max_qty    FLOAT       DEFAULT 700.0,
    updated_at DATETIME    DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (mat_code) REFERENCES material_master(mat_code),
    FOREIGN KEY (unit_id)  REFERENCES unit_master(unit_id)
);

CREATE TABLE IF NOT EXISTS part_master (
    part_code INT         PRIMARY KEY,
    part_name VARCHAR(50) NOT NULL
);

CREATE TABLE IF NOT EXISTS part_stock (
    part_code  INT         PRIMARY KEY,
    quantity   FLOAT       NOT NULL DEFAULT 0.0,
    unit_id    INT         NOT NULL,
    max_qty    FLOAT       DEFAULT 700.0,
    updated_at DATETIME    DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (part_code) REFERENCES part_master(part_code),
    FOREIGN KEY (unit_id)   REFERENCES unit_master(unit_id)
);

-- =============================================================================
-- 데이터 삽입
-- =============================================================================

INSERT INTO unit_master (unit_id, unit_name) VALUES
    (1, '개'),
    (2, 'g')
ON DUPLICATE KEY UPDATE unit_name = VALUES(unit_name);

INSERT INTO material_master (mat_code, mat_name) VALUES
    (1, 'metal_cube'),
    (2, 'metal_panel')
ON DUPLICATE KEY UPDATE mat_name = VALUES(mat_name);

-- metal_cube 700개, metal_panel 0개 (초기 상태)
INSERT INTO material_stock (mat_code, quantity, unit_id, max_qty) VALUES
    (1, 700.0, 1, 700.0),
    (2,   0.0, 1, 700.0)
ON DUPLICATE KEY UPDATE
    quantity   = VALUES(quantity),
    unit_id    = VALUES(unit_id),
    max_qty    = VALUES(max_qty),
    updated_at = NOW();

INSERT INTO part_master (part_code, part_name) VALUES
    (1, 'metal_panel'),
    (2, 'final_product')
ON DUPLICATE KEY UPDATE part_name = VALUES(part_name);

-- 생산품 재고 초기값 전부 0
INSERT INTO part_stock (part_code, quantity, unit_id, max_qty) VALUES
    (1, 0.0, 1, 700.0),
    (2, 0.0, 1, 70000.0)
ON DUPLICATE KEY UPDATE
    quantity   = VALUES(quantity),
    unit_id    = VALUES(unit_id),
    max_qty    = VALUES(max_qty),
    updated_at = NOW();

-- =============================================================================
-- 확인
-- =============================================================================

SELECT 'unit_master'     AS tbl, COUNT(*) AS rows FROM unit_master     UNION ALL
SELECT 'material_master',         COUNT(*)         FROM material_master UNION ALL
SELECT 'material_stock',          COUNT(*)         FROM material_stock  UNION ALL
SELECT 'part_master',             COUNT(*)         FROM part_master     UNION ALL
SELECT 'part_stock',              COUNT(*)         FROM part_stock;

SELECT mm.mat_code, mm.mat_name, ms.quantity, ms.max_qty
FROM material_master mm JOIN material_stock ms ON ms.mat_code = mm.mat_code
ORDER BY mm.mat_code;

SELECT pm.part_code, pm.part_name, ps.quantity, ps.max_qty
FROM part_master pm JOIN part_stock ps ON ps.part_code = pm.part_code
ORDER BY pm.part_code;
