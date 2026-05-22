-- =============================================================================
-- 자재/부품 마스터 테이블 (이름 정보 전용)
-- =============================================================================
-- 실시간 테이블 (material_stock, part_stock, material_history, part_history)
-- 은 건드리지 않음 — 숫자 코드만 유지
-- =============================================================================

USE factory_db;

-- ── 원자재 마스터 ────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS material_master (
  mat_code     SMALLINT     NOT NULL,
  mat_name     VARCHAR(50)  NOT NULL,
  PRIMARY KEY (mat_code)
);

-- ── 부품 마스터 ──────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS part_master (
  part_code    SMALLINT     NOT NULL,
  part_name    VARCHAR(50)  NOT NULL,
  PRIMARY KEY (part_code)
);

-- ── 단위 마스터 ──────────────────────────────────────────────────────────────
CREATE TABLE IF NOT EXISTS unit_master (
  unit_id       SMALLINT    NOT NULL,
  unit_name     VARCHAR(10) NOT NULL,
  PRIMARY KEY (unit_id)
);

-- ── 원자재 재고 ──────────────────────────────────────────────────────────────
CREATE TABLE material_stock (
  mat_code   SMALLINT      NOT NULL,
  quantity   DECIMAL(12,3) NOT NULL DEFAULT 0,
  unit_id    SMALLINT       NOT NULL,
  max_qty    DECIMAL(12,3) NOT NULL DEFAULT 0,
  updated_at DATETIME      NOT NULL DEFAULT CURRENT_TIMESTAMP
                           ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (mat_code),
  CONSTRAINT fk_ms_mat  FOREIGN KEY (mat_code) REFERENCES material_master (mat_code),
  CONSTRAINT fk_ms_unit FOREIGN KEY (unit_id)  REFERENCES unit_master (unit_id)
);
 
-- ── 부품 재고 ────────────────────────────────────────────────────────────────
CREATE TABLE part_stock (
  part_code  SMALLINT NOT NULL,
  quantity   INT      NOT NULL DEFAULT 0,
  unit_id    SMALLINT  NOT NULL,
  max_qty    INT      NOT NULL DEFAULT 0,
  updated_at DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP
                      ON UPDATE CURRENT_TIMESTAMP,
  PRIMARY KEY (part_code),
  CONSTRAINT fk_ps_part FOREIGN KEY (part_code) REFERENCES part_master (part_code),
  CONSTRAINT fk_ps_unit FOREIGN KEY (unit_id)   REFERENCES unit_master (unit_id)
);

-- ── 테스트용 데이터 ───────────────────────────────────────────────────────────
INSERT INTO unit_master     (unit_id, unit_name)   VALUES (1, '개');
INSERT INTO material_master (mat_code, mat_name)   VALUES (1, 'metal_cube');
INSERT INTO part_master     (part_code, part_name) VALUES (1, 'metal_part');
 
INSERT INTO material_stock (mat_code, quantity, unit_id, max_qty) VALUES (1, 500, 1, 2000);
INSERT INTO part_stock     (part_code, quantity, unit_id, max_qty) VALUES (1, 0,   1, 2000);
 
-- 확인
SELECT * FROM material_master;
SELECT * FROM part_master;
SELECT * FROM unit_master;
SELECT mat_code, quantity, max_qty, updated_at FROM material_stock;
SELECT part_code, quantity, max_qty, updated_at FROM part_stock;
