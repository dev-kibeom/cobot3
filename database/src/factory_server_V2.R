# =============================================================================
# factory_server.R — Isaac Sim ↔ R ↔ MySQL + Machbase
# =============================================================================
# [실행]  Rscript factory_server.R
# [필요]  install.packages(c("plumber","jsonlite","DBI","RMySQL"))
# =============================================================================

needed  <- c("plumber", "jsonlite", "DBI", "RMySQL", "later", "sys")
missing <- needed[!sapply(needed, requireNamespace, quietly = TRUE)]
if (length(missing) > 0) {
  message("Installing: ", paste(missing, collapse=", "))
  install.packages(missing, repos="https://cran.rstudio.com/")
}

library(plumber)
library(jsonlite)
library(DBI)
library(RMySQL)
library(later)
if (requireNamespace("sys", quietly=TRUE)) library(sys)

# =============================================================================
# 0. 설정
# =============================================================================

MYSQL <- list(
  host     = "127.0.0.1",
  port     = 3306,
  dbname   = "factory_db",
  user     = "rokey",
  password = "rokey1234"
)

MACHBASE <- list(
  host = "127.0.0.1",
  port = 5654,
  exe  = "~/Factorio/machbase-neo-v8.5.2-linux-amd64/machbase-neo"
  # exe 경로는 실제 설치 위치로 수정하세요
)

# =============================================================================
# Machbase 헬스체크 + 자동 시작 + 워치독
# =============================================================================

MACH_HEALTH_URL <- sprintf("http://%s:%d/db/query?q=%s",
                            MACHBASE$host, MACHBASE$port,
                            utils::URLencode("SELECT 1", reserved=TRUE))
MACH_WATCHDOG_INTERVAL <- 10   # 초 단위 헬스체크 주기

mach_is_alive <- function() {
  tryCatch({
    con <- url(MACH_HEALTH_URL)
    suppressWarnings(readLines(con, warn=FALSE))
    close(con)
    TRUE
  }, error=function(e) FALSE)
}

mach_do_start <- function() {
  exe <- path.expand(MACHBASE$exe)
  if (!file.exists(exe)) {
    log_warn("Machbase exe not found: ", exe)
    return(invisible(FALSE))
  }
  log_info("Starting Machbase server...")
  system2(exe, args="serve", wait=FALSE,
          stdout=file.path(dirname(exe), "machbase.log"),
          stderr=file.path(dirname(exe), "machbase.err"))
  # 최대 10초 대기
  for (i in 1:10) {
    Sys.sleep(1)
    if (mach_is_alive()) {
      log_ok(sprintf("Machbase started (attempt %d)", i))
      return(invisible(TRUE))
    }
    message(sprintf("  waiting for Machbase... (%d/10)", i))
  }
  log_warn("Machbase did not respond after 10s")
  invisible(FALSE)
}

# 최초 기동
start_machbase <- function() {
  if (mach_is_alive()) {
    log_info("Machbase already running")
    return(invisible(TRUE))
  }
  mach_do_start()
}

# 워치독 — 별도 R 스레드에서 주기적으로 헬스체크
# Machbase가 꺼지면 자동 재시작 (최대 3회 연속 재시도 후 60초 대기)
start_machbase_watchdog <- function() {
  later::later(function() {
    tryCatch({
      if (!mach_is_alive()) {
        log_warn(sprintf("Machbase DOWN detected at %s — restarting...", now()))
        success <- FALSE
        for (attempt in 1:3) {
          log_warn(sprintf("  restart attempt %d/3", attempt))
          if (mach_do_start()) { success <- TRUE; break }
          Sys.sleep(5)
        }
        if (!success) {
          log_err("Machbase restart failed 3 times — will retry in 60s")
          Sys.sleep(60)
        }
      }
    }, error=function(e) {
      log_err("Watchdog error: ", conditionMessage(e))
    })
    start_machbase_watchdog()   # 재귀 스케줄링
  }, delay=MACH_WATCHDOG_INTERVAL)
}

PORT         <- 8765

# =============================================================================
# 1. 공통 헬퍼
# =============================================================================

J      <- function(x) jsonlite::toJSON(x, auto_unbox=TRUE, null="null")
now    <- function()  format(Sys.time(), "%Y-%m-%d %H:%M:%S")
`%||%` <- function(a, b) if (!is.null(a) && !is.na(a)) a else b

# ── 로그 출력 ──────────────────────────────────────────────────────────────────
SEP  <- strrep("-", 64)
SEP2 <- strrep("=", 64)

log_info <- function(...) message("  [INFO]  ", ...)
log_ok   <- function(...) message("  [ OK ]  ", ...)
log_warn <- function(...) message("  [WARN]  ", ...)
log_err  <- function(...) message("  [ERR ]  ", ...)

# =============================================================================
# 2. DB 연결
# =============================================================================

db_connect <- function() {
  dbConnect(RMySQL::MySQL(),
    host=MYSQL$host, port=MYSQL$port, dbname=MYSQL$dbname,
    user=MYSQL$user, password=MYSQL$password)
}

with_db <- function(fn) {
  err_msg <- NULL
  con <- tryCatch(db_connect(),
                  error=function(e) { err_msg <<- conditionMessage(e); NULL })
  if (is.null(con)) {
    msg <- paste("MySQL connection failed:", err_msg)
    log_err(msg)
    return(list(ok=FALSE, reason=msg))
  }
  on.exit(tryCatch(dbDisconnect(con), error=function(e) NULL), add=TRUE)
  # suppressWarnings: RMySQL UNSIGNED/Decimal 타입 변환 경고 억제
  # 우리 검증 경고는 message()로 출력하므로 영향 없음
  tryCatch(suppressWarnings(fn(con)), error=function(e) {
    log_err("with_db: ", conditionMessage(e))
    list(ok=FALSE, reason=conditionMessage(e))
  })
}

# =============================================================================
# 3. 이름 룩업 (DB에서 읽기)
# =============================================================================

# 서버 시작 시 한 번 로드해서 캐싱
NAME_CACHE <- list(mat=list(), part=list(), unit=list())

load_name_cache <- function() {
  tryCatch({
    con <- db_connect()
    on.exit(dbDisconnect(con))

    # material_master 에서 mat_code → mat_name 로드
    mm <- suppressWarnings(dbGetQuery(con,
      "SELECT mat_code, mat_name FROM material_master"))
    for (i in seq_len(nrow(mm)))
      NAME_CACHE$mat[[as.character(mm$mat_code[i])]] <<- mm$mat_name[i]

    # part_master 에서 part_code → part_name 로드
    pm <- suppressWarnings(dbGetQuery(con,
      "SELECT part_code, part_name FROM part_master"))
    for (i in seq_len(nrow(pm)))
      NAME_CACHE$part[[as.character(pm$part_code[i])]] <<- pm$part_name[i]

    # unit_master 에서 unit_id → unit_name 로드
    um <- suppressWarnings(dbGetQuery(con,
      "SELECT unit_id, unit_name FROM unit_master"))
    for (i in seq_len(nrow(um)))
      NAME_CACHE$unit[[as.character(um$unit_id[i])]] <<- um$unit_name[i]

    message(SEP)
    log_info("Name cache loaded:")
    for (k in names(NAME_CACHE$mat))
      log_info(sprintf("  mat  %-4s -> %s", k, NAME_CACHE$mat[[k]]))
    for (k in names(NAME_CACHE$part))
      log_info(sprintf("  part %-4s -> %s", k, NAME_CACHE$part[[k]]))
    for (k in names(NAME_CACHE$unit))
      log_info(sprintf("  unit %-4s -> %s", k, NAME_CACHE$unit[[k]]))
    message(SEP)
  }, error=function(e) log_warn("name cache load failed: ", conditionMessage(e)))
}

mat_name  <- function(code) NAME_CACHE$mat[[as.character(code)]]  %||% paste0("mat_",  code)
part_name <- function(code) NAME_CACHE$part[[as.character(code)]] %||% paste0("part_", code)
unit_name <- function(id)   NAME_CACHE$unit[[as.character(id)]]   %||% "?"

# =============================================================================
# 4. 기동 시 테이블 출력
# =============================================================================

print_stock_table <- function(con) {
  message(SEP2)
  message("  MATERIAL STOCK")
  message(SEP)
  ms <- suppressWarnings(dbGetQuery(con,
    "SELECT mat_code, quantity, max_qty, unit_id
     FROM material_stock ORDER BY mat_code"))
  message(sprintf("  %-4s  %-20s  %8s  %8s  %-4s",
                  "CODE","NAME","CURRENT","MAX","UNIT"))
  message(sprintf("  %s", strrep("-", 50)))
  for (i in seq_len(nrow(ms))) {
    r    <- ms[i, ]
    stat <- if (r$quantity == 0)               "EMPTY"
            else if (r$quantity < r$max_qty*0.1) " LOW"
            else                                 "  OK"
    message(sprintf("  %-4d  %-20s  %8.1f  %8.1f  %-4s  [%s]",
      r$mat_code, mat_name(r$mat_code),
      r$quantity, r$max_qty,
      unit_name(r$unit_id), stat))
  }
  message(SEP)
  message("  PART STOCK")
  message(SEP)
  ps <- suppressWarnings(dbGetQuery(con,
    "SELECT part_code, quantity, max_qty, unit_id
     FROM part_stock ORDER BY part_code"))
  message(sprintf("  %-4s  %-20s  %8s  %8s  %-4s",
                  "CODE","NAME","CURRENT","MAX","UNIT"))
  message(sprintf("  %s", strrep("-", 50)))
  for (i in seq_len(nrow(ps))) {
    r    <- ps[i, ]
    stat <- if (r$quantity == 0)               "EMPTY"
            else if (r$quantity < r$max_qty*0.1) " LOW"
            else                                 "  OK"
    message(sprintf("  %-4d  %-20s  %8d  %8d  %-4s  [%s]",
      r$part_code, part_name(r$part_code),
      r$quantity, r$max_qty,
      unit_name(r$unit_id), stat))
  }
  message(SEP2)
}

print_history_table <- function(n=10) {
  message(SEP2)
  message("  RECENT HISTORY  (last ", n, " events)")
  message(SEP)
  message(sprintf("  %-5s  %-19s  %-20s  %8s  %8s  %8s",
                  "TYPE","TIME","ITEM","DELTA","BEFORE","AFTER"))
  message(sprintf("  %s", strrep("-", 70)))

  mh <- mach_query(sprintf(
    "SELECT time, mat_code, consumed_qty, qty_before, qty_after
     FROM material_history ORDER BY time DESC LIMIT %d", n))
  ph <- mach_query(sprintf(
    "SELECT time, part_code, consumed_qty, qty_before, qty_after
     FROM part_history ORDER BY time DESC LIMIT %d", n))

  rows <- c(
    lapply(mh, function(r) list(type="MAT",  time=r$time,
      name=mat_name(r$mat_code),
      delta=as.numeric(r$consumed_qty),
      before=as.numeric(r$qty_before),
      after=as.numeric(r$qty_after))),
    lapply(ph, function(r) list(type="PART", time=r$time,
      name=part_name(r$part_code),
      delta=as.numeric(r$consumed_qty),
      before=as.numeric(r$qty_before),
      after=as.numeric(r$qty_after)))
  )

  if (length(rows) == 0) {
    message("  (no history yet)")
  } else {
    for (r in rows) {
      sign <- if (!is.na(r$delta) && r$delta > 0) "+" else ""
      message(sprintf("  %-5s  %-19s  %-20s  %s%7.1f  %8.1f  %8.1f",
        r$type, r$time, r$name, sign, r$delta, r$before, r$after))
    }
  }
  message(SEP2)
}

# =============================================================================
# 5. Machbase HTTP API
# =============================================================================

mach_insert <- function(table, df) {
  if (nrow(df) == 0) return(invisible(NULL))
  base_url <- sprintf("http://%s:%d/db/query", MACHBASE$host, MACHBASE$port)
  for (i in seq_len(nrow(df))) {
    row  <- df[i, ]
    vals <- paste(sapply(row, function(v) {
      if (is.character(v)) paste0("'", v, "'")
      else if (is.na(v))   "NULL"
      else                  as.character(v)
    }), collapse=", ")
    sql  <- sprintf("INSERT INTO %s VALUES (%s)", table, vals)
    tryCatch({
      resp   <- url(paste0(base_url, "?q=", URLencode(sql, reserved=TRUE)))
      body   <- readLines(resp, warn=FALSE); close(resp)
      parsed <- fromJSON(paste(body, collapse=""))
      if (!isTRUE(parsed$success))
        log_warn("Machbase insert failed: ", parsed$reason)
    }, error=function(e) log_warn("Machbase HTTP error: ", conditionMessage(e)))
  }
}

mach_mat_history <- function(mat_code, consumed_qty, qty_before, qty_after) {
  mach_insert("material_history", data.frame(
    name=paste0("mat_",mat_code), time=now(),
    mat_code=as.integer(mat_code),
    consumed_qty=as.numeric(consumed_qty),
    qty_before=as.numeric(qty_before), qty_after=as.numeric(qty_after),
    stringsAsFactors=FALSE))
}

mach_part_history <- function(part_code, consumed_qty, qty_before, qty_after) {
  mach_insert("part_history", data.frame(
    name=paste0("part_",part_code), time=now(),
    part_code=as.integer(part_code),
    consumed_qty=as.double(consumed_qty),
    qty_before=as.double(qty_before), qty_after=as.double(qty_after),
    stringsAsFactors=FALSE))
}

mach_query <- function(sql) {
  url_str <- sprintf("http://%s:%d/db/query?q=%s",
                     MACHBASE$host, MACHBASE$port, URLencode(sql, reserved=TRUE))
  tryCatch({
    resp   <- url(url_str)
    body   <- readLines(resp, warn=FALSE); close(resp)
    parsed <- fromJSON(paste(body, collapse=""))
    if (!isTRUE(parsed$success)) { log_warn("Machbase query: ", parsed$reason); return(list()) }
    cols <- parsed$data$columns
    rows <- parsed$data$rows
    if (is.null(rows) || length(rows)==0) return(list())
    lapply(rows, function(r) setNames(as.list(r), cols))
  }, error=function(e) { log_warn("Machbase query error: ", conditionMessage(e)); list() })
}

# =============================================================================
# 6. 입력값 검증
# =============================================================================

validate <- function(...) {
  for (chk in list(...)) {
    if (!isTRUE(chk[[1]])) {
      log_warn("Validation failed: ", chk[[2]])
      return(list(ok=FALSE, approved=FALSE, reason=paste("Invalid input:", chk[[2]])))
    }
  }
  NULL
}

is_pos_int    <- function(x) !is.null(x) && !is.na(x) && is.numeric(x) && x==floor(x) && x>0
is_pos_num    <- function(x) !is.null(x) && !is.na(x) && is.numeric(x) && x>0
is_nn_num     <- function(x) !is.null(x) && !is.na(x) && is.numeric(x) && x>=0
is_valid_type <- function(x) !is.null(x) && x %in% c("material","part")

# =============================================================================
# 7. 핵심 로직
# =============================================================================

# BOM 로직 없음 — 공정 소모/생산량은 시뮬레이션(fake_factory_sim_V2.py)이 결정
# /check_work : part_code가 part_stock에 존재하는지만 확인 (작업 허가 신호)
# /update_stock : 시뮬레이션이 계산한 소모/생산량을 그대로 DB에 반영

check_part_exists <- function(con, part_code) {
  row <- suppressWarnings(dbGetQuery(con, sprintf(
    "SELECT part_code FROM part_stock WHERE part_code = %d", part_code)))
  nrow(row) > 0
}

df_to_list <- function(df) lapply(seq_len(nrow(df)), function(i) as.list(df[i,]))

# =============================================================================
# 8. 핸들러
# =============================================================================

handle_health <- function() {
  result <- with_db(function(con) {
    tbls <- dbGetQuery(con, "SHOW TABLES")[[1]]
    list(status="ok", mode="mysql+machbase", ts=now(), tables=as.list(tbls))
  })
  if (!is.null(result$ok) && !result$ok)
    return(list(status="error", ts=now(), reason=result$reason))
  result
}

# ── GET /stock ─────────────────────────────────────────────────────────────────
handle_stock <- function() {
  with_db(function(con) {
    ms <- suppressWarnings(dbGetQuery(con,
      "SELECT mat_code, quantity, unit_id, max_qty, updated_at
       FROM material_stock ORDER BY mat_code"))
    ps <- suppressWarnings(dbGetQuery(con,
      "SELECT part_code, quantity, unit_id, max_qty, updated_at
       FROM part_stock ORDER BY part_code"))
    list(ok=TRUE, material_stock=df_to_list(ms), part_stock=df_to_list(ps))
  })
}

# ── POST /check_work ───────────────────────────────────────────────────────────
handle_check_work <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody), error=function(e) NULL)
  if (is.null(body) || is.null(body$part_code) || is.null(body$prod_qty)) {
    res$status <- 400
    return(list(approved=FALSE, reason="part_code and prod_qty required"))
  }
  robot_id  <- as.integer(body$robot_id  %||% -1L)
  part_code <- as.integer(body$part_code)
  prod_qty  <- as.integer(body$prod_qty)

  err <- validate(
    list(is_pos_int(part_code), paste("part_code must be positive integer, got:", part_code)),
    list(is_pos_int(prod_qty),  paste("prod_qty must be positive integer, got:", prod_qty)))
  if (!is.null(err)) { res$status <- 400; return(err) }

  with_db(function(con) {
    # part_code 존재 여부만 확인 — 공정 로직은 시뮬레이션이 담당
    exists <- check_part_exists(con, part_code)

    message(SEP)
    if (exists) {
      log_ok(sprintf("check_work  robot=%-3d  %s x%d  =>  APPROVED",
                     robot_id, part_name(part_code), prod_qty))
    } else {
      log_warn(sprintf("check_work  robot=%-3d  part_code=%d  =>  DENIED (not found)",
                       robot_id, part_code))
    }
    message(SEP)

    list(approved = exists,
         reason   = if (exists) "OK" else paste("part_code not found:", part_code),
         robot_id = robot_id, part_code = part_code, prod_qty = prod_qty)
  })
}

# ── POST /update_stock ─────────────────────────────────────────────────────────
handle_update_stock <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody, simplifyDataFrame=FALSE), error=function(e) NULL)
  if (is.null(body) || is.null(body$materials)) {
    res$status <- 400
    return(list(ok=FALSE, reason="materials required"))
  }
  robot_id  <- as.integer(body$robot_id  %||% -1L)
  part_code <- as.integer(body$part_code %||% -1L)
  prod_qty  <- as.integer(body$prod_qty  %||% 0L)

  mats <- body$materials
  if (!is.null(names(mats))) mats <- list(mats)

  err <- validate(
    list(is_pos_int(part_code)||part_code==-1L, paste("part_code invalid:", part_code)),
    list(is_nn_num(prod_qty),                    paste("prod_qty must be >=0, got:", prod_qty)))
  if (!is.null(err)) { res$status <- 400; return(err) }

  with_db(function(con) {
    today        <- as.integer(format(Sys.Date(), "%Y%m%d"))
    updated_mats <- c()
    skipped_mats <- c()

    message(SEP)
    log_ok(sprintf("update_stock  robot=%-3d  %s x%d",
                   robot_id, part_name(part_code), prod_qty))
    message(sprintf("  %s  %-20s  %8s  %10s  %10s",
                    "   ","MATERIAL","USED","BEFORE","AFTER"))

    # 원자재 차감
    for (m in mats) {
      mc       <- as.integer(m$mat_code)
      used_qty <- as.numeric(m$used_qty)

      if (!is_pos_int(mc)) {
        log_warn(sprintf("  SKIP  mat_code=%-4d  invalid code", mc))
        skipped_mats <- c(skipped_mats, mc); next
      }
      if (!is_pos_num(used_qty)) {
        log_warn(sprintf("  SKIP  %-20s  used_qty=%g  must be > 0",
                         mat_name(mc), used_qty))
        skipped_mats <- c(skipped_mats, mc); next
      }

      row <- dbGetQuery(con, sprintf(
        "SELECT quantity FROM material_stock WHERE mat_code=%d", mc))
      if (nrow(row)==0) next

      qty_before <- row$quantity
      qty_after  <- max(qty_before - used_qty, 0)

      dbExecute(con, sprintf(
        "UPDATE material_stock SET quantity=%.3f, updated_at=NOW() WHERE mat_code=%d",
        qty_after, mc))
      mach_mat_history(mc, -used_qty, qty_before, qty_after)

      updated_mats <- c(updated_mats, mc)
      message(sprintf("  MAT  %-20s  %8.1f  %10.1f  %10.1f",
                      mat_name(mc), used_qty, qty_before, qty_after))
    }

    # 부품 생산
    if (part_code>0 && prod_qty>0) {
      row <- dbGetQuery(con, sprintf(
        "SELECT quantity FROM part_stock WHERE part_code=%d", part_code))
      if (nrow(row)>0) {
        qty_before <- row$quantity
        qty_after  <- qty_before + prod_qty
        dbExecute(con, sprintf(
          "UPDATE part_stock SET quantity=%d, updated_at=NOW() WHERE part_code=%d",
          qty_after, part_code))
        mach_part_history(part_code, prod_qty, qty_before, qty_after)
        message(sprintf("  PART %-20s  %8d  %10d  %10d  (+%d produced)",
                        part_name(part_code), prod_qty, qty_before, qty_after, prod_qty))
      }
    }
    if (length(skipped_mats)>0)
      log_warn("  skipped: [", paste(skipped_mats, collapse=","), "]")
    message(SEP)

    list(ok=TRUE, updated_mat_codes=updated_mats,
         skipped_mat_codes=skipped_mats,
         part_code=part_code, added_qty=prod_qty)
  })
}

# ── POST /set_stock ────────────────────────────────────────────────────────────
handle_set_stock <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody), error=function(e) NULL)
  if (is.null(body)||is.null(body$type)||is.null(body$code)||is.null(body$quantity)) {
    res$status <- 400
    return(list(ok=FALSE, reason="type, code, quantity required"))
  }
  type     <- body$type
  code     <- as.integer(body$code)
  quantity <- as.numeric(body$quantity)
  today    <- as.integer(format(Sys.Date(), "%Y%m%d"))

  err <- validate(
    list(is_valid_type(type), paste("type must be 'material' or 'part', got:", type)),
    list(is_pos_int(code),    paste("code must be positive integer, got:", code)),
    list(is_nn_num(quantity), paste("quantity must be >=0, got:", quantity)))
  if (!is.null(err)) { res$status <- 400; return(err) }

  with_db(function(con) {
    if (type=="material") {
      row <- dbGetQuery(con, sprintf(
        "SELECT quantity FROM material_stock WHERE mat_code=%d", code))
      if (nrow(row)==0) return(list(ok=FALSE, reason=paste("mat_code not found:", code)))
      qty_before <- row$quantity
      dbExecute(con, sprintf(
        "UPDATE material_stock SET quantity=%.3f, updated_at=NOW() WHERE mat_code=%d",
        quantity, code))
      mach_mat_history(code, quantity-qty_before, qty_before, quantity)
      item_nm <- mat_name(code)
    } else {
      row <- dbGetQuery(con, sprintf(
        "SELECT quantity FROM part_stock WHERE part_code=%d", code))
      if (nrow(row)==0) return(list(ok=FALSE, reason=paste("part_code not found:", code)))
      qty_before <- row$quantity
      dbExecute(con, sprintf(
        "UPDATE part_stock SET quantity=%d, updated_at=NOW() WHERE part_code=%d",
        as.integer(quantity), code))
      mach_part_history(code,
                        as.double(quantity-qty_before),
                        as.double(qty_before), as.double(quantity))
      item_nm <- part_name(code)
    }
    message(SEP)
    log_ok(sprintf("set_stock  type=%-8s  code=%-3d  %-20s  %g -> %g",
                   type, code, item_nm, row$quantity, quantity))
    message(SEP)
    list(ok=TRUE, type=type, code=code, quantity=quantity)
  })
}

# ── GET /history ───────────────────────────────────────────────────────────────
handle_history <- function() {
  mh <- mach_query(
    "SELECT name, time, mat_code, consumed_qty, qty_before, qty_after
     FROM material_history ORDER BY time DESC LIMIT 50")
  ph <- mach_query(
    "SELECT name, time, part_code, consumed_qty, qty_before, qty_after
     FROM part_history ORDER BY time DESC LIMIT 50")
  list(ok=TRUE, material_history=mh, part_history=ph)
}

# =============================================================================
# 9. 런처 — 외부 프로세스 실행
# =============================================================================

SCRIPT_DIR <- tryCatch(
  dirname(normalizePath(sys.frame(1)$ofile)),
  error = function(e) getwd()
)

# 실행된 외부 프로세스 PID 목록
CHILD_PIDS <- c()

find_terminal <- function() {
  for (term in c("gnome-terminal","xterm","konsole","xfce4-terminal","lxterminal")) {
    if (system2("which", term, stdout=FALSE, stderr=FALSE) == 0) return(term)
  }
  return(NULL)
}

open_terminal <- function(title, cmd, geometry="120x40") {
  term <- find_terminal()
  if (is.null(term)) {
    log_warn(paste("No terminal found — skipping:", title))
    return(invisible(NULL))
  }
  log_info(paste("Opening:", title, "->", term))

  full_cmd <- switch(term,
    "gnome-terminal" = sprintf(
      "nohup gnome-terminal --window --geometry=%s --title='%s' -- bash -c '%s; exec bash' >/dev/null 2>&1 &",
      geometry, title, cmd),
    "xterm" = sprintf(
      "nohup xterm -title '%s' -geometry %s -e bash -c '%s; exec bash' >/dev/null 2>&1 &",
      title, geometry, cmd),
    "konsole" = sprintf(
      "nohup konsole -- bash -c '%s; exec bash' >/dev/null 2>&1 &",
      cmd),
    sprintf(
      "nohup %s --geometry=%s --title '%s' -- bash -c '%s; exec bash' >/dev/null 2>&1 &",
      term, geometry, title, cmd)
  )
  system(full_cmd, ignore.stdout=TRUE, ignore.stderr=TRUE, wait=FALSE)
}

# 모니터를 현재 R 세션의 자식 프로세스로 실행 (종속 관계)
start_monitor <- function(cmd) {
  # python3 프로세스를 R의 자식으로 직접 실행
  # R 종료 시 SIGHUP으로 자식도 같이 종료됨
  pid <- sys::exec_background(
    "bash", c("-c", cmd),
    std_out=FALSE, std_err=FALSE
  )
  log_info(paste("Monitor PID:", pid))
  CHILD_PIDS <<- c(CHILD_PIDS, pid)
  invisible(pid)
}

get_geometry <- function() {
  tryCatch({
    out  <- system2("xrandr", stdout=TRUE, stderr=FALSE)
    line <- grep("[*]", out, value=TRUE)[1]
    m    <- regmatches(line, regexpr("[0-9]+x[0-9]+", line))
    if (length(m) == 0) return("120x38")
    dims <- as.integer(strsplit(m, "x")[[1]])
    cols <- dims[1] %/% 8  %/% 2
    rows <- dims[2] %/% 16 %/% 2
    paste0(max(cols, 120), "x", max(rows, 35))
  }, error=function(e) "120x38")
}

# =============================================================================
# 10. 서버 기동
# =============================================================================

# ── MySQL 연결 확인 ────────────────────────────────────────────────────────────
message(SEP2)
message("  연결 확인 중...")
message(SEP)
mysql_ok <- tryCatch({
  con_test <- db_connect()
  dbDisconnect(con_test)
  TRUE
}, error=function(e) {
  log_err("MySQL 연결 실패: ", conditionMessage(e))
  FALSE
})
if (!mysql_ok) {
  message(SEP2)
  stop("MySQL 연결 실패 — 서버를 확인하세요: sudo systemctl start mysql")
}
log_ok("MySQL 연결 성공")

# ── Machbase 시작 + 연결 확인 ─────────────────────────────────────────────────
start_machbase()
if (!mach_is_alive()) {
  stop("Machbase 연결 실패 — exe 경로를 확인하세요: MACHBASE$exe")
}
log_ok("Machbase 연결 성공")

# Machbase 워치독 시작
start_machbase_watchdog()
log_info(sprintf("Machbase watchdog started (interval: %ds)", MACH_WATCHDOG_INTERVAL))

# 이름 캐시 로드
load_name_cache()

message(SEP2)
message("  FACTORY SERVER  (MySQL + Machbase)   port:", PORT)
message(SEP2)
message("  GET  /health   GET  /stock   GET  /history")
message("  POST /check_work   POST /update_stock   POST /set_stock")
message(SEP2)

# ── 모니터 실행 (R 서버 자식 프로세스로 종속) ────────────────────────────────
geo     <- get_geometry()
mon_cmd <- paste("cd", SCRIPT_DIR, "&& python3 monitor_V2.py --inner")

# 항상 별도 터미널 창으로 실행
open_terminal("Smart Factory Monitor", mon_cmd, geometry=geo)
on.exit({
  message()
  log_info("Shutting down — closing monitor...")
  system("pkill -f 'monitor_V2.py'", ignore.stdout=TRUE, ignore.stderr=TRUE)
  log_ok("Monitor closed")
}, add=TRUE)
log_ok(paste("Monitor launched (geometry:", geo, ")"))

# ── 모니터 워치독 — 모니터 꺼지면 R 서버도 종료 ──────────────────────────────
start_monitor_watchdog <- function() {
  later::later(function() {
    alive <- tryCatch({
      result <- system2("pgrep", args=c("-f", "monitor_V2.py"),
                        stdout=TRUE, stderr=FALSE)
      length(result) > 0
    }, error=function(e) FALSE)

    if (!alive) {
      message(SEP2)
      log_info("Monitor closed — shutting down R server...")
      message(SEP2)
      # Machbase 워치독 정리
      system("pkill -f 'machbase-neo serve'",
             ignore.stdout=TRUE, ignore.stderr=TRUE)
      Sys.sleep(1)
      quit(save="no", status=0)
    }
    start_monitor_watchdog()
  }, delay=5)
}

# plumber 기동 후 10초 뒤 워치독 시작 (초기 실행 여유 시간)
later::later(function() {
  log_info("Monitor watchdog started (check every 5s)")
  start_monitor_watchdog()
}, delay=10)

# 기동 시 재고 현황 + 이력 출력
tryCatch({
  con_init <- db_connect()
  print_stock_table(con_init)
  dbDisconnect(con_init)
  print_history_table(10)
}, error=function(e) log_warn("Could not print initial tables: ", conditionMessage(e)))

router <- pr()
router <- pr_get( router, "/health",       function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_health());               res })
router <- pr_get( router, "/stock",        function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_stock());                res })
router <- pr_post(router, "/check_work",   function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_check_work(req,res));   res })
router <- pr_post(router, "/update_stock", function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_update_stock(req,res));  res })
router <- pr_post(router, "/set_stock",    function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_set_stock(req,res));     res })
router <- pr_get( router, "/history",      function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_history());              res })

pr_run(router, host="0.0.0.0", port=PORT)
