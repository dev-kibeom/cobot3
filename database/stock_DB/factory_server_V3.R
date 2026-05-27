# =============================================================================
# factory_server_V3.R — MySQL + Machbase
# =============================================================================

needed  <- c("plumber", "jsonlite", "DBI", "RMySQL", "later", "sys")
missing <- needed[!sapply(needed, requireNamespace, quietly = TRUE)]
if (length(missing) > 0) {
  install.packages(missing, repos="https://cran.rstudio.com/")
}

library(plumber); library(jsonlite); library(DBI)
library(RMySQL);  library(later)
if (requireNamespace("sys", quietly=TRUE)) library(sys)

# =============================================================================
# 0. 설정
# =============================================================================

MYSQL <- list(
  host="127.0.0.1", port=3306, dbname="factory_db",
  user="rokey", password="rokey1234"
)
MACHBASE <- list(
  host="127.0.0.1", port=5654,
  exe="./machbase-neo-v8.5.2-linux-amd64/machbase-neo"
)
PORT <- 8765

# =============================================================================
# 1. 공통 헬퍼
# =============================================================================

J      <- function(x) jsonlite::toJSON(x, auto_unbox=TRUE, null="null")
now    <- function()  format(Sys.time(), "%Y-%m-%d %H:%M:%S")
`%||%` <- function(a,b) if (!is.null(a) && !is.na(a)) a else b
SEP    <- strrep("-", 64)
SEP2   <- strrep("=", 64)

log_info <- function(...) message("  [INFO]  ", ...)
log_ok   <- function(...) message("  [ OK ]  ", ...)
log_warn <- function(...) message("  [WARN]  ", ...)
log_err  <- function(...) message("  [ERR ]  ", ...)

# =============================================================================
# 2. Machbase
# =============================================================================

MACH_HEALTH_URL <- sprintf("http://%s:%d/db/query?q=%s",
  MACHBASE$host, MACHBASE$port,
  utils::URLencode("SELECT 1", reserved=TRUE))

mach_is_alive <- function() {
  tryCatch({
    con <- url(MACH_HEALTH_URL)
    suppressWarnings(readLines(con, warn=FALSE)); close(con); TRUE
  }, error=function(e) FALSE)
}

mach_do_start <- function() {
  exe <- path.expand(MACHBASE$exe)
  if (!file.exists(exe)) { log_warn("Machbase exe not found: ", exe); return(invisible(FALSE)) }
  log_info("Starting Machbase...")
  system2(exe, args="serve", wait=FALSE,
          stdout=file.path(dirname(exe),"machbase.log"),
          stderr=file.path(dirname(exe),"machbase.err"))
  for (i in 1:10) {
    Sys.sleep(1)
    if (mach_is_alive()) { log_ok(sprintf("Machbase started (%d)", i)); return(invisible(TRUE)) }
  }
  log_warn("Machbase did not respond"); invisible(FALSE)
}

start_machbase <- function() {
  if (mach_is_alive()) { log_info("Machbase already running"); return(invisible(TRUE)) }
  mach_do_start()
}

start_machbase_watchdog <- function() {
  later::later(function() {
    tryCatch({
      if (!mach_is_alive()) {
        log_warn("Machbase DOWN — restarting...")
        for (i in 1:3) { if (mach_do_start()) break; Sys.sleep(5) }
      }
    }, error=function(e) log_err("Watchdog: ", conditionMessage(e)))
    start_machbase_watchdog()
  }, delay=10)
}

mach_query_url <- function(sql) {
  sprintf("http://%s:%d/db/query?q=%s",
          MACHBASE$host, MACHBASE$port,
          URLencode(sql, reserved=TRUE))
}

mach_exec <- function(sql) {
  tryCatch({
    resp   <- url(mach_query_url(sql))
    body   <- readLines(resp, warn=FALSE); close(resp)
    parsed <- fromJSON(paste(body, collapse=""))
    if (!isTRUE(parsed$success))
      log_warn("Machbase exec failed: ", parsed$reason %||% "unknown")
    isTRUE(parsed$success)
  }, error=function(e) { log_warn("Machbase HTTP error: ", conditionMessage(e)); FALSE })
}

mach_query <- function(sql) {
  tryCatch({
    resp   <- url(mach_query_url(sql))
    body   <- readLines(resp, warn=FALSE); close(resp)
    parsed <- fromJSON(paste(body, collapse=""))
    if (!isTRUE(parsed$success)) { log_warn("Machbase query: ", parsed$reason); return(list()) }
    cols <- parsed$data$columns; rows <- parsed$data$rows
    if (is.null(rows) || length(rows)==0) return(list())
    lapply(rows, function(r) setNames(as.list(r), cols))
  }, error=function(e) { log_warn("Machbase query error: ", conditionMessage(e)); list() })
}

mach_insert <- function(table, df) {
  if (nrow(df)==0) return(invisible(NULL))
  for (i in seq_len(nrow(df))) {
    row  <- df[i,]
    vals <- paste(sapply(row, function(v) {
      if (is.character(v)) paste0("'", gsub("'","''",v), "'")
      else if (is.na(v))   "NULL"
      else                  as.character(v)
    }), collapse=", ")
    mach_exec(sprintf("INSERT INTO %s VALUES (%s)", table, vals))
  }
}

# =============================================================================
# 3. Machbase 테이블 초기화
# =============================================================================

init_machbase_tables <- function() {
  tables <- list(
    # 재고 이력 (기존)
    material_history = "CREATE TABLE material_history (
      name VARCHAR(50), time DATETIME, mat_code INTEGER,
      consumed_qty DOUBLE, qty_before DOUBLE, qty_after DOUBLE)",
    part_history = "CREATE TABLE part_history (
      name VARCHAR(50), time DATETIME, part_code INTEGER,
      consumed_qty DOUBLE, qty_before DOUBLE, qty_after DOUBLE)",

    # 로봇 작동 로그 (신규)
    robot_state_log = "CREATE TABLE IF NOT EXISTS robot_state_log (
      time DATETIME, robot_id INTEGER, robot_type INTEGER,
      status INTEGER, pos_x DOUBLE, pos_y DOUBLE)",
    robot_command_log = "CREATE TABLE IF NOT EXISTS robot_command_log (
      time DATETIME, command_id INTEGER, robot_id INTEGER,
      command INTEGER, to_loc INTEGER, from_loc INTEGER,
      status INTEGER, fail_reason VARCHAR(200))",
    factory_event_log = "CREATE TABLE IF NOT EXISTS factory_event_log (
      time DATETIME, event_type VARCHAR(50),
      robot_id INTEGER, location_id INTEGER, payload VARCHAR(500))"
  )

  message(SEP)
  for (name in names(tables)) {
    ok <- mach_exec(tables[[name]])
    if (ok) log_ok(sprintf("Table ready: %s", name))
    else    log_ok(sprintf("Table already exists: %s", name))
  }
  message(SEP)
}

# =============================================================================
# 4. MySQL
# =============================================================================

db_connect <- function() {
  dbConnect(RMySQL::MySQL(),
    host=MYSQL$host, port=MYSQL$port, dbname=MYSQL$dbname,
    user=MYSQL$user, password=MYSQL$password)
}

with_db <- function(fn) {
  err_msg <- NULL
  con <- tryCatch(db_connect(), error=function(e) { err_msg <<- conditionMessage(e); NULL })
  if (is.null(con)) return(list(ok=FALSE, reason=paste("MySQL:", err_msg)))
  on.exit(tryCatch(dbDisconnect(con), error=function(e) NULL), add=TRUE)
  tryCatch(suppressWarnings(fn(con)), error=function(e) {
    log_err("with_db: ", conditionMessage(e))
    list(ok=FALSE, reason=conditionMessage(e))
  })
}

# =============================================================================
# 5. 이름 캐시
# =============================================================================

NAME_CACHE <- list(mat=list(), part=list(), unit=list())

load_name_cache <- function() {
  tryCatch({
    con <- db_connect(); on.exit(dbDisconnect(con))
    mm <- suppressWarnings(dbGetQuery(con, "SELECT mat_code,mat_name FROM material_master"))
    pm <- suppressWarnings(dbGetQuery(con, "SELECT part_code,part_name FROM part_master"))
    um <- suppressWarnings(dbGetQuery(con, "SELECT unit_id,unit_name FROM unit_master"))
    for (i in seq_len(nrow(mm))) NAME_CACHE$mat[[as.character(mm$mat_code[i])]]  <<- mm$mat_name[i]
    for (i in seq_len(nrow(pm))) NAME_CACHE$part[[as.character(pm$part_code[i])]] <<- pm$part_name[i]
    for (i in seq_len(nrow(um))) NAME_CACHE$unit[[as.character(um$unit_id[i])]]   <<- um$unit_name[i]
    log_ok(sprintf("Name cache: mat=%d part=%d unit=%d",
                   length(NAME_CACHE$mat), length(NAME_CACHE$part), length(NAME_CACHE$unit)))
  }, error=function(e) log_warn("name cache load failed: ", conditionMessage(e)))
}

mat_name  <- function(code) NAME_CACHE$mat[[as.character(code)]]  %||% paste0("mat_",  code)
part_name <- function(code) NAME_CACHE$part[[as.character(code)]] %||% paste0("part_", code)
unit_name <- function(id)   NAME_CACHE$unit[[as.character(id)]]   %||% "?"

# =============================================================================
# 6. 검증 헬퍼
# =============================================================================

validate <- function(...) {
  for (chk in list(...)) {
    if (!isTRUE(chk[[1]])) return(list(ok=FALSE, reason=paste("Invalid:", chk[[2]])))
  }
  NULL
}
is_pos_int    <- function(x) !is.null(x)&&!is.na(x)&&is.numeric(x)&&x==floor(x)&&x>0
is_pos_num    <- function(x) !is.null(x)&&!is.na(x)&&is.numeric(x)&&x>0
is_nn_num     <- function(x) !is.null(x)&&!is.na(x)&&is.numeric(x)&&x>=0
is_valid_type <- function(x) !is.null(x)&&x %in% c("material","part")
df_to_list    <- function(df) lapply(seq_len(nrow(df)), function(i) as.list(df[i,]))

# =============================================================================
# 7. Machbase 기록 함수
# =============================================================================

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

mach_robot_state <- function(robot_id, robot_type, status, pos_x, pos_y) {
  mach_insert("robot_state_log", data.frame(
    time=now(),
    robot_id=as.integer(robot_id), robot_type=as.integer(robot_type),
    status=as.integer(status),
    pos_x=as.double(pos_x), pos_y=as.double(pos_y),
    stringsAsFactors=FALSE))
}

mach_command <- function(command_id, robot_id, command, to_loc,
                          from_loc=NA, status=0, fail_reason="") {
  mach_insert("robot_command_log", data.frame(
    time=now(),
    command_id=as.integer(command_id), robot_id=as.integer(robot_id),
    command=as.integer(command), to_loc=as.integer(to_loc),
    from_loc=as.integer(from_loc %||% -1L),
    status=as.integer(status),
    fail_reason=as.character(fail_reason %||% ""),
    stringsAsFactors=FALSE))
}

mach_factory_event <- function(event_type, robot_id, location_id, payload="") {
  mach_insert("factory_event_log", data.frame(
    time=now(),
    event_type=as.character(event_type),
    robot_id=as.integer(robot_id), location_id=as.integer(location_id),
    payload=as.character(payload),
    stringsAsFactors=FALSE))
}

# =============================================================================
# 8. 핸들러
# =============================================================================

# ── GET /health ───────────────────────────────────────────────────────────────
handle_health <- function() {
  result <- with_db(function(con) {
    tbls <- dbGetQuery(con, "SHOW TABLES")[[1]]
    list(status="ok", mode="mysql+machbase", ts=now(), tables=as.list(tbls))
  })
  if (!is.null(result$ok) && !result$ok)
    return(list(status="error", ts=now(), reason=result$reason))
  result
}

# ── GET /stock ────────────────────────────────────────────────────────────────
handle_stock <- function() {
  with_db(function(con) {
    ms <- suppressWarnings(dbGetQuery(con,
      "SELECT ms.mat_code, mm.mat_name, ms.quantity, ms.unit_id,
              um.unit_name, ms.max_qty, ms.updated_at
       FROM material_stock ms
       LEFT JOIN material_master mm ON mm.mat_code = ms.mat_code
       LEFT JOIN unit_master um ON um.unit_id = ms.unit_id
       ORDER BY ms.mat_code"))
    ps <- suppressWarnings(dbGetQuery(con,
      "SELECT ps.part_code, pm.part_name, ps.quantity, ps.unit_id,
              um.unit_name, ps.max_qty, ps.updated_at
       FROM part_stock ps
       LEFT JOIN part_master pm ON pm.part_code = ps.part_code
       LEFT JOIN unit_master um ON um.unit_id = ps.unit_id
       ORDER BY ps.part_code"))
    list(ok=TRUE, material_stock=df_to_list(ms), part_stock=df_to_list(ps))
  })
}

# ── POST /update_stock ────────────────────────────────────────────────────────
handle_update_stock <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody, simplifyDataFrame=FALSE), error=function(e) NULL)
  if (is.null(body)||is.null(body$materials)) {
    res$status <- 400; return(list(ok=FALSE, reason="materials required"))
  }
  robot_id  <- as.integer(body$robot_id  %||% -1L)
  part_code <- as.integer(body$part_code %||% -1L)
  prod_qty  <- as.integer(body$prod_qty  %||% 0L)
  mats      <- body$materials
  if (!is.null(names(mats))) mats <- list(mats)

  err <- validate(
    list(is_pos_int(part_code)||part_code==-1L, paste("part_code invalid:", part_code)),
    list(is_nn_num(prod_qty),                    paste("prod_qty must be >=0:", prod_qty)))
  if (!is.null(err)) { res$status <- 400; return(err) }

  with_db(function(con) {
    updated_mats <- c(); skipped_mats <- c()
    message(SEP)
    log_ok(sprintf("update_stock  robot=%-3d  %s x%d", robot_id, part_name(part_code), prod_qty))

    for (m in mats) {
      mc       <- as.integer(m$mat_code)
      used_qty <- as.numeric(m$used_qty)
      if (!is_pos_int(mc)||!is_pos_num(used_qty)) { skipped_mats <- c(skipped_mats,mc); next }
      row <- dbGetQuery(con, sprintf("SELECT quantity FROM material_stock WHERE mat_code=%d", mc))
      if (nrow(row)==0) next
      qty_before <- row$quantity; qty_after <- max(qty_before-used_qty, 0)
      dbExecute(con, sprintf(
        "UPDATE material_stock SET quantity=%.3f, updated_at=NOW() WHERE mat_code=%d",
        qty_after, mc))
      mach_mat_history(mc, -used_qty, qty_before, qty_after)
      updated_mats <- c(updated_mats, mc)
      message(sprintf("  MAT  %-20s  -%8.1f  %10.1f → %10.1f",
                      mat_name(mc), used_qty, qty_before, qty_after))
    }

    if (part_code>0 && prod_qty>0) {
      row <- dbGetQuery(con, sprintf("SELECT quantity FROM part_stock WHERE part_code=%d", part_code))
      if (nrow(row)>0) {
        qty_before <- row$quantity; qty_after <- qty_before+prod_qty
        dbExecute(con, sprintf(
          "UPDATE part_stock SET quantity=%d, updated_at=NOW() WHERE part_code=%d",
          qty_after, part_code))
        mach_part_history(part_code, prod_qty, qty_before, qty_after)
        message(sprintf("  PART %-20s  +%8d  %10d → %10d",
                        part_name(part_code), prod_qty, qty_before, qty_after))
      }
    }
    message(SEP)
    list(ok=TRUE, updated_mat_codes=updated_mats,
         skipped_mat_codes=skipped_mats, part_code=part_code, added_qty=prod_qty)
  })
}

# ── POST /set_stock ───────────────────────────────────────────────────────────
handle_set_stock <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody), error=function(e) NULL)
  if (is.null(body)||is.null(body$type)||is.null(body$code)||is.null(body$quantity)) {
    res$status <- 400; return(list(ok=FALSE, reason="type, code, quantity required"))
  }
  type <- body$type; code <- as.integer(body$code); quantity <- as.numeric(body$quantity)
  err <- validate(
    list(is_valid_type(type), paste("type must be material or part, got:", type)),
    list(is_pos_int(code),    paste("code must be positive integer:", code)),
    list(is_nn_num(quantity), paste("quantity must be >=0:", quantity)))
  if (!is.null(err)) { res$status <- 400; return(err) }
  with_db(function(con) {
    if (type=="material") {
      row <- dbGetQuery(con, sprintf("SELECT quantity FROM material_stock WHERE mat_code=%d", code))
      if (nrow(row)==0) return(list(ok=FALSE, reason=paste("mat_code not found:", code)))
      qty_before <- row$quantity
      dbExecute(con, sprintf("UPDATE material_stock SET quantity=%.3f, updated_at=NOW() WHERE mat_code=%d", quantity, code))
      mach_mat_history(code, quantity-qty_before, qty_before, quantity)
    } else {
      row <- dbGetQuery(con, sprintf("SELECT quantity FROM part_stock WHERE part_code=%d", code))
      if (nrow(row)==0) return(list(ok=FALSE, reason=paste("part_code not found:", code)))
      qty_before <- row$quantity
      dbExecute(con, sprintf("UPDATE part_stock SET quantity=%d, updated_at=NOW() WHERE part_code=%d", as.integer(quantity), code))
      mach_part_history(code, as.double(quantity-qty_before), as.double(qty_before), as.double(quantity))
    }
    list(ok=TRUE, type=type, code=code, quantity=quantity)
  })
}

# ── POST /sync_production ─────────────────────────────────────────────────────
handle_sync_production <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody, simplifyDataFrame=FALSE), error=function(e) NULL)
  if (is.null(body)||is.null(body$production)) {
    res$status <- 400; return(list(ok=FALSE, reason="production field required"))
  }
  production <- body$production; added_parts <- c(); added_units <- c()
  with_db(function(con) {
    existing_parts <- suppressWarnings(dbGetQuery(con, "SELECT part_code FROM part_master"))$part_code
    existing_units <- suppressWarnings(dbGetQuery(con, "SELECT unit_id,unit_name FROM unit_master"))
    unit_name_to_id <- setNames(as.integer(existing_units$unit_id), existing_units$unit_name)
    message(SEP); log_info("sync_production 시작")
    for (part_code_str in names(production)) {
      part_code <- as.integer(part_code_str)
      info      <- production[[part_code_str]]
      part_nm   <- info$part_name %||% paste0("part_", part_code)
      unit_nm   <- info$unit      %||% "개"
      max_qty   <- as.integer(info$max_qty %||% 10000L)
      if (is.null(unit_name_to_id[[unit_nm]])) {
        new_unit_id <- max(as.integer(names(unit_name_to_id)), 0L)+1L
        dbExecute(con, sprintf("INSERT IGNORE INTO unit_master (unit_id,unit_name) VALUES (%d,'%s')", new_unit_id, unit_nm))
        unit_name_to_id[[unit_nm]] <- new_unit_id
        added_units <- c(added_units, unit_nm)
        NAME_CACHE$unit[[as.character(new_unit_id)]] <<- unit_nm
        log_ok(sprintf("  unit 추가: id=%d name=%s", new_unit_id, unit_nm))
      }
      unit_id <- unit_name_to_id[[unit_nm]]
      if (!(part_code %in% existing_parts)) {
        dbExecute(con, sprintf("INSERT IGNORE INTO part_master (part_code,part_name) VALUES (%d,'%s')", part_code, part_nm))
        dbExecute(con, sprintf("INSERT IGNORE INTO part_stock (part_code,quantity,unit_id,max_qty) VALUES (%d,0,%d,%d)", part_code, unit_id, max_qty))
        added_parts <- c(added_parts, part_code)
        NAME_CACHE$part[[as.character(part_code)]] <<- part_nm
        log_ok(sprintf("  부품 추가: code=%d name=%s", part_code, part_nm))
      } else {
        log_info(sprintf("  부품 확인: code=%d name=%s  (이미 존재)", part_code, part_nm))
      }
    }
    message(SEP)
    list(ok=TRUE, added_part_codes=added_parts, added_unit_names=added_units,
         total_production=length(production))
  })
}

# ── GET /history ──────────────────────────────────────────────────────────────
handle_history <- function() {
  mh <- mach_query("SELECT name,time,mat_code,consumed_qty,qty_before,qty_after FROM material_history ORDER BY time DESC LIMIT 50")
  ph <- mach_query("SELECT name,time,part_code,consumed_qty,qty_before,qty_after FROM part_history ORDER BY time DESC LIMIT 50")
  list(ok=TRUE, material_history=mh, part_history=ph)
}

# ── POST /log_robot_state — bridge가 호출 ─────────────────────────────────────
handle_log_robot_state <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody, simplifyDataFrame=FALSE), error=function(e) NULL)
  if (is.null(body)||is.null(body$robot_id)) {
    res$status <- 400; return(list(ok=FALSE, reason="robot_id required"))
  }
  mach_robot_state(
    body$robot_id, body$robot_type %||% 1L,
    body$status %||% 1L,
    body$pos_x %||% 0.0, body$pos_y %||% 0.0
  )
  list(ok=TRUE)
}

# ── POST /log_command — nav_node가 호출 ──────────────────────────────────────
handle_log_command <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody, simplifyDataFrame=FALSE), error=function(e) NULL)
  if (is.null(body)||is.null(body$robot_id)||is.null(body$command)) {
    res$status <- 400; return(list(ok=FALSE, reason="robot_id, command required"))
  }
  mach_command(
    body$command_id %||% -1L, body$robot_id, body$command,
    body$to_loc %||% -1L, body$from_loc,
    body$status %||% 0L, body$fail_reason %||% ""
  )
  list(ok=TRUE)
}

# ── POST /log_event — bridge 또는 orchestrator가 호출 ────────────────────────
handle_log_event <- function(req, res) {
  body <- tryCatch(fromJSON(req$postBody, simplifyDataFrame=FALSE), error=function(e) NULL)
  if (is.null(body)||is.null(body$event_type)) {
    res$status <- 400; return(list(ok=FALSE, reason="event_type required"))
  }
  payload_str <- tryCatch(toJSON(body$payload %||% list(), auto_unbox=TRUE), error=function(e) "")
  mach_factory_event(body$event_type, body$robot_id %||% -1L,
                     body$location_id %||% -1L, payload_str)
  list(ok=TRUE)
}

# ── GET /robot_log — 최근 로봇 로그 조회 ─────────────────────────────────────
handle_robot_log <- function() {
  state_log <- mach_query("SELECT time,robot_id,robot_type,status,pos_x,pos_y FROM robot_state_log ORDER BY time DESC LIMIT 100")
  cmd_log   <- mach_query("SELECT time,command_id,robot_id,command,to_loc,status,fail_reason FROM robot_command_log ORDER BY time DESC LIMIT 100")
  evt_log   <- mach_query("SELECT time,event_type,robot_id,location_id,payload FROM factory_event_log ORDER BY time DESC LIMIT 100")
  list(ok=TRUE, state_log=state_log, command_log=cmd_log, event_log=evt_log)
}

# ── POST /monitor_alive ───────────────────────────────────────────────────────
MONITOR_LAST_ALIVE <- proc.time()[3]
HEARTBEAT_TIMEOUT  <- 15

# =============================================================================
# 9. 모니터/런처 헬퍼
# =============================================================================

SCRIPT_DIR <- tryCatch(dirname(normalizePath(sys.frame(1)$ofile)), error=function(e) getwd())
CHILD_PIDS <- c()

find_terminal <- function() {
  for (term in c("gnome-terminal","xterm","konsole","xfce4-terminal","lxterminal")) {
    if (system2("which", term, stdout=FALSE, stderr=FALSE)==0) return(term)
  }
  NULL
}

open_terminal <- function(title, cmd, geometry="120x40") {
  term <- find_terminal()
  if (is.null(term)) { log_warn("No terminal — skipping: ", title); return(invisible(NULL)) }
  full_cmd <- switch(term,
    "gnome-terminal"=sprintf("nohup gnome-terminal --window --geometry=%s --title='%s' -- bash -c '%s' >/dev/null 2>&1 &", geometry, title, cmd),
    "xterm"=sprintf("nohup xterm -title '%s' -geometry %s -e bash -c '%s' >/dev/null 2>&1 &", title, geometry, cmd),
    sprintf("nohup %s --geometry=%s --title '%s' -- bash -c '%s' >/dev/null 2>&1 &", term, geometry, title, cmd))
  system(full_cmd, ignore.stdout=TRUE, ignore.stderr=TRUE, wait=FALSE)
}

get_geometry <- function() {
  tryCatch({
    out  <- system2("xrandr", stdout=TRUE, stderr=FALSE)
    line <- grep("[*]", out, value=TRUE)[1]
    m    <- regmatches(line, regexpr("[0-9]+x[0-9]+", line))
    if (length(m)==0) return("120x38")
    dims <- as.integer(strsplit(m,"x")[[1]])
    paste0(max(dims[1]%/%8%/%2,120),"x",max(dims[2]%/%16%/%2,35))
  }, error=function(e) "120x38")
}

# =============================================================================
# 10. 서버 기동
# =============================================================================

message(SEP2); message("  연결 확인 중..."); message(SEP)

mysql_ok <- tryCatch({
  con_test <- db_connect(); dbDisconnect(con_test); TRUE
}, error=function(e) { log_err("MySQL 연결 실패: ", conditionMessage(e)); FALSE })
if (!mysql_ok) stop("MySQL 연결 실패")
log_ok("MySQL 연결 성공")

start_machbase()
if (!mach_is_alive()) stop("Machbase 연결 실패")
log_ok("Machbase 연결 성공")

start_machbase_watchdog()
init_machbase_tables()
load_name_cache()

message(SEP2)
message("  FACTORY SERVER V3  (MySQL + Machbase)   port:", PORT)
message(SEP2)
message("  재고: GET /health  GET /stock  GET /history")
message("  재고: POST /update_stock  /set_stock  /sync_production")
message("  로그: POST /log_robot_state  /log_command  /log_event")
message("  로그: GET /robot_log")
message(SEP2)

geo     <- get_geometry()

# FMS Server는 팀원 코드(database/run.py)를 별도 실행
# python3 run.py  ← 팀원 폴더에서 직접 실행

mon_cmd <- paste("cd", SCRIPT_DIR, "&& python3 monitor_V2.py --inner")
open_terminal("Smart Factory Monitor", mon_cmd, geometry=geo)
on.exit({
  system("pkill -f 'monitor_V2.py'", ignore.stdout=TRUE, ignore.stderr=TRUE)
}, add=TRUE)

start_monitor_watchdog <- function() {
  later::later(function() {
    elapsed <- proc.time()[3] - MONITOR_LAST_ALIVE
    if (elapsed >= HEARTBEAT_TIMEOUT) {
      log_info(sprintf("Monitor heartbeat lost (%.0fs) — shutting down...", elapsed))
      system("pkill -f 'machbase-neo serve'", ignore.stdout=TRUE, ignore.stderr=TRUE)
      Sys.sleep(1); quit(save="no", status=0)
    }
    start_monitor_watchdog()
  }, delay=5)
}
later::later(function() {
  log_info("Monitor watchdog started")
  start_monitor_watchdog()
}, delay=10)

router <- pr()

# CORS 설정 — 웹UI(localhost:5173 등)에서 접근 허용
router <- pr_filter(router, "cors", function(req, res) {
  res$setHeader("Access-Control-Allow-Origin",  "*")
  res$setHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
  res$setHeader("Access-Control-Allow-Headers", "Content-Type")
  if (req$REQUEST_METHOD == "OPTIONS") {
    res$status <- 204
    return(res)
  }
  plumber::forward()
})

router <- pr_post(router, "/monitor_alive",     function(req,res){ MONITOR_LAST_ALIVE <<- proc.time()[3]; res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- '{"ok":true}'; res })
router <- pr_get( router, "/health",            function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_health());                    res })
router <- pr_get( router, "/stock",             function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_stock());                     res })
router <- pr_get( router, "/history",           function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_history());                   res })
router <- pr_get( router, "/robot_log",         function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_robot_log());                 res })
router <- pr_post(router, "/update_stock",      function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_update_stock(req,res));       res })
router <- pr_post(router, "/set_stock",         function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_set_stock(req,res));          res })
router <- pr_post(router, "/sync_production",   function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_sync_production(req,res));    res })
router <- pr_post(router, "/log_robot_state",   function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_log_robot_state(req,res));   res })
router <- pr_post(router, "/log_command",       function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_log_command(req,res));        res })
router <- pr_post(router, "/log_event",         function(req,res){ res$setHeader("Content-Type","application/json; charset=utf-8"); res$body <- J(handle_log_event(req,res));          res })

shutdown <- function() {
  message(); message(SEP)
  log_info("Shutting down...")
  system("pkill -f 'monitor_V2.py'",       ignore.stdout=TRUE, ignore.stderr=TRUE)
  message(SEP); quit(save="no", status=0)
}

tryCatch(
  pr_run(router, host="0.0.0.0", port=PORT),
  interrupt=function(e) shutdown(),
  error=function(e) { log_err("Server error: ", conditionMessage(e)); shutdown() }
)
