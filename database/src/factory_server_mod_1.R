# =============================================================================
# factory_server.R — Isaac Sim ↔ R ↔ MySQL + Machbase
# =============================================================================
# [실행]  Rscript factory_server.R
# [필요]  install.packages(c("plumber","jsonlite","DBI","RMySQL"))
# =============================================================================

needed  <- c("plumber", "jsonlite", "DBI", "RMySQL")
missing <- needed[!sapply(needed, requireNamespace, quietly = TRUE)]
if (length(missing) > 0) {
  message("Installing: ", paste(missing, collapse=", "))
  install.packages(missing, repos="https://cran.rstudio.com/")
}

library(plumber)
library(jsonlite)
library(DBI)
library(RMySQL)

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
  port = 5654
)

PORT         <- 8765
STOCK_MARGIN <- 1.0

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

mach_mat_history <- function(mat_code, material_id, consumed_qty, qty_before, qty_after) {
  mach_insert("material_history", data.frame(
    name=paste0("mat_",mat_code), time=now(),
    mat_code=as.integer(mat_code), material_id=as.integer(material_id),
    consumed_qty=as.numeric(consumed_qty),
    qty_before=as.numeric(qty_before), qty_after=as.numeric(qty_after),
    stringsAsFactors=FALSE))
}

mach_part_history <- function(part_code, material_id, consumed_qty, qty_before, qty_after) {
  mach_insert("part_history", data.frame(
    name=paste0("part_",part_code), time=now(),
    part_code=as.integer(part_code), material_id=as.integer(material_id),
    consumed_qty=as.integer(consumed_qty),
    qty_before=as.integer(qty_before), qty_after=as.integer(qty_after),
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

get_bom_requirements <- function(con, part_code, prod_qty) {
  sql <- sprintf(
    "SELECT b.mat_code, b.qty_per_unit,
            b.qty_per_unit * %d        AS required_qty,
            ms.quantity, ms.material_id,
            ms.quantity >= b.qty_per_unit * %d * %.2f AS sufficient
     FROM   bom b
     JOIN   material_stock ms ON ms.mat_code = b.mat_code
     WHERE  b.part_code = %d",
    prod_qty, prod_qty, STOCK_MARGIN, part_code)
  df <- dbGetQuery(con, sql)
  if (nrow(df)==0) return(data.frame())
  df$sufficient <- as.logical(df$sufficient)
  df
}

evaluate_permission <- function(df) {
  if (is.null(df) || nrow(df)==0)
    return(list(approved=FALSE, reason="BOM not found", materials=list()))
  bad <- df[!df$sufficient, ]
  if (nrow(bad)>0)
    return(list(approved=FALSE,
                reason=paste0("Low stock — ", paste(mat_name(bad$mat_code), collapse=", ")),
                materials=df_to_list(df)))
  list(approved=TRUE, reason="OK", materials=df_to_list(df))
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
    df     <- get_bom_requirements(con, part_code, prod_qty)
    result <- evaluate_permission(df)
    result$robot_id  <- robot_id
    result$part_code <- part_code
    result$prod_qty  <- prod_qty

    # ── 가독성 높은 로그 출력 ──────────────────────────────────────────────────
    message(SEP)
    if (result$approved) {
      log_ok(sprintf("check_work  robot=%-3d  %s x%d  =>  APPROVED",
                     robot_id, part_name(part_code), prod_qty))
      message(sprintf("  %s  %-20s  %8s  %8s  %8s",
                      "   ","MATERIAL","NEED","HAVE","STATUS"))
      for (m in result$materials) {
        st <- if (m$sufficient) " OK" else "LOW"
        message(sprintf("  BOM  %-20s  %8.1f  %8.1f  [%s]",
                        mat_name(m$mat_code), m$required_qty, m$quantity, st))
      }
    } else {
      log_warn(sprintf("check_work  robot=%-3d  %s x%d  =>  DENIED",
                       robot_id, part_name(part_code), prod_qty))
      log_warn("  reason: ", result$reason)
      for (m in result$materials) {
        st <- if (m$sufficient) " OK" else "LOW"
        message(sprintf("  BOM  %-20s  need=%7.1f  have=%7.1f  [%s]",
                        mat_name(m$mat_code), m$required_qty, m$quantity, st))
      }
    }
    message(SEP)
    result
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
        "SELECT quantity, material_id FROM material_stock WHERE mat_code=%d", mc))
      if (nrow(row)==0) next

      qty_before <- row$quantity
      qty_after  <- max(qty_before - used_qty, 0)

      dbExecute(con, sprintf(
        "UPDATE material_stock SET quantity=%.3f, updated_at=NOW() WHERE mat_code=%d",
        qty_after, mc))
      mach_mat_history(mc, row$material_id, -used_qty, qty_before, qty_after)

      updated_mats <- c(updated_mats, mc)
      message(sprintf("  MAT  %-20s  %8.1f  %10.1f  %10.1f",
                      mat_name(mc), used_qty, qty_before, qty_after))
    }

    # 부품 생산
    if (part_code>0 && prod_qty>0) {
      row <- dbGetQuery(con, sprintf(
        "SELECT quantity, material_id FROM part_stock WHERE part_code=%d", part_code))
      if (nrow(row)>0) {
        qty_before <- row$quantity
        qty_after  <- qty_before + prod_qty
        dbExecute(con, sprintf(
          "UPDATE part_stock SET quantity=%d, updated_at=NOW() WHERE part_code=%d",
          qty_after, part_code))
        mach_part_history(part_code, row$material_id, prod_qty, qty_before, qty_after)
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
        "SELECT quantity, material_id FROM material_stock WHERE mat_code=%d", code))
      if (nrow(row)==0) return(list(ok=FALSE, reason=paste("mat_code not found:", code)))
      qty_before <- row$quantity
      dbExecute(con, sprintf(
        "UPDATE material_stock SET quantity=%.3f, updated_at=NOW() WHERE mat_code=%d",
        quantity, code))
      mach_mat_history(code, row$material_id, quantity-qty_before, qty_before, quantity)
      item_nm <- mat_name(code)
    } else {
      row <- dbGetQuery(con, sprintf(
        "SELECT quantity, material_id FROM part_stock WHERE part_code=%d", code))
      if (nrow(row)==0) return(list(ok=FALSE, reason=paste("part_code not found:", code)))
      qty_before <- row$quantity
      dbExecute(con, sprintf(
        "UPDATE part_stock SET quantity=%d, updated_at=NOW() WHERE part_code=%d",
        as.integer(quantity), code))
      mach_part_history(code, row$material_id,
                        as.integer(quantity-qty_before),
                        as.integer(qty_before), as.integer(quantity))
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
    "SELECT name, time, mat_code, material_id, consumed_qty, qty_before, qty_after
     FROM material_history ORDER BY time DESC LIMIT 50")
  ph <- mach_query(
    "SELECT name, time, part_code, material_id, consumed_qty, qty_before, qty_after
     FROM part_history ORDER BY time DESC LIMIT 50")
  list(ok=TRUE, material_history=mh, part_history=ph)
}

# =============================================================================
# 9. 서버 기동
# =============================================================================

# 이름 캐시 로드
load_name_cache()

message(SEP2)
message("  FACTORY SERVER  (MySQL + Machbase)   port:", PORT)
message(SEP2)
message("  GET  /health   GET  /stock   GET  /history")
message("  POST /check_work   POST /update_stock   POST /set_stock")
message(SEP2)

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
