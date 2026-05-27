"""
monitor_V2.py — Smart Factory 실시간 모니터 (Textual TUI)
==========================================================
실행: python3 monitor_V2.py
필요: pip install textual mysql-connector-python

  별도 창으로 실행하려면:
    gnome-terminal -- python3 monitor_V2.py
    xterm -e python3 monitor_V2.py
"""

import json
import urllib.request
import urllib.parse
from datetime import datetime

try:
    import mysql.connector
    HAS_MYSQL = True
except ImportError:
    HAS_MYSQL = False

from textual.app        import App, ComposeResult
from textual.widgets    import DataTable, Label, Footer, Header, Static
from textual.containers import Horizontal, Vertical
from textual            import work
from rich.text          import Text

# =============================================================================
# 설정
# =============================================================================

MYSQL = {
    "host":     "127.0.0.1",
    "port":     3306,
    "database": "factory_db",
    "user":     "rokey",
    "password": "rokey1234",
}

MACHBASE_HTTP  = "http://127.0.0.1:5654"
R_SERVER       = "http://127.0.0.1:8765"
REFRESH_SEC    = 3
HISTORY_ROWS   = 50
HEARTBEAT_SEC  = 5    # R 서버로 생존 신고 주기 (초)

# =============================================================================
# DB 쿼리
# =============================================================================

def mysql_query(sql):
    if not HAS_MYSQL:
        return []
    try:
        conn = mysql.connector.connect(**MYSQL)
        cur  = conn.cursor(dictionary=True)
        cur.execute(sql)
        rows = cur.fetchall()
        cur.close(); conn.close()
        return rows
    except Exception as e:
        return [{"__error__": str(e)}]

def mach_query(sql):
    try:
        url = f"{MACHBASE_HTTP}/db/query?q={urllib.parse.quote(sql)}"
        with urllib.request.urlopen(url, timeout=3) as r:
            parsed = json.loads(r.read())
        if not parsed.get("success"):
            return []
        cols = parsed["data"]["columns"]
        rows = parsed["data"]["rows"] or []
        return [dict(zip(cols, row)) for row in rows]
    except Exception:
        return []

def fmt_time(raw):
    try:
        v = str(raw)
        if v.isdigit() and len(v) >= 15:
            ts = int(v) // 1_000_000_000
            return datetime.fromtimestamp(ts).strftime("%m-%d %H:%M:%S")
        return v[:19]
    except Exception:
        return str(raw)[:19]

def stock_status(qty, mx):
    if qty == 0:        return "EMPTY"
    if qty < mx * 0.1:  return "LOW"
    return "OK"

# =============================================================================
# CSS
# =============================================================================

CSS = """
Screen {
    background: #121212;
    layout: vertical;
}

#main {
    layout: horizontal;
    height: 1fr;
}

/* ── 좌측 — 배경색으로 구분, 보더 없음 ── */
#left-panel {
    width: 40%;
    height: 100%;
    background: #1a1a2e;
    padding: 0 1;
    margin-right: 1;
    layout: vertical;
}

#left-title {
    text-style: bold;
    color: #e8a838;
    height: 3;
    content-align: center middle;
}

/* 재고 섹션 — 각각 절반 고정 */
.stock-section {
    height: 1fr;
    layout: vertical;
}

.stock-section-label {
    text-style: bold;
    height: 2;
}

/* 재고 섹션 사이 구분선 */
#stock-divider {
    height: 1;
    background: #1e3a5f;
    margin: 0 0 1 0;
}

.stock-table {
    height: auto;
    width: 100%;
    margin-bottom: 1;

.stock-table > .datatable--cursor { background: transparent; }
.stock-table > .datatable--hover  { background: transparent; }
.stock-table > .datatable--header {
    text-style: bold;
    background: #152d4a;
}}

/* ── 우측 — 배경색으로 구분, 보더 없음 ── */
#right-panel {
    width: 60%;
    height: 100%;
    layout: vertical;
}

#right-title {
    text-style: bold;
    color: #e8a838;
    height: 3;
    content-align: center middle;
}

/* 히스토리 섹션 — 배경색으로 구분, 보더 없음 */
.hist-section {
    height: 1fr;
    layout: vertical;
    background: #1a1a2e;
    margin-bottom: 1;
}

.hist-section:last-of-type {
    margin-bottom: 0;
}

/* 히스토리 섹션 사이 구분선 */
#hist-divider {
    height: 1;
    background: #1e3a5f;
    color: #2196f3;
    content-align: center middle;
    margin: 0;
}

/* 제목 — 배경색으로 강조 */
.hist-title {
    text-style: bold;
    height: 2;
    padding: 0 1;
    background: #152d4a;
}

/* DataTable — 이 영역만 스크롤 */
.hist-table {
    height: 1fr;
    padding: 0 1;
}

.hist-table > .datatable--cursor { background: transparent; }
.hist-table > .datatable--hover  { background: transparent; }
.hist-table > .datatable--header {
    text-style: bold;
    background: #152d4a;
}

.stock-table > .datatable--cursor { background: transparent; }
.stock-table > .datatable--hover  { background: transparent; }
.stock-table > .datatable--header {
    text-style: bold;
    background: #152d4a;
}

"""

# =============================================================================
# App
# =============================================================================

class MonitorApp(App):
    CSS = CSS
    BINDINGS = [("q", "quit", "종료"), ("r", "refresh", "새로고침")]

    def action_quit(self) -> None:
        self.exit()

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="main"):

            # ── 좌측: 재고 현황 ──────────────────────────────────────────────
            with Vertical(id="left-panel"):
                yield Label("[ MySQL ]  재고 현황", id="left-title")
                # MATERIAL STOCK — 절반 고정
                with Vertical(classes="stock-section"):
                    yield Label("MATERIAL STOCK", classes="stock-section-label")
                    yield DataTable(id="mat-stock", show_cursor=False,
                                    classes="stock-table")
                yield Static("", id="stock-divider")
                # PART STOCK — 절반 고정
                with Vertical(classes="stock-section"):
                    yield Label("PART STOCK", classes="stock-section-label")
                    yield DataTable(id="part-stock", show_cursor=False,
                                    classes="stock-table")

            # ── 우측: 변동 이력 ──────────────────────────────────────────────
            with Vertical(id="right-panel"):
                yield Label("[ Machbase ]  변동 이력", id="right-title")

                # MATERIAL HISTORY — 제목 고정 + 표만 스크롤
                with Vertical(classes="hist-section"):
                    yield Label("MATERIAL HISTORY", classes="hist-title")
                    yield DataTable(id="mat-hist", show_cursor=False,
                                    classes="hist-table")

                # 섹션 구분선
                yield Static("", id="hist-divider")

                # PART HISTORY — 제목 고정 + 표만 스크롤
                with Vertical(classes="hist-section"):
                    yield Label("PART HISTORY", classes="hist-title")
                    yield DataTable(id="part-hist", show_cursor=False,
                                    classes="hist-table")

        yield Footer()

    def on_resize(self, event) -> None:
        """터미널 리사이즈 이벤트 무시 — 초기 크기 유지"""
        # Textual은 터미널 크기 변경을 막을 수 없으나
        # 레이아웃 재계산만 막아서 깨짐 최소화
        event.stop()

    def on_mount(self) -> None:
        self._mat_names  = {}
        self._part_names = {}
        # 컬럼 초기화 — 고정폭으로 양쪽 테이블 정렬 통일
        from textual.widgets.data_table import ColumnKey
        STOCK_COLS = [("CODE",4), ("NAME",12), ("QTY",8), ("MAX",8), ("UNIT",5), ("STATUS",9)]
        HIST_COLS  = [("TIME",17), ("NAME",12), ("DELTA",8), ("BEFORE",8), ("AFTER",8)]

        for tid in ("#mat-stock", "#part-stock"):
            t = self.query_one(tid, DataTable)
            for label, w in STOCK_COLS:
                t.add_column(label, width=w)

        for tid in ("#mat-hist", "#part-hist"):
            t = self.query_one(tid, DataTable)
            for label, w in HIST_COLS:
                t.add_column(label, width=w)
        self.set_interval(REFRESH_SEC, self.refresh_data)
        self.set_interval(HEARTBEAT_SEC, self._send_heartbeat)
        self.call_later(self.refresh_data)

    @work(thread=True)
    def _send_heartbeat(self) -> None:
        """R 서버에 모니터 생존 신고"""
        try:
            body = json.dumps({"alive": True}).encode("utf-8")
            req  = urllib.request.Request(
                f"{R_SERVER}/monitor_alive", data=body,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            urllib.request.urlopen(req, timeout=2)
        except Exception:
            pass   # R 서버 없어도 모니터는 계속 동작

    def refresh_data(self) -> None:
        self._reload_names()
        self._update_mat_stock()
        self._update_part_stock()
        self._update_mat_hist()
        self._update_part_hist()

    def action_refresh(self) -> None:
        self.refresh_data()

    # ── 이름 캐시 ─────────────────────────────────────────────────────────────
    def _reload_names(self):
        rows = mysql_query("SELECT mat_code, mat_name FROM material_master")
        self._mat_names = {str(r["mat_code"]): r["mat_name"]
                           for r in rows if "__error__" not in r}
        rows = mysql_query("SELECT part_code, part_name FROM part_master")
        self._part_names = {str(r["part_code"]): r["part_name"]
                            for r in rows if "__error__" not in r}

    def _mat_name(self, code):
        return self._mat_names.get(str(code), f"mat_{code}")

    def _part_name(self, code):
        return self._part_names.get(str(code), f"part_{code}")

    # ── MySQL: material_stock ─────────────────────────────────────────────────
    def _update_mat_stock(self):
        rows = mysql_query(
            "SELECT ms.mat_code, mm.mat_name, ms.quantity, ms.max_qty, um.unit_name "
            "FROM material_stock ms "
            "LEFT JOIN material_master mm ON mm.mat_code = ms.mat_code "
            "LEFT JOIN unit_master um ON um.unit_id = ms.unit_id "
            "ORDER BY ms.mat_code"
        )
        def _u():
            t = self.query_one("#mat-stock", DataTable)
            t.clear()
            for r in rows:
                if "__error__" in r:
                    t.add_row("ERR", str(r["__error__"])[:30],"","","",""); continue
                qty  = float(r["quantity"])
                mx   = float(r["max_qty"])
                stat = stock_status(qty, mx)
                sc   = {"OK":" OK  ","LOW":" LOW  ","EMPTY":" EMPTY"}.get(stat,stat)
                t.add_row(str(r["mat_code"]), str(r["mat_name"] or ""),
                          f"{qty:.1f}", f"{mx:.1f}",
                          str(r["unit_name"] or ""), sc)
        _u()

    # ── MySQL: part_stock ─────────────────────────────────────────────────────
    def _update_part_stock(self):
        rows = mysql_query(
            "SELECT ps.part_code, pm.part_name, ps.quantity, ps.max_qty, um.unit_name "
            "FROM part_stock ps "
            "LEFT JOIN part_master pm ON pm.part_code = ps.part_code "
            "LEFT JOIN unit_master um ON um.unit_id = ps.unit_id "
            "ORDER BY ps.part_code"
        )
        def _u():
            t = self.query_one("#part-stock", DataTable)
            t.clear()
            for r in rows:
                if "__error__" in r:
                    t.add_row("ERR", str(r["__error__"])[:30],"","","",""); continue
                qty  = float(r["quantity"])
                mx   = float(r["max_qty"])
                stat = stock_status(qty, mx)
                sc   = {"OK":" OK  ","LOW":" LOW  ","EMPTY":" EMPTY"}.get(stat,stat)
                t.add_row(str(r["part_code"]), str(r["part_name"] or ""),
                          str(qty), str(mx),
                          str(r["unit_name"] or ""), sc)
        _u()

    # ── Machbase: material_history ────────────────────────────────────────────
    def _update_mat_hist(self):
        rows = mach_query(
            f"SELECT time, mat_code, consumed_qty, qty_before, qty_after "
            f"FROM material_history ORDER BY time DESC LIMIT {HISTORY_ROWS}"
        )
        def _u():
            t = self.query_one("#mat-hist", DataTable)
            t.clear()
            if not rows:
                t.add_row("—","—","—","—","—"); return
            for r in rows:
                delta = float(r.get("consumed_qty", 0))
                sign  = "+" if delta >= 0 else ""
                dt    = (Text(f"▲ {sign}{delta:.1f}", style="bold red")
                         if delta >= 0 else
                         Text(f"▼ {delta:.1f}", style="bold #0055ff"))
                t.add_row(
                    fmt_time(r.get("time","")),
                    self._mat_name(r.get("mat_code","")),
                    dt,
                    f"{float(r.get('qty_before',0)):.1f}",
                    f"{float(r.get('qty_after',0)):.1f}",
                )
        _u()

    # ── Machbase: part_history ────────────────────────────────────────────────
    def _update_part_hist(self):
        rows = mach_query(
            f"SELECT time, part_code, consumed_qty, qty_before, qty_after "
            f"FROM part_history ORDER BY time DESC LIMIT {HISTORY_ROWS}"
        )
        def _u():
            t = self.query_one("#part-hist", DataTable)
            t.clear()
            if not rows:
                t.add_row("—","—","—","—","—"); return
            for r in rows:
                delta = float(r.get("consumed_qty", 0))
                sign  = "+" if delta >= 0 else ""
                dt    = (Text(f"▲ {sign}{delta:.1f}", style="bold red")
                         if delta >= 0 else
                         Text(f"▼ {delta:.1f}", style="bold #0055ff"))
                t.add_row(
                    fmt_time(r.get("time","")),
                    self._part_name(r.get("part_code","")),
                    dt,
                    f"{float(r.get('qty_before', 0)):.1f}",
                    f"{float(r.get('qty_after',  0)):.1f}",
                )
        _u()


# =============================================================================
# 메인 — 별도 터미널 창으로 실행
# =============================================================================

if __name__ == "__main__":
    import subprocess, sys, os

    if not HAS_MYSQL:
        print("pip install mysql-connector-python 실행 후 다시 시작하세요.")
        sys.exit(1)

    # 이미 --inner 플래그로 실행된 경우 → 실제 앱 실행
    if "--inner" in sys.argv:
        app = MonitorApp()
        app.title = "Smart Factory Monitor"
        app.run()
        sys.exit(0)

    # 화면 해상도 가져오기 (픽셀 → 터미널 문자 단위 변환)
    def get_terminal_screen_size():
        try:
            import tkinter as tk
            root = tk.Tk(); root.withdraw()
            sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
            root.destroy()
            # 픽셀 → 터미널 문자 근사치 (폰트 8x16 가정)
            cols = sw // 8 // 2    # 화면 가로 절반
            rows = sh // 16 // 2   # 화면 세로 절반
            return max(cols, 120), max(rows, 35)
        except Exception:
            return 160, 40         # fallback

    cols, rows = get_terminal_screen_size()
    geometry   = f"{cols}x{rows}"
    script     = os.path.abspath(__file__)
    cmd        = None

    for term, args in [
        # gnome-terminal: --geometry로 크기 고정 + resize 비활성화
        ("gnome-terminal", [
            f"--geometry={geometry}",
            "--",
            "bash", "-c",
            f"python3 {script} --inner; read -p 'Press Enter to close'"
        ]),
        # xterm: 크기 고정 + resizable=false
        ("xterm", [
            "-title", "Smart Factory Monitor",
            "-geometry", geometry,
            "-resizable", "false",
            "-e", f"python3 {script} --inner"
        ]),
        ("konsole", [
            "--geometry", f"{cols*7}x{rows*15}",   # konsole는 픽셀 단위
            "--",
            "python3", script, "--inner"
        ]),
        ("xfce4-terminal", [
            f"--geometry={geometry}",
            "--",
            "python3", script, "--inner"
        ]),
        ("lxterminal", [
            f"--geometry={geometry}",
            "-e", f"python3 {script} --inner"
        ]),
    ]:
        if subprocess.run(["which", term], capture_output=True).returncode == 0:
            cmd = [term] + args
            break

    if cmd:
        print(f"Opening monitor ({cmd[0]}, size={geometry})...")
        subprocess.Popen(cmd)
    else:
        print("No supported terminal found — running in current terminal.")
        app = MonitorApp()
        app.title = "Smart Factory Monitor"
        app.run()
