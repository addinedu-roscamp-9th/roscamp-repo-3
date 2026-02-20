import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import json
from datetime import datetime
import websocket
from urllib.parse import urlencode
import threading


class RetroButton(tk.Button):
    """Retro-styled button"""

    def __init__(self, parent, text, command=None, bg="#e8e8e8", **kwargs):
        super().__init__(
            parent,
            text=text,
            command=command,
            font=("Courier New", 12, "bold"),
            bg=bg,
            fg="black" if bg in ["#e8e8e8", "#00ff00", "#ff9800"] else "white",
            relief="solid",
            bd=3,
            padx=20,
            pady=10,
            cursor="hand2",
            **kwargs,
        )
        self.default_bg = bg
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, e):
        self["bg"] = self.darken_color(self.default_bg)

    def on_leave(self, e):
        self["bg"] = self.default_bg

    def darken_color(self, hex_color):
        """Darken a hex color"""
        hex_color = hex_color.lstrip("#")
        r, g, b = tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
        r = int(r * 0.9)
        g = int(g * 0.9)
        b = int(b * 0.9)
        return f"#{r:02x}{g:02x}{b:02x}"


class ConsoleWidget(scrolledtext.ScrolledText):
    """Console output widget"""

    def __init__(self, parent):
        super().__init__(
            parent,
            height=8,
            font=("Courier New", 10),
            bg="#1e1e1e",
            fg="#00ff00",
            state="disabled",
            wrap="word",
        )
        self.log_count = 0

    def log(self, message, is_error=False):
        """Add a log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        color = "#ff5555" if is_error else "#00ff00"

        self.config(state="normal")
        self.insert("end", f"[{timestamp}] {message}\n")
        self.see("end")
        self.config(state="disabled")

        self.log_count += 1
        if self.log_count > 50:
            self.config(state="normal")
            self.delete("1.0", "2.0")
            self.config(state="disabled")


class PopupSelector(tk.Toplevel):
    """Popup selector dialog"""

    def __init__(self, parent, title, options):
        super().__init__(parent)
        self.title(title)
        self.geometry("600x400")
        self.configure(bg="#c0c0c0")
        self.resizable(False, False)

        self.selected_value = None
        self.selected_index = None

        # Center window
        self.transient(parent)
        self.grab_set()

        # Title
        title_label = tk.Label(
            self, text=title, font=("Courier New", 14, "bold"), bg="#c0c0c0"
        )
        title_label.pack(pady=10)

        # Options frame
        options_frame = tk.Frame(self, bg="#c0c0c0")
        options_frame.pack(fill="both", expand=True, padx=20, pady=10)

        # Create grid of options
        cols = 3
        for i, option in enumerate(options):
            btn = RetroButton(
                options_frame,
                option,
                command=lambda v=option, idx=i: self.select_option(v, idx),
            )
            btn.grid(row=i // cols, column=i % cols, padx=5, pady=5, sticky="ew")

        # Configure grid columns
        for i in range(cols):
            options_frame.columnconfigure(i, weight=1)

        # Close button
        close_btn = RetroButton(self, "닫기", command=self.destroy, bg="#f44336")
        close_btn.pack(pady=10)

    def select_option(self, value, index):
        """Handle option selection"""
        self.selected_value = value
        self.selected_index = index
        self.destroy()


class BaseScreen(tk.Frame):
    """Base class for all screens"""

    def __init__(self, parent, app):
        super().__init__(parent, bg="#c0c0c0")
        self.app = app


class LoginScreen(BaseScreen):
    """Login screen"""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        center_frame = tk.Frame(self, bg="#c0c0c0")
        center_frame.place(relx=0.5, rely=0.4, anchor="center")

        self.status_label = tk.Label(
            center_frame,
            text=f"",
            font=("Courier New", 10),
            bg="#c0c0c0",
            fg="#666",
        )
        self.status_label.pack(pady=(0, 10))

        login_box = tk.Frame(center_frame, bg="#808080", relief="solid", bd=3)
        login_box.pack(padx=20, pady=20)

        title = tk.Label(
            login_box, text="Login", font=("Courier New", 16, "bold"), bg="#808080"
        )
        title.pack(pady=(20, 10))

        id_label = tk.Label(
            login_box, text="I D :", font=("Courier New", 12), bg="#808080"
        )
        id_label.pack(anchor="w", padx=20)

        self.user_id = tk.Entry(
            login_box, font=("Courier New", 12), width=30, relief="solid", bd=2
        )
        self.user_id.pack(padx=20, pady=(0, 10))

        pw_label = tk.Label(
            login_box, text="Pw :", font=("Courier New", 12), bg="#808080"
        )
        pw_label.pack(anchor="w", padx=20)

        self.user_pw = tk.Entry(
            login_box,
            font=("Courier New", 12),
            width=30,
            show="*",
            relief="solid",
            bd=2,
        )
        self.user_pw.pack(padx=20, pady=(0, 10))
        self.user_pw.bind("<Return>", lambda e: self.login())

        login_btn = RetroButton(login_box, "로그인", command=self.login, bg="#4a90e2")
        login_btn.pack(padx=20, pady=(10, 20), fill="x")

        connect_btn = RetroButton(
            center_frame, "Server connect", command=self.connect_server, bg="#00ff00"
        )
        connect_btn.pack(pady=10)

        help_text = tk.Label(
            center_frame, text="", font=("Courier New", 9), bg="#c0c0c0", fg="#666"
        )
        help_text.pack(pady=(10, 0))

    def login(self):
        user_id = self.user_id.get().strip()
        user_pw = self.user_pw.get().strip()

        if not user_id or not user_pw:
            self.app.console.log("ID와 비밀번호를 입력하세요", True)
            return

        result = self.app.send_message("login", {"id": user_id, "pw": user_pw})

        if result["success"]:
            user_name = result.get("data", {}).get("data", {}).get("user_name")
            if user_name:
                self.app.current_user = user_id
                self.app.show_screen("home")
                self.app.console.log(f"로그인 성공! ({user_name})")
                messagebox.showinfo("환영합니다", f"{user_name}님 환영합니다!")
            else:
                self.app.console.log(
                    "로그인 실패 - ID 또는 비밀번호를 확인하세요", True
                )
                self.user_pw.delete(0, "end")
        else:
            self.app.console.log("로그인 실패", True)
            self.user_pw.delete(0, "end")

    def connect_server(self):
        result = self.app.send_message("connect")
        if result["success"]:
            self.app.console.log("서버에 연결되었습니다!")
        else:
            self.app.console.log(f"서버 연결 실패: {result.get('error', '')}", True)


class HomeScreen(BaseScreen):
    """Home screen with menu buttons"""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        center_frame = tk.Frame(self, bg="#c0c0c0")
        center_frame.place(relx=0.5, rely=0.5, anchor="center")

        buttons = [
            ("물건 가져오기", lambda: self.app.show_screen("fetch")),
            ("물건 갖다놓기", lambda: self.app.show_screen("take")),
            ("  스케줄    ", lambda: self.app.show_screen("schedule_list")),
            ("   기록    ", lambda: self.app.show_screen("history")),
        ]

        for text, command in buttons:
            btn = RetroButton(center_frame, text, command=command)
            btn.pack(pady=10, ipadx=40, ipady=20)


class FetchScreen(BaseScreen):
    """Fetch screen for selecting room"""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.selected_room = None
        self.room_buttons = []
        self.rooms = [" 안방 ", " 거실 "]
        self.setup_ui()

    def setup_ui(self):
        back_btn = RetroButton(
            self, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        )
        back_btn.pack(anchor="w", padx=10, pady=10)

        title = tk.Label(
            self, text="물건 가져오기", font=("Courier New", 20, "bold"), bg="#c0c0c0"
        )
        title.pack(pady=10)

        content_box = tk.Frame(self, bg="white", relief="solid", bd=3)
        content_box.pack(padx=20, pady=10, fill="both", expand=True)

        instruction = tk.Label(
            content_box,
            text="현재 있는 방 선택:",
            font=("Courier New", 12, "bold"),
            bg="white",
        )
        instruction.pack(pady=10)

        grid_frame = tk.Frame(content_box, bg="white")
        grid_frame.pack(pady=20)

        for i, room in enumerate(self.rooms):
            btn = RetroButton(
                grid_frame, room, command=lambda r=room: self.select_room(r)
            )
            btn.grid(row=i // 3, column=i % 3, padx=10, pady=10, ipadx=20, ipady=20)
            self.room_buttons.append(btn)

        execute_btn = RetroButton(
            self, "물건 가져오기 실행", command=self.execute_fetch, bg="#4a90e2"
        )
        execute_btn.pack(pady=20, ipadx=40, ipady=20)

    def select_room(self, room):
        for btn in self.room_buttons:
            btn["bg"] = "#e8e8e8"
        for btn in self.room_buttons:
            if btn["text"] == room:
                btn["bg"] = "#4a90e2"
                btn["fg"] = "white"
        self.selected_room = room

    def execute_fetch(self):
        if not self.selected_room:
            self.app.console.log("방을 선택하세요", True)
            return

        result = self.app.send_message("fetch_req")
        if not result["success"]:
            self.app.console.log(f"요청 실패: {result.get('error', '')}", True)
            return

        result = self.app.send_message(
            "fetch_cmd", {"item_id": "item_001", "position_id": self.selected_room}
        )

        if result["success"]:
            self.app.console.log(f"물건 가져오기 명령 전송 - 방: {self.selected_room}")
            self.app.show_screen("home")
        else:
            self.app.console.log("명령 전송 실패", True)


class TakeScreen(BaseScreen):
    """Take screen"""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        back_btn = RetroButton(
            self, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        )
        back_btn.pack(anchor="w", padx=10, pady=10)

        title = tk.Label(
            self, text="물건 갖다놓기", font=("Courier New", 20, "bold"), bg="#c0c0c0"
        )
        title.pack(pady=10)

        center_frame = tk.Frame(self, bg="#c0c0c0")
        center_frame.place(relx=0.5, rely=0.5, anchor="center")

        execute_btn = RetroButton(
            center_frame, "물건 갖다놓기 실행", command=self.execute_take, bg="#ff9800"
        )
        execute_btn.pack(ipadx=40, ipady=30)

    def execute_take(self):
        result = self.app.send_message("take_req")
        if not result["success"]:
            self.app.console.log(f"요청 실패: {result.get('error', '')}", True)
            return

        result = self.app.send_message("take_cmd", {"position_id": "default_position"})

        if result["success"]:
            self.app.console.log("물건 갖다놓기 명령 전송")
            self.app.show_screen("home")
        else:
            self.app.console.log("명령 전송 실패", True)


class ScheduleListScreen(BaseScreen):
    """Schedule list screen"""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        header = tk.Frame(self, bg="#c0c0c0")
        header.pack(fill="x", padx=10, pady=10)

        back_btn = RetroButton(
            header, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        )
        back_btn.pack(side="left")

        title = tk.Label(
            header, text="   스케줄   ", font=("Courier New", 16, "bold"), bg="#c0c0c0"
        )
        title.pack(side="left", padx=20)

        refresh_btn = RetroButton(
            header, "새로고침", command=self.load_schedules, bg="#4a90e2"
        )
        refresh_btn.pack(side="right")

        list_frame = tk.Frame(self, bg="white", relief="solid", bd=3)
        list_frame.pack(fill="both", expand=True, padx=20, pady=10)

        self.schedule_canvas = tk.Canvas(list_frame, bg="white", highlightthickness=0)
        scrollbar = tk.Scrollbar(
            list_frame, orient="vertical", command=self.schedule_canvas.yview
        )

        self.schedule_container = tk.Frame(self.schedule_canvas, bg="white")
        self.schedule_canvas.create_window(
            (0, 0), window=self.schedule_container, anchor="nw"
        )
        self.schedule_canvas.configure(yscrollcommand=scrollbar.set)

        self.schedule_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.schedule_container.bind(
            "<Configure>",
            lambda e: self.schedule_canvas.configure(
                scrollregion=self.schedule_canvas.bbox("all")
            ),
        )

        button_frame = tk.Frame(self, bg="#c0c0c0")
        button_frame.pack(fill="x", padx=20, pady=10)

        add_btn = RetroButton(
            button_frame, "추가 (Add)", command=self.show_add_schedule, bg="#4caf50"
        )
        add_btn.pack(side="left", fill="x", expand=True, padx=(0, 5))

        delete_btn = RetroButton(
            button_frame, "삭제 (Delete)", command=self.delete_selected, bg="#f44336"
        )
        delete_btn.pack(side="right", fill="x", expand=True, padx=(5, 0))

    def load_schedules(self):
        result = self.app.send_message("schedule_req")

        for widget in self.schedule_container.winfo_children():
            widget.destroy()

        if result["success"]:
            schedules = result.get("data", {}).get("schedules", [])

            if not schedules:
                empty_label = tk.Label(
                    self.schedule_container,
                    text="등록된 스케줄이 없습니다",
                    font=("Courier New", 12),
                    fg="#666",
                    bg="white",
                )
                empty_label.pack(pady=50)
            else:
                for schedule in schedules:
                    self.create_schedule_item(schedule)

            self.app.console.log("스케줄을 불러왔습니다")
        else:
            error_label = tk.Label(
                self.schedule_container,
                text="스케줄 로드 실패",
                font=("Courier New", 12),
                fg="red",
                bg="white",
            )
            error_label.pack(pady=50)
            self.app.console.log("스케줄 로드 실패", True)

    def create_schedule_item(self, schedule):
        item = tk.Frame(self.schedule_container, bg="#e8e8e8", relief="solid", bd=2)
        item.pack(fill="x", padx=10, pady=5)

        info_frame = tk.Frame(item, bg="#e8e8e8")
        info_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        tk.Label(
            info_frame,
            text=f"시간: {schedule.get('execute_time', 'N/A')}",
            font=("Courier New", 10),
            bg="#e8e8e8",
            anchor="w",
        ).pack(fill="x")
        tk.Label(
            info_frame,
            text=f"물건: {schedule.get('item_id', 'N/A')}",
            font=("Courier New", 10),
            bg="#e8e8e8",
            anchor="w",
        ).pack(fill="x")
        tk.Label(
            info_frame,
            text=f"장소: {schedule.get('position_id', 'N/A')}",
            font=("Courier New", 10),
            bg="#e8e8e8",
            anchor="w",
        ).pack(fill="x")
        tk.Label(
            info_frame,
            text=f"주기: {schedule.get('cycle', 'N/A')}일",
            font=("Courier New", 10),
            bg="#e8e8e8",
            anchor="w",
        ).pack(fill="x")

        btn_frame = tk.Frame(item, bg="#e8e8e8")
        btn_frame.pack(side="right", padx=10)

        edit_btn = RetroButton(
            btn_frame,
            "수정",
            command=lambda: self.edit_schedule(schedule.get("schedule_id")),
            bg="#ff9800",
        )
        edit_btn.pack(side="left", padx=2)

        delete_btn = RetroButton(
            btn_frame,
            "삭제",
            command=lambda: self.delete_schedule(schedule.get("schedule_id")),
            bg="#f44336",
        )
        delete_btn.pack(side="left", padx=2)

    def show_add_schedule(self):
        self.app.screens["schedule_edit"].reset_for_new()
        self.app.show_screen("schedule_edit")

    def edit_schedule(self, schedule_id):
        self.app.screens["schedule_edit"].editing_schedule_id = schedule_id
        self.app.show_screen("schedule_edit")

    def delete_schedule(self, schedule_id):
        result = self.app.send_message(
            "schedule_edit",
            {"action": "delete", "schedule": {"schedule_id": schedule_id}},
        )

        if result["success"]:
            self.app.console.log("스케줄이 삭제되었습니다")
            self.load_schedules()
        else:
            self.app.console.log("스케줄 삭제 실패", True)

    def delete_selected(self):
        self.app.console.log("삭제할 스케줄을 목록에서 선택하세요")


# ─────────────────────────────────────────────────────────────────────────────
# DRUM ROLL COMPONENTS
# ─────────────────────────────────────────────────────────────────────────────


class DrumRollColumn(tk.Frame):
    """A single drum-roll (slot-machine style) column."""

    ITEM_H = 72  # pixel height of each cell
    VISIBLE = 3  # how many cells are visible (odd number keeps center clear)
    ANIM_STEPS = 8  # animation steps per scroll tick
    ANIM_MS = 12  # ms between animation frames

    def __init__(self, parent, items, width=110, **kwargs):
        canvas_h = self.ITEM_H * self.VISIBLE
        super().__init__(parent, bg="#111111", width=width, height=canvas_h, **kwargs)
        self.pack_propagate(False)

        self.items = items
        self.col_width = width
        self._index = 0
        self._anim_job = None
        self._drag_start_y = 0
        self._drag_start_idx = 0

        # Canvas
        self.canvas = tk.Canvas(
            self,
            width=width,
            height=canvas_h,
            bg="#111111",
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)

        # Selection highlight (centre row)
        mid_y0 = self.ITEM_H
        mid_y1 = self.ITEM_H * 2
        self.canvas.create_rectangle(
            3,
            mid_y0,
            width - 3,
            mid_y1,
            outline="#4a90e2",
            width=2,
            fill="#1c2340",
            tags="highlight",
        )
        # Top/bottom fade overlays
        self.canvas.create_rectangle(
            0,
            0,
            width,
            self.ITEM_H,
            fill="#111111",
            stipple="gray50",
            outline="",
            tags="fade",
        )
        self.canvas.create_rectangle(
            0,
            self.ITEM_H * 2,
            width,
            canvas_h,
            fill="#111111",
            stipple="gray50",
            outline="",
            tags="fade",
        )

        self._draw(0)

        # Bindings
        for widget in (self, self.canvas):
            widget.bind("<MouseWheel>", self._on_wheel)
            widget.bind("<Button-4>", lambda e: self._step(-1))
            widget.bind("<Button-5>", lambda e: self._step(1))
            widget.bind("<ButtonPress-1>", self._drag_start)
            widget.bind("<B1-Motion>", self._drag_move)
            widget.bind("<ButtonRelease-1>", self._drag_end)

    # ── drawing ──────────────────────────────────────────────────────────────

    def _draw(self, pixel_offset=0):
        self.canvas.delete("label")
        for row in range(self.VISIBLE):
            idx = (self._index + row - 1) % len(self.items)
            y = row * self.ITEM_H + self.ITEM_H // 2 + pixel_offset
            is_center = row == 1
            self.canvas.create_text(
                self.col_width // 2,
                y,
                text=self.items[idx],
                font=(
                    "Courier New",
                    15 if is_center else 11,
                    "bold" if is_center else "normal",
                ),
                fill="#ffffff" if is_center else "#444444",
                tags="label",
            )
        self.canvas.tag_raise("highlight")
        self.canvas.tag_raise("label")
        self.canvas.tag_raise("fade")

    # ── scroll / animation ───────────────────────────────────────────────────

    def _step(self, direction, extra_callback=None):
        """Scroll one item in direction (+1 = down = next item)."""
        if self._anim_job is not None:
            self.after_cancel(self._anim_job)
            self._anim_job = None

        self._index = (self._index + direction) % len(self.items)
        self._animate(-direction * self.ITEM_H, 0, extra_callback)

    def _animate(self, start_offset, step, callback):
        remaining = self.ANIM_STEPS - step
        if remaining <= 0:
            self._draw(0)
            if callback:
                callback()
            return
        offset = int(start_offset * remaining / self.ANIM_STEPS)
        self._draw(offset)
        self._anim_job = self.after(
            self.ANIM_MS,
            lambda: self._animate(start_offset, step + 1, callback),
        )

    # ── event handlers ───────────────────────────────────────────────────────

    def _on_wheel(self, event):
        direction = -1 if event.delta > 0 else 1
        self._step(direction)

    def _drag_start(self, event):
        self._drag_start_y = event.y
        self._drag_start_idx = self._index

    def _drag_move(self, event):
        delta = event.y - self._drag_start_y
        steps = int(-delta / (self.ITEM_H // 2))
        new_idx = (self._drag_start_idx + steps) % len(self.items)
        if new_idx != self._index:
            self._index = new_idx
        pixel_offset = (
            -(delta % self.ITEM_H) if delta >= 0 else ((-delta) % self.ITEM_H)
        )
        self._draw(int(pixel_offset * 0.6))

    def _drag_end(self, event):
        self._draw(0)

    # ── public API ───────────────────────────────────────────────────────────

    def get_value(self):
        return self.items[self._index]

    def get_index(self):
        return self._index

    def set_index(self, idx):
        self._index = idx % len(self.items)
        self._draw(0)


# ─────────────────────────────────────────────────────────────────────────────
# SCHEDULE EDIT SCREEN  (drum-roll version)
# ─────────────────────────────────────────────────────────────────────────────


class ScheduleEditScreen(BaseScreen):
    """Schedule add/edit screen with a drum-roll picker UI."""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.configure(bg="#111111")
        self.editing_schedule_id = None

        self.hours = [str(h).zfill(2) for h in range(24)]
        self.minutes = [str(m).zfill(2) for m in range(0, 60, 5)]
        self.items = ["초코스틱", "우유팩", "약통"]
        self.locations = ["안방", "거실", "Pick Up"]
        self.cycles = [f"{i}일 주기" for i in range(1, 8)]

        self._setup_ui()
        self._start_preview_loop()

    def _setup_ui(self):
        # ── Header ──────────────────────────────────────────────────────────
        header = tk.Frame(self, bg="#111111")
        header.pack(fill="x", padx=15, pady=(12, 4))

        tk.Button(
            header,
            text="◀ 뒤로가기",
            command=self.cancel_edit,
            font=("Courier New", 11, "bold"),
            bg="#222222",
            fg="#888888",
            relief="flat",
            padx=12,
            pady=7,
            cursor="hand2",
        ).pack(side="left")

        self.title_lbl = tk.Label(
            header,
            text="스케줄 추가",
            font=("Courier New", 17, "bold"),
            bg="#111111",
            fg="#ffffff",
        )
        self.title_lbl.pack(side="left", padx=16)

        # ── Preview bar ─────────────────────────────────────────────────────
        preview_bar = tk.Frame(self, bg="#141428")
        preview_bar.pack(fill="x", padx=15, pady=(0, 8))

        self.preview_lbl = tk.Label(
            preview_bar,
            text="09:00  |  초코스틱  |  안방  |  1일 주기",
            font=("Courier New", 12),
            bg="#141428",
            fg="#4a90e2",
            pady=7,
        )
        self.preview_lbl.pack()

        # ── Column labels ────────────────────────────────────────────────────
        col_specs = [
            ("시(Hour)", self.hours, 90),
            ("분(Min)", self.minutes, 90),
            ("물건", self.items, 140),
            ("장소", self.locations, 130),
            ("주기", self.cycles, 130),
        ]

        labels_row = tk.Frame(self, bg="#111111")
        labels_row.pack(padx=15, fill="x")

        for lbl_text, _, w in col_specs:
            cell = tk.Frame(labels_row, bg="#111111", width=w)
            cell.pack(side="left", padx=4)
            cell.pack_propagate(False)
            tk.Label(
                cell,
                text=lbl_text,
                font=("Courier New", 9),
                bg="#111111",
                fg="#555555",
            ).pack()

        # ── Drum-roll columns ────────────────────────────────────────────────
        drums_row = tk.Frame(self, bg="#111111")
        drums_row.pack(padx=15, pady=4, fill="both", expand=True)

        self.hour_drum = DrumRollColumn(drums_row, self.hours, width=90)
        self.minute_drum = DrumRollColumn(drums_row, self.minutes, width=90)
        self.item_drum = DrumRollColumn(drums_row, self.items, width=140)
        self.location_drum = DrumRollColumn(drums_row, self.locations, width=130)
        self.cycle_drum = DrumRollColumn(drums_row, self.cycles, width=130)

        for drum in (
            self.hour_drum,
            self.minute_drum,
            self.item_drum,
            self.location_drum,
            self.cycle_drum,
        ):
            drum.pack(side="left", padx=4)

        # Default: 09:00
        self.hour_drum.set_index(9)

        # ── Action buttons ───────────────────────────────────────────────────
        btn_row = tk.Frame(self, bg="#111111")
        btn_row.pack(fill="x", padx=15, pady=12)

        tk.Button(
            btn_row,
            text="저장",
            command=self.save_schedule,
            font=("Courier New", 13, "bold"),
            bg="#4a90e2",
            fg="white",
            relief="flat",
            padx=20,
            pady=12,
            cursor="hand2",
        ).pack(side="left", fill="x", expand=True, padx=(0, 5))

        tk.Button(
            btn_row,
            text="취소",
            command=self.cancel_edit,
            font=("Courier New", 13, "bold"),
            bg="#2a2a2a",
            fg="#888888",
            relief="flat",
            padx=20,
            pady=12,
            cursor="hand2",
        ).pack(side="right", fill="x", expand=True, padx=(5, 0))

    # ── preview loop ─────────────────────────────────────────────────────────

    def _start_preview_loop(self):
        self._update_preview()

    def _update_preview(self):
        try:
            h = self.hour_drum.get_value()
            m = self.minute_drum.get_value()
            it = self.item_drum.get_value()
            lo = self.location_drum.get_value()
            cy = self.cycle_drum.get_value()
            self.preview_lbl.config(text=f"{h}:{m}  |  {it}  |  {lo}  |  {cy}")
        except Exception:
            pass
        self.after(150, self._update_preview)

    # ── public API ────────────────────────────────────────────────────────────

    def reset_for_new(self):
        self.editing_schedule_id = None
        self.title_lbl["text"] = "스케줄 추가"
        self.hour_drum.set_index(9)
        self.minute_drum.set_index(0)
        self.item_drum.set_index(0)
        self.location_drum.set_index(0)
        self.cycle_drum.set_index(0)

    def save_schedule(self):
        hour = self.hour_drum.get_value()
        minute = self.minute_drum.get_value()
        execute_time = f"{hour}:{minute}"
        item = self.item_drum.get_value()
        location = self.location_drum.get_value()
        cycle_n = self.cycle_drum.get_index() + 1

        schedule_id = (
            self.editing_schedule_id
            or f"schedule_{int(datetime.now().timestamp() * 1000)}"
        )

        result = self.app.send_message(
            "schedule_edit",
            {
                "action": "edit" if self.editing_schedule_id else "add",
                "schedule": {
                    "schedule_id": schedule_id,
                    "cmd_id": "fetch",
                    "item_id": item,
                    "position_id": location,
                    "execute_time": execute_time,
                    "cycle": cycle_n,
                    "on_weekends": False,
                },
            },
        )

        server_success = result.get("data", {}).get("data", {}).get("success", False)
        if result["success"] and server_success:
            msg = (
                "스케줄이 수정되었습니다"
                if self.editing_schedule_id
                else "스케줄이 추가되었습니다"
            )
            self.app.console.log(msg)
            self.app.show_screen("schedule_list")
            self.app.screens["schedule_list"].load_schedules()
        else:
            self.app.console.log("스케줄 저장 실패", True)

    def cancel_edit(self):
        self.app.show_screen("schedule_list")


class HistoryScreen(BaseScreen):
    """History screen"""

    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        header = tk.Frame(self, bg="#c0c0c0")
        header.pack(fill="x", padx=10, pady=10)

        back_btn = RetroButton(
            header, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        )
        back_btn.pack(side="left")

        title = tk.Label(
            header, text="기록 보관", font=("Courier New", 16, "bold"), bg="#c0c0c0"
        )
        title.pack(side="left", padx=20)

        refresh_btn = RetroButton(
            header, "새로고침", command=self.load_history, bg="#4a90e2"
        )
        refresh_btn.pack(side="right")

        content_box = tk.Frame(self, bg="white", relief="solid", bd=3)
        content_box.pack(fill="both", expand=True, padx=20, pady=10)

        self.history_text = scrolledtext.ScrolledText(
            content_box, font=("Courier New", 10), bg="white", wrap="word"
        )
        self.history_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.history_text.insert(
            "1.0", "히스토리를 불러오려면 새로고침 버튼을 누르세요"
        )

    def load_history(self):
        result = self.app.send_message("history_req")
        self.history_text.delete("1.0", "end")

        if result["success"]:
            self.history_text.insert(
                "1.0", json.dumps(result.get("data", {}), indent=2, ensure_ascii=False)
            )
            self.app.console.log("히스토리를 불러왔습니다")
        else:
            self.history_text.insert(
                "1.0", f"히스토리 로드 실패\n\n{result.get('error', '')}"
            )
            self.app.console.log("히스토리 로드 실패", True)


class PinkyRobotGUI:
    """Main application"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Pinky Robot Control")
        self.root.geometry("900x700")
        self.root.configure(bg="#c0c0c0")

        self.SERVER_URL = "ws://192.168.0.48:8000/gui"
        self.current_user = None

        self.setup_menu()
        self.setup_ui()

    def setup_menu(self):
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        settings_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="설정", menu=settings_menu)
        settings_menu.add_command(
            label="서버 주소 변경", command=self.change_server_url
        )
        settings_menu.add_separator()
        settings_menu.add_command(label="종료", command=self.root.quit)

        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="도움말", menu=help_menu)
        help_menu.add_command(
            label="서버 연결 테스트", command=self.test_server_connection
        )
        help_menu.add_command(label="정보", command=self.show_about)

    def change_server_url(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("서버 주소 변경")
        dialog.geometry("500x150")
        dialog.configure(bg="#c0c0c0")
        dialog.transient(self.root)
        dialog.grab_set()

        tk.Label(dialog, text="서버 URL:", font=("Courier New", 12), bg="#c0c0c0").pack(
            pady=10
        )

        url_entry = tk.Entry(dialog, font=("Courier New", 12), width=40)
        url_entry.insert(0, self.SERVER_URL)
        url_entry.pack(pady=5)

        def save_url():
            new_url = url_entry.get().strip()
            if new_url:
                self.SERVER_URL = new_url
                self.console.log(f"서버 URL 변경: {new_url}")
                dialog.destroy()

        btn_frame = tk.Frame(dialog, bg="#c0c0c0")
        btn_frame.pack(pady=10)

        RetroButton(btn_frame, "저장", command=save_url, bg="#4caf50").pack(
            side="left", padx=5
        )
        RetroButton(btn_frame, "취소", command=dialog.destroy, bg="#f44336").pack(
            side="left", padx=5
        )

    def test_server_connection(self):
        self.console.log("서버 연결 테스트 중...")
        result = self.send_message("connect")

        if result["success"]:
            messagebox.showinfo(
                "연결 성공", f"서버에 정상적으로 연결되었습니다!\n\n{self.SERVER_URL}"
            )
        else:
            messagebox.showerror(
                "연결 실패",
                f"서버에 연결할 수 없습니다.\n\n서버 주소: {self.SERVER_URL}\n오류: {result.get('error', '알 수 없음')}\n\n서버가 실행 중인지 확인하세요.",
            )

    def show_about(self):
        messagebox.showinfo(
            "Pinky Robot Control",
            "Pinky Robot Control GUI\n\nVersion: 1.0\nPython tkinter 기반\n\n"
            f"현재 서버: {self.SERVER_URL}",
        )

    def setup_ui(self):
        main_frame = tk.Frame(self.root, bg="#c0c0c0")
        main_frame.pack(fill="both", expand=True)

        self.screen_container = tk.Frame(main_frame, bg="#c0c0c0")
        self.screen_container.pack(fill="both", expand=True)

        self.screens = {}
        self.screens["login"] = LoginScreen(self.screen_container, self)
        self.screens["home"] = HomeScreen(self.screen_container, self)
        self.screens["fetch"] = FetchScreen(self.screen_container, self)
        self.screens["take"] = TakeScreen(self.screen_container, self)
        self.screens["schedule_list"] = ScheduleListScreen(self.screen_container, self)
        self.screens["history"] = HistoryScreen(self.screen_container, self)
        self.screens["schedule_edit"] = ScheduleEditScreen(self.screen_container, self)

        for screen in self.screens.values():
            screen.place(relx=0, rely=0, relwidth=1, relheight=1)

        self.console = ConsoleWidget(main_frame)
        self.console.pack(fill="x", side="bottom")

        self.show_screen("login")
        self.console.log("System ready")

    def send_message(self, msg, data=None):
        if data is None:
            data = {}

        payload = {"msg": msg, "data": data}
        self.console.log(f"SEND: {json.dumps(payload, ensure_ascii=False)}")

        try:
            ws = websocket.create_connection(self.SERVER_URL, timeout=5)
            ws.send(json.dumps(payload))
            response = ws.recv()
            ws.close()

            result = json.loads(response)
            self.console.log(f"RECV: {json.dumps(result, ensure_ascii=False)}")
            return {"success": True, "data": result}

        except Exception as e:
            error_msg = f"WebSocket 오류: {str(e)}"
            self.console.log(f"ERROR: {error_msg}", True)
            return {"success": False, "error": error_msg}

    def show_screen(self, screen_name):
        self.screens[screen_name].lift()

    def run(self):
        self.root.mainloop()


def main():
    app = PinkyRobotGUI()
    app.run()


if __name__ == "__main__":
    main()
