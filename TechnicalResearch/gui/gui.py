import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import json
from datetime import datetime
import websocket


class RetroButton(tk.Button):
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
        hex_color = hex_color.lstrip("#")
        r, g, b = tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
        return f"#{int(r*0.9):02x}{int(g*0.9):02x}{int(b*0.9):02x}"


class ConsoleWidget(scrolledtext.ScrolledText):
    def __init__(self, parent):
        super().__init__(
            parent,
            height=8,
            font=("Courier New", 10),
            bg="#a8a8a8",  # 메인(#c0c0c0)보다 살짝 어두운 배경
            fg="#111111",  # 검정 글씨
            insertbackground="#111111",
            state="disabled",
            wrap="word",
            relief="solid",
            bd=4,  # 두꺼운 테두리
            highlightthickness=2,
            highlightbackground="#606060",
            highlightcolor="#606060",
        )
        self.log_count = 0

    def log(self, message, is_error=False):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.config(state="normal")
        self.insert("end", f"[{timestamp}] {message}\n")
        self.see("end")
        self.config(state="disabled")
        self.log_count += 1
        if self.log_count > 50:
            self.config(state="normal")
            self.delete("1.0", "2.0")
            self.config(state="disabled")


class NotificationPopup(tk.Toplevel):
    """결과 알림 팝업"""

    def __init__(self, parent, success, action_label):
        super().__init__(parent)
        self.overrideredirect(True)
        self.attributes("-topmost", True)

        bg = "#2e7d32" if success else "#c62828"
        text = f"{action_label} 완료" if success else f"{action_label} 실패"

        frame = tk.Frame(self, bg=bg, relief="solid", bd=2)
        frame.pack(fill="both", expand=True)

        tk.Label(
            frame,
            text=text,
            font=("Courier New", 14, "bold"),
            bg=bg,
            fg="white",
            padx=30,
            pady=20,
        ).pack()

        tk.Button(
            frame,
            text="확인",
            command=self.destroy,
            font=("Courier New", 11),
            bg="white",
            fg=bg,
            relief="flat",
            padx=20,
            pady=6,
            cursor="hand2",
        ).pack(pady=(0, 16))

        # 화면 중앙 배치
        self.update_idletasks()
        pw = parent.winfo_width()
        ph = parent.winfo_height()
        px = parent.winfo_rootx()
        py = parent.winfo_rooty()
        w = self.winfo_width()
        h = self.winfo_height()
        self.geometry(f"+{px + (pw - w) // 2}+{py + (ph - h) // 2}")

        self.grab_set()


class BaseScreen(tk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent, bg="#c0c0c0")
        self.app = app


class LoginScreen(BaseScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        center_frame = tk.Frame(self, bg="#c0c0c0")
        center_frame.place(relx=0.5, rely=0.4, anchor="center")

        self.status_label = tk.Label(
            center_frame,
            text="",
            font=("Courier New", 10),
            bg="#c0c0c0",
            fg="#666",
        )
        self.status_label.pack(pady=(0, 10))

        login_box = tk.Frame(center_frame, bg="#808080", relief="solid", bd=3)
        login_box.pack(padx=20, pady=20)

        tk.Label(
            login_box, text="Login", font=("Courier New", 16, "bold"), bg="#808080"
        ).pack(pady=(20, 10))

        tk.Label(login_box, text="I D :", font=("Courier New", 12), bg="#808080").pack(
            anchor="w", padx=20
        )

        self.user_id = tk.Entry(
            login_box, font=("Courier New", 12), width=30, relief="solid", bd=2
        )
        self.user_id.pack(padx=20, pady=(0, 10))

        tk.Label(login_box, text="Pw :", font=("Courier New", 12), bg="#808080").pack(
            anchor="w", padx=20
        )

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

        RetroButton(login_box, "로그인", command=self.login, bg="#4a90e2").pack(
            padx=20, pady=(10, 20), fill="x"
        )

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
                self.app.load_db_data()
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


class HomeScreen(BaseScreen):
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
            RetroButton(center_frame, text, command=command).pack(
                pady=10, ipadx=40, ipady=20
            )


class FetchScreen(BaseScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.selected_item_id = None
        self.selected_pos_id = None
        self.item_buttons = []
        self.room_buttons = []
        self._build_static_frame()

    def _build_static_frame(self):
        RetroButton(
            self, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        ).pack(anchor="w", padx=10, pady=10)
        tk.Label(
            self, text="물건 가져오기", font=("Courier New", 20, "bold"), bg="#c0c0c0"
        ).pack(pady=(0, 10))

        self.content_box = tk.Frame(self, bg="white", relief="solid", bd=3)
        self.content_box.pack(padx=20, pady=5, fill="both", expand=True)

        RetroButton(
            self, "물건 가져오기 실행", command=self.execute_fetch, bg="#4a90e2"
        ).pack(pady=20, ipadx=40, ipady=20)

    def on_show(self):
        for w in self.content_box.winfo_children():
            w.destroy()
        self.item_buttons = []
        self.room_buttons = []
        self.selected_item_id = None
        self.selected_pos_id = None

        tk.Label(
            self.content_box,
            text="Select item:",
            font=("Courier New", 12, "bold"),
            bg="white",
        ).pack(pady=(15, 5))
        item_frame = tk.Frame(self.content_box, bg="white")
        item_frame.pack(pady=5)
        for iid, iname in self.app.db_items:
            display = self.app.get_display_name("item", iid, iname)
            btn = RetroButton(
                item_frame,
                display,
                command=lambda i=iid, n=display: self.select_item(i, n),
            )
            btn.pack(side="left", padx=8, ipadx=12, ipady=12)
            self.item_buttons.append(btn)

        tk.Frame(self.content_box, bg="#cccccc", height=2).pack(
            fill="x", padx=20, pady=15
        )

        tk.Label(
            self.content_box,
            text="Select your location:",
            font=("Courier New", 12, "bold"),
            bg="white",
        ).pack(pady=(0, 5))
        room_frame = tk.Frame(self.content_box, bg="white")
        room_frame.pack(pady=5)
        for pid, pname in self.app.db_positions:
            display = self.app.get_display_name("position", pid, pname)
            btn = RetroButton(
                room_frame,
                display,
                command=lambda p=pid, n=display: self.select_room(p, n),
            )
            btn.pack(side="left", padx=8, ipadx=12, ipady=12)
            self.room_buttons.append(btn)

    def select_item(self, item_id, name):
        for btn in self.item_buttons:
            btn["bg"] = "#e8e8e8"
            btn["fg"] = "black"
        for btn in self.item_buttons:
            if btn["text"] == name:
                btn["bg"] = "#4caf50"
                btn["fg"] = "white"
        self.selected_item_id = item_id

    def select_room(self, pos_id, name):
        for btn in self.room_buttons:
            btn["bg"] = "#e8e8e8"
            btn["fg"] = "black"
        for btn in self.room_buttons:
            if btn["text"] == name:
                btn["bg"] = "#4a90e2"
                btn["fg"] = "white"
        self.selected_pos_id = pos_id

    def execute_fetch(self):
        if not self.selected_item_id:
            self.app.console.log("물건을 선택하세요", True)
            return
        if not self.selected_pos_id:
            self.app.console.log("위치를 선택하세요", True)
            return

        item_raw = next(
            (n for i, n in self.app.db_items if i == self.selected_item_id),
            self.selected_item_id,
        )
        pos_raw = next(
            (n for p, n in self.app.db_positions if p == self.selected_pos_id),
            self.selected_pos_id,
        )
        item_display = self.app.get_display_name(
            "item", self.selected_item_id, item_raw
        )
        pos_display = self.app.get_display_name(
            "position", self.selected_pos_id, pos_raw
        )

        self.app.send_fetch_request(
            item_id=self.selected_item_id,
            pos_id=self.selected_pos_id,
            item_display=item_display,
            pos_display=pos_display,
        )
        self.app.show_screen("home")


class TakeScreen(BaseScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.selected_pos_id = None
        self.room_buttons = []
        self._build_static_frame()

    def _build_static_frame(self):
        RetroButton(
            self, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        ).pack(anchor="w", padx=10, pady=10)
        tk.Label(
            self, text="물건 갖다놓기", font=("Courier New", 20, "bold"), bg="#c0c0c0"
        ).pack(pady=(0, 10))

        self.content_box = tk.Frame(self, bg="white", relief="solid", bd=3)
        self.content_box.pack(padx=20, pady=5, fill="both", expand=True)

        RetroButton(
            self, "물건 갖다놓기 실행", command=self.execute_take, bg="#ff9800"
        ).pack(pady=20, ipadx=40, ipady=20)

    def on_show(self):
        for w in self.content_box.winfo_children():
            w.destroy()
        self.room_buttons = []
        self.selected_pos_id = None

        tk.Label(
            self.content_box,
            text="Select your location:",
            font=("Courier New", 12, "bold"),
            bg="white",
        ).pack(pady=(30, 10))
        room_frame = tk.Frame(self.content_box, bg="white")
        room_frame.pack(pady=10)
        for pid, pname in self.app.db_positions:
            display = self.app.get_display_name("position", pid, pname)
            btn = RetroButton(
                room_frame,
                display,
                command=lambda p=pid, n=display: self.select_room(p, n),
            )
            btn.pack(side="left", padx=15, ipadx=20, ipady=20)
            self.room_buttons.append(btn)

    def select_room(self, pos_id, name):
        for btn in self.room_buttons:
            btn["bg"] = "#e8e8e8"
            btn["fg"] = "black"
        for btn in self.room_buttons:
            if btn["text"] == name:
                btn["bg"] = "#ff9800"
                btn["fg"] = "white"
        self.selected_pos_id = pos_id

    def execute_take(self):
        if not self.selected_pos_id:
            self.app.console.log("위치를 선택하세요", True)
            return

        result = self.app.send_message("take_req")
        if not result["success"]:
            self.app.console.log(f"요청 실패: {result.get('error', '')}", True)
            return

        result = self.app.send_message(
            "take_cmd", {"position_id": self.selected_pos_id}
        )
        if result["success"]:
            pos_raw = next(
                (n for p, n in self.app.db_positions if p == self.selected_pos_id),
                self.selected_pos_id,
            )
            pos_display = self.app.get_display_name(
                "position", self.selected_pos_id, pos_raw
            )
            self.app.console.log(f"take_cmd sent: pos={self.selected_pos_id}")
            self.app.add_log("갖다놓기", f"{pos_display}에 갖다놓기 요청")
            self.app.show_screen("home")
        else:
            self.app.console.log("명령 전송 실패", True)


class ScheduleListScreen(BaseScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        header = tk.Frame(self, bg="#c0c0c0")
        header.pack(fill="x", padx=10, pady=10)

        RetroButton(
            header, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        ).pack(side="left")
        tk.Label(
            header, text="   스케줄   ", font=("Courier New", 16, "bold"), bg="#c0c0c0"
        ).pack(side="left", padx=20)
        RetroButton(header, "새로고침", command=self.load_schedules, bg="#4a90e2").pack(
            side="right"
        )

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
        RetroButton(
            button_frame, "추가 (Add)", command=self.show_add_schedule, bg="#4caf50"
        ).pack(fill="x", expand=True)

    def load_schedules(self):
        result = self.app.send_message("schedule_req")

        for widget in self.schedule_container.winfo_children():
            widget.destroy()

        if result["success"]:
            schedules = result.get("data", {}).get("data", {}).get("schedules", [])
            if schedules:
                self.app.console.log(f"스케줄 키: {list(schedules[0].keys())}")

            if not schedules:
                tk.Label(
                    self.schedule_container,
                    text="등록된 스케줄이 없습니다",
                    font=("Courier New", 12),
                    fg="#666",
                    bg="white",
                ).pack(pady=50)
            else:
                for schedule in schedules:
                    self.create_schedule_item(schedule)
            self.app.console.log("스케줄을 불러왔습니다")
        else:
            tk.Label(
                self.schedule_container,
                text="스케줄 로드 실패",
                font=("Courier New", 12),
                fg="red",
                bg="white",
            ).pack(pady=50)
            self.app.console.log("스케줄 로드 실패", True)

    def create_schedule_item(self, schedule):
        item = tk.Frame(self.schedule_container, bg="#606060", relief="solid", bd=2)
        item.pack(fill="x", padx=10, pady=6)

        time_str = schedule.get("execute_time", "??:??")[:5]
        pos_id = schedule.get("position_id", "")
        item_id = schedule.get("item_id") or schedule.get("cmd_id") or ""
        cycle_val = schedule.get("cycle", "?")

        raw_pos = next(
            (pname for pid, pname in self.app.db_positions if pid == pos_id), pos_id
        )
        raw_item = next(
            (iname for iid, iname in self.app.db_items if iid == item_id), item_id
        )
        pos_display = self.app.get_display_name("position", pos_id, raw_pos)
        item_display = self.app.get_display_name("item", item_id, raw_item)

        cells = [
            ("시", time_str[:2]),
            ("분", time_str[3:5]),
            ("장소", pos_display),
            ("물건", item_display),
            ("주기", f"{cycle_val}일"),
        ]

        info_frame = tk.Frame(item, bg="#606060")
        info_frame.pack(side="left", fill="both", expand=True, padx=10, pady=8)

        for label, value in cells:
            cell = tk.Frame(info_frame, bg="#d8d8d8", relief="solid", bd=1)
            cell.pack(side="left", padx=5, pady=4, ipadx=8, ipady=6)
            tk.Label(
                cell, text=label, font=("Courier New", 8), bg="#d8d8d8", fg="#888888"
            ).pack()
            tk.Label(
                cell,
                text=value,
                font=("Courier New", 12, "bold"),
                bg="#d8d8d8",
                fg="#222222",
            ).pack(padx=6, pady=(0, 4))

        btn_frame = tk.Frame(item, bg="#606060")
        btn_frame.pack(side="right", padx=10, pady=8)
        RetroButton(
            btn_frame,
            "수정",
            command=lambda: self.edit_schedule(schedule.get("schedule_id")),
            bg="#ff9800",
        ).pack(pady=2)
        RetroButton(
            btn_frame,
            "삭제",
            command=lambda: self.delete_schedule(schedule.get("schedule_id")),
            bg="#f44336",
        ).pack(pady=2)

    def show_add_schedule(self):
        self.app.screens["schedule_edit"].editing_schedule_id = None
        self.app.screens["schedule_edit"].title_lbl["text"] = "스케줄 추가"
        self.app.show_screen("schedule_edit")

    def edit_schedule(self, schedule_id):
        self.app.screens["schedule_edit"].editing_schedule_id = schedule_id
        self.app.show_screen("schedule_edit")

    def delete_schedule(self, schedule_id):
        result = self.app.send_message(
            "schedule_edit", {"action": "delete", "schedule_id": schedule_id}
        )
        if result["success"]:
            self.app.console.log("스케줄이 삭제되었습니다")
            self.app.add_log("스케줄 삭제", f"스케줄 ID: {schedule_id} 삭제됨")
            self.load_schedules()
        else:
            self.app.console.log("스케줄 삭제 실패", True)


class DrumRollColumn(tk.Frame):
    ITEM_H = 72
    VISIBLE = 3
    ANIM_STEPS = 8
    ANIM_MS = 12

    def __init__(self, parent, items, width=110, **kwargs):
        canvas_h = self.ITEM_H * self.VISIBLE
        super().__init__(parent, bg="#c0c0c0", width=width, height=canvas_h, **kwargs)
        self.pack_propagate(False)

        self.items = items
        self.col_width = width
        self._index = 0
        self._anim_job = None
        self._drag_start_y = 0
        self._drag_start_idx = 0

        self.canvas = tk.Canvas(
            self, width=width, height=canvas_h, bg="#d8d8d8", highlightthickness=0
        )
        self.canvas.pack(fill="both", expand=True)

        mid_y0, mid_y1 = self.ITEM_H, self.ITEM_H * 2
        self.canvas.create_rectangle(
            3,
            mid_y0,
            width - 3,
            mid_y1,
            outline="#808080",
            width=2,
            fill="#ffffff",
            tags="highlight",
        )
        self.canvas.create_rectangle(
            0,
            0,
            width,
            self.ITEM_H,
            fill="#c0c0c0",
            stipple="gray50",
            outline="",
            tags="fade",
        )
        self.canvas.create_rectangle(
            0,
            self.ITEM_H * 2,
            width,
            canvas_h,
            fill="#c0c0c0",
            stipple="gray50",
            outline="",
            tags="fade",
        )

        self._draw(0)

        for widget in (self, self.canvas):
            widget.bind("<MouseWheel>", self._on_wheel)
            widget.bind("<Button-4>", lambda e: self._step(-1))
            widget.bind("<Button-5>", lambda e: self._step(1))
            widget.bind("<ButtonPress-1>", self._drag_start)
            widget.bind("<B1-Motion>", self._drag_move)
            widget.bind("<ButtonRelease-1>", self._drag_end)

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
                fill="#111111" if is_center else "#999999",
                tags="label",
            )
        self.canvas.tag_raise("highlight")
        self.canvas.tag_raise("label")
        self.canvas.tag_raise("fade")

    def _step(self, direction, extra_callback=None):
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
            self.ANIM_MS, lambda: self._animate(start_offset, step + 1, callback)
        )

    def _on_wheel(self, event):
        self._step(-1 if event.delta > 0 else 1)

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

    def get_value(self):
        return self.items[self._index]

    def get_index(self):
        return self._index

    def set_index(self, idx):
        self._index = idx % len(self.items)
        self._draw(0)


class ScheduleEditScreen(BaseScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.configure(bg="#c0c0c0")
        self.editing_schedule_id = None
        self.hours = [str(h).zfill(2) for h in range(24)]
        self.minutes = [str(m).zfill(2) for m in range(0, 60, 5)]
        self.cycles = [f"{i}일" for i in range(1, 8)]
        self.hour_drum = self.minute_drum = None
        self.location_drum = self.item_drum = self.cycle_drum = None
        self._positions = []
        self._items = []
        self._build_skeleton()
        self._start_preview_loop()

    def _build_skeleton(self):
        header = tk.Frame(self, bg="#c0c0c0")
        header.pack(fill="x", padx=15, pady=(12, 4))
        tk.Button(
            header,
            text="◀ 뒤로가기",
            command=self.cancel_edit,
            font=("Courier New", 11, "bold"),
            bg="#a0a0a0",
            fg="#000000",
            relief="solid",
            bd=2,
            padx=12,
            pady=7,
            cursor="hand2",
        ).pack(side="left")
        self.title_lbl = tk.Label(
            header,
            text="스케줄 추가",
            font=("Courier New", 17, "bold"),
            bg="#c0c0c0",
            fg="#000000",
        )
        self.title_lbl.pack(side="left", padx=16)

        preview_bar = tk.Frame(self, bg="#808080", relief="solid", bd=2)
        preview_bar.pack(fill="x", padx=15, pady=(0, 8))
        self.preview_lbl = tk.Label(
            preview_bar,
            text="--:--  |  --  |  --  |  --",
            font=("Courier New", 12, "bold"),
            bg="#808080",
            fg="#ffffff",
            pady=8,
        )
        self.preview_lbl.pack()

        self.drum_container = tk.Frame(self, bg="#c0c0c0")
        self.drum_container.pack(pady=4, fill="x")

        btn_row = tk.Frame(self, bg="#c0c0c0")
        btn_row.pack(pady=16, padx=40, fill="x")
        tk.Button(
            btn_row,
            text="저장",
            command=self.save_schedule,
            font=("Courier New", 13, "bold"),
            bg="#606060",
            fg="white",
            relief="solid",
            bd=3,
            padx=20,
            pady=12,
            cursor="hand2",
        ).pack(side="left", fill="x", expand=True, padx=(0, 5))
        tk.Button(
            btn_row,
            text="취소",
            command=self.cancel_edit,
            font=("Courier New", 13, "bold"),
            bg="#a0a0a0",
            fg="#000000",
            relief="solid",
            bd=3,
            padx=20,
            pady=12,
            cursor="hand2",
        ).pack(side="right", fill="x", expand=True, padx=(5, 0))

    def on_show(self):
        for w in self.drum_container.winfo_children():
            w.destroy()

        self._positions = self.app.db_positions
        self._items = self.app.db_items

        loc_names = [
            self.app.get_display_name("position", pid, pname)
            for pid, pname in self._positions
        ]
        item_names = [
            self.app.get_display_name("item", iid, iname) for iid, iname in self._items
        ]

        col_specs = [
            ("시", self.hours, 80),
            ("분", self.minutes, 80),
            ("장소", loc_names, 120),
            ("물건", item_names, 120),
            ("주기", self.cycles, 100),
        ]
        labels_row = tk.Frame(self.drum_container, bg="#c0c0c0")
        labels_row.pack(pady=(4, 0))
        for lbl_text, _, w in col_specs:
            cell = tk.Frame(labels_row, bg="#c0c0c0", width=w)
            cell.pack(side="left", padx=4)
            cell.pack_propagate(False)
            tk.Label(
                cell,
                text=lbl_text,
                font=("Courier New", 10, "bold"),
                bg="#c0c0c0",
                fg="#444444",
            ).pack()

        drums_row = tk.Frame(self.drum_container, bg="#c0c0c0")
        drums_row.pack(pady=4)
        self.hour_drum = DrumRollColumn(drums_row, self.hours, width=80)
        self.minute_drum = DrumRollColumn(drums_row, self.minutes, width=80)
        self.location_drum = DrumRollColumn(drums_row, loc_names, width=120)
        self.item_drum = DrumRollColumn(drums_row, item_names, width=120)
        self.cycle_drum = DrumRollColumn(drums_row, self.cycles, width=100)
        for drum in (
            self.hour_drum,
            self.minute_drum,
            self.location_drum,
            self.item_drum,
            self.cycle_drum,
        ):
            drum.pack(side="left", padx=4)
        self.hour_drum.set_index(9)

    def _start_preview_loop(self):
        self._update_preview()

    def _update_preview(self):
        try:
            h = self.hour_drum.get_value()
            m = self.minute_drum.get_value()
            lo = self.location_drum.get_value()
            it = self.item_drum.get_value()
            cy = self.cycle_drum.get_value()
            self.preview_lbl.config(text=f"{h}:{m}  |  {lo}  |  {it}  |  {cy}")
        except Exception:
            pass
        self.after(150, self._update_preview)

    def save_schedule(self):
        if not self.hour_drum:
            self.app.console.log("드럼롤이 아직 로드되지 않았습니다", True)
            return

        hour = self.hour_drum.get_value()
        minute = self.minute_drum.get_value()
        execute_time = f"{hour}:{minute}:00"

        loc_display = self.location_drum.get_value()
        item_display = self.item_drum.get_value()

        position_id = next(
            pid
            for pid, pname in self._positions
            if self.app.get_display_name("position", pid, pname) == loc_display
        )
        item_id = next(
            iid
            for iid, iname in self._items
            if self.app.get_display_name("item", iid, iname) == item_display
        )
        cycle_n = self.cycle_drum.get_index() + 1

        if self.editing_schedule_id:
            schedule_id = self.editing_schedule_id
        else:
            date_str = datetime.now().strftime("%y%m%d")
            seq = int(datetime.now().strftime("%H%M%S")) % 9999 + 1
            schedule_id = f"s{date_str}{seq:04d}"

        result = self.app.send_message(
            "schedule_edit",
            {
                "action": "edit" if self.editing_schedule_id else "add",
                "schedule_id": schedule_id,
                "cmd_id": "c2602070001",
                "item_id": item_id,
                "position_id": position_id,
                "execute_time": execute_time,
                "cycle": cycle_n,
                "on_weekends": False,
            },
        )

        server_success = result.get("data", {}).get("data", {}).get("success", False)
        if result["success"] and server_success:
            is_edit = bool(self.editing_schedule_id)
            msg = "스케줄이 수정되었습니다" if is_edit else "스케줄이 추가되었습니다"
            action_label = "스케줄 수정" if is_edit else "스케줄 추가"
            detail = f"{execute_time[:5]}  |  {loc_display}  |  {item_display}  |  {cycle_n}일마다"
            self.app.console.log(msg)
            self.app.add_log(action_label, detail)
            self.app.show_screen("schedule_list")
            self.app.screens["schedule_list"].load_schedules()
        else:
            self.app.console.log("스케줄 저장 실패", True)

    def cancel_edit(self):
        self.app.show_screen("schedule_list")


class HistoryScreen(BaseScreen):
    def __init__(self, parent, app):
        super().__init__(parent, app)
        self.setup_ui()

    def setup_ui(self):
        header = tk.Frame(self, bg="#c0c0c0")
        header.pack(fill="x", padx=10, pady=10)

        RetroButton(
            header, "◀ 뒤로가기", command=lambda: self.app.show_screen("home")
        ).pack(side="left")
        tk.Label(
            header, text="기록 관리", font=("Courier New", 16, "bold"), bg="#c0c0c0"
        ).pack(side="left", padx=20)
        RetroButton(header, "새로고침", command=self.load_history, bg="#4a90e2").pack(
            side="right"
        )
        RetroButton(
            header, "기록 지우기", command=self.clear_history, bg="#f44336"
        ).pack(side="right", padx=(0, 8))

        list_frame = tk.Frame(self, bg="white", relief="solid", bd=3)
        list_frame.pack(fill="both", expand=True, padx=20, pady=10)

        self.log_canvas = tk.Canvas(list_frame, bg="white", highlightthickness=0)
        scrollbar = tk.Scrollbar(
            list_frame, orient="vertical", command=self.log_canvas.yview
        )
        self.log_container = tk.Frame(self.log_canvas, bg="white")
        self.log_canvas.create_window((0, 0), window=self.log_container, anchor="nw")
        self.log_canvas.configure(yscrollcommand=scrollbar.set)
        self.log_container.bind(
            "<Configure>",
            lambda e: self.log_canvas.configure(
                scrollregion=self.log_canvas.bbox("all")
            ),
        )
        self.log_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def on_show(self):
        self.load_history()

    def load_history(self):
        for w in self.log_container.winfo_children():
            w.destroy()

        logs = self.app.activity_log
        if not logs:
            tk.Label(
                self.log_container,
                text="기록이 없습니다",
                font=("Courier New", 12),
                fg="#999",
                bg="white",
            ).pack(pady=40)
            return

        for entry in reversed(logs):
            self._create_log_item(entry)

    def _create_log_item(self, entry):
        color_map = {
            "가져오기": "#4a90e2",
            "가져오기 완료": "#2e7d32",
            "가져오기 실패": "#c62828",
            "갖다놓기": "#ff9800",
            "스케줄 추가": "#4caf50",
            "스케줄 수정": "#9c27b0",
            "스케줄 삭제": "#f44336",
            "서버 연결": "#607d8b",
        }
        action = entry.get("action", "기타")
        bar_color = color_map.get(action, "#808080")

        row = tk.Frame(self.log_container, bg="white")
        row.pack(fill="x", padx=8, pady=3)
        tk.Frame(row, bg=bar_color, width=6).pack(side="left", fill="y")

        content = tk.Frame(row, bg="#f5f5f5", relief="flat")
        content.pack(side="left", fill="x", expand=True, padx=(6, 0), ipady=6)

        top_row = tk.Frame(content, bg="#f5f5f5")
        top_row.pack(fill="x", padx=10, pady=(6, 2))
        tk.Label(
            top_row,
            text=action,
            font=("Courier New", 11, "bold"),
            bg="#f5f5f5",
            fg=bar_color,
        ).pack(side="left")
        tk.Label(
            top_row,
            text=entry.get("time", ""),
            font=("Courier New", 9),
            bg="#f5f5f5",
            fg="#aaaaaa",
        ).pack(side="right")
        tk.Label(
            content,
            text=entry.get("detail", ""),
            font=("Courier New", 11),
            bg="#f5f5f5",
            fg="#333333",
            anchor="w",
        ).pack(fill="x", padx=10, pady=(0, 6))
        tk.Frame(self.log_container, bg="#eeeeee", height=1).pack(fill="x", padx=8)

    def clear_history(self):
        if messagebox.askyesno("확인", "모든 기록을 삭제할까요?"):
            self.app.activity_log = []
            self.app._save_activity_log()
            self.load_history()
            self.app.console.log("기록이 삭제되었습니다")


class PinkyRobotGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Pinky Robot Control")
        self.root.geometry("900x700")
        self.root.configure(bg="#c0c0c0")

        self.SERVER_URL = "ws://192.168.0.48:8000/gui"
        self.current_user = None

        self.db_items = []
        self.db_positions = []

        import os

        self.log_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "pinky_activity_log.json"
        )
        self.activity_log = self._load_activity_log()

        self.friendly_names = {
            "choco": "초코",
            "juice": "주스",
            "medicine": "약",
            "living room": "거실",
            "bed room": "안방",
            "kitchen": "안방",
            "bathroom": "안방",
        }

        self.db_positions = [
            ("p2602150003", "living room"),
            ("p2602150004", "bed room"),
        ]

        self.setup_menu()
        self.setup_ui()
        self.root.after(300, self.auto_connect)

    def get_display_name(self, kind, id_val, raw_name):
        return self.friendly_names.get(raw_name, raw_name)

    def _load_activity_log(self):
        try:
            with open(self.log_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return []

    def _save_activity_log(self):
        try:
            with open(self.log_file, "w", encoding="utf-8") as f:
                json.dump(self.activity_log, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.console.log(f"로그 저장 실패: {e}", True)

    def add_log(self, action, detail=""):
        entry = {
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "action": action,
            "detail": detail,
        }
        self.activity_log.append(entry)
        if len(self.activity_log) > 500:
            self.activity_log = self.activity_log[-500:]
        self._save_activity_log()

    def send_fetch_request(self, item_id, pos_id, item_display, pos_display):
        self.add_log("가져오기 요청", f"{pos_display}에서 {item_display} 가져오기 요청")
        self.console.log(f"fetch_cmd 전송: item={item_id}, pos={pos_id}")

        import threading

        def run():
            result = self.send_message(
                "fetch_cmd",
                {"item_id": item_id, "position_id": pos_id},
                timeout=300,
            )
            success = result.get("data", {}).get("data", {}).get("success", False)
            action = "가져오기 완료" if success else "가져오기 실패"
            self.add_log(action, f"{pos_display}에서 {item_display}")
            self.root.after(
                0, lambda: NotificationPopup(self.root, success, "물건 가져오기")
            )

        threading.Thread(target=run, daemon=True).start()

    def auto_connect(self):
        result = self.send_message("connect")
        if result["success"]:
            self.console.log("Server Connect")
            self.add_log("서버 연결", f"성공 - {self.SERVER_URL}")
        else:
            self.console.log(f"서버 자동 연결 실패: {result.get('error', '')}", True)
            self.add_log("서버 연결", f"실패 - {self.SERVER_URL}")

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

    def send_message(self, msg, data=None, timeout=5):
        if data is None:
            data = {}
        payload = {"msg": msg, "data": data}
        try:
            ws = websocket.create_connection(self.SERVER_URL, timeout=timeout)
            ws.send(json.dumps(payload))
            response = ws.recv()
            ws.close()
            result = json.loads(response)
            return {"success": True, "data": result}
        except Exception as e:
            self.console.log(f"ERROR: {e}", True)
            return {"success": False, "error": str(e)}

    def load_db_data(self):
        result = self.send_message("fetch_req")
        if result["success"]:
            d = result.get("data", {}).get("data", {})
            self.db_items = [(i["item_id"], i["item_name"]) for i in d.get("items", [])]
            self.console.log(
                f"DB 로드: items={len(self.db_items)}, positions={len(self.db_positions)}"
            )
        else:
            self.console.log("DB 로드 실패 - 기본값 사용", True)
            self.db_items = [
                ("i2602150001", "choco"),
                ("i2602150002", "juice"),
                ("i2602150003", "medicine"),
            ]

    def show_screen(self, screen_name):
        screen = self.screens[screen_name]
        screen.lift()
        if hasattr(screen, "on_show"):
            screen.on_show()

    def run(self):
        self.root.mainloop()


def main():
    app = PinkyRobotGUI()
    app.run()


if __name__ == "__main__":
    main()
