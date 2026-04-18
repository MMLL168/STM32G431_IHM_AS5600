from __future__ import annotations

import math
import queue
import threading
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import messagebox, ttk

from follow_tuning_tool import (
    AspepMcpClient,
    FollowTuningProtocolError,
    TEST_MODE_NAMES,
    TEST_MODE_VALUES,
    format_snapshot,
    list_preset_paths,
    list_serial_ports,
    load_json,
    load_profile,
    serial,
)


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_PRESET_NAME = "05_normal_follow"
POLL_INTERVAL_MS = 350
VISUAL_HISTORY_LIMIT = 90
MOTION_CANVAS_MIN_HEIGHT = 340

VISUAL_BG_TOP = "#07131c"
VISUAL_BG_BOTTOM = "#163645"
VISUAL_PANEL = "#102530"
VISUAL_GRID = "#2a4f61"
KNOB_COLOR = "#ffb066"
TARGET_COLOR = "#ffd166"
COMMAND_COLOR = "#ff6b6b"
ACTUAL_COLOR = "#54d2d2"
SPEED_COLOR = "#74c0fc"
ZONE_COLORS = {
    0: "#4cc9f0",
    1: "#ffd166",
    2: "#ff6b6b",
}

PRESET_HINTS = {
    "00_direction_probe": "Probe sensored direction and electrical trim.",
    "01_fixed_target_hold": "Lock the current position and verify static holding.",
    "02_fixed_target_offset_150": "Lock near the current position with +15 degree offset.",
    "03_step_small_120": "Run a small step response around the captured center angle.",
    "04_step_medium_360": "Run a medium step response to inspect larger braking behavior.",
    "05_normal_follow": "Normal knob-following mode for application use.",
}

SCENARIOS = {
    "Normal Follow": {"preset": "05_normal_follow"},
    "Hold Here": {"test_mode": "fixed_target", "fixed_target_offset_deg10": 0},
    "Hold +15 deg": {"test_mode": "fixed_target", "fixed_target_offset_deg10": 150},
    "Step 12 deg": {"test_mode": "step_sequence", "step_amplitude_deg10": 120},
    "Step 36 deg": {"test_mode": "step_sequence", "step_amplitude_deg10": 360},
    "Direction Probe": {"preset": "00_direction_probe"},
}

QUICK_FIELDS = [
    ("position_current_ma_num", "Position Gain"),
    ("damping_current_ma_per_rpm", "Damping"),
    ("settle_min_current_ma", "Settle Floor"),
    ("max_current_ma", "Max Current"),
    ("hold_exit_deg10", "Hold Exit"),
    ("target_angle_slew_max_deg10_per_tick", "Slew Max"),
]

CARD_SPECS = {
    "Tracking": [
        ("Mode", "mode"),
        ("Control Zone", "zone"),
        ("Controller", "controller"),
        ("Hold", "hold"),
        ("Source", "source"),
        ("Follower", "follower"),
    ],
    "Angles": [
        ("Knob", "knob"),
        ("Command", "command"),
        ("Target", "target"),
        ("Actual", "actual"),
        ("Error", "error"),
        ("Probe/Settle", "phase"),
    ],
    "Motion": [
        ("Target RPM", "target_rpm"),
        ("Measured RPM", "speed"),
        ("Current Cmd", "current"),
        ("Active", "active"),
        ("Knob Online", "knob_online"),
        ("Sensored", "sensored"),
    ],
    "Health": [
        ("Follower Status", "follower_status"),
        ("Follower Magnet", "magnet"),
        ("Direction Sign", "direction"),
        ("Elec Trim", "trim"),
        ("Protocol", "protocol"),
        ("Board Snapshot", "board_mode"),
    ],
}


def format_deg10(value: int) -> str:
    return f"{value / 10.0:.1f} deg"


def format_signed_deg10(value: int) -> str:
    return f"{value / 10.0:+.1f} deg"


def on_off(flag: int, on_text: str = "Online", off_text: str = "Offline") -> str:
    return on_text if int(flag) != 0 else off_text


def active_idle(flag: int) -> str:
    return "Active" if int(flag) != 0 else "Idle"


def hold_state(flag: int) -> str:
    return "Holding" if int(flag) != 0 else "Tracking"


def follower_magnet_text(snapshot: dict) -> str:
    if int(snapshot["follower_magnet_detected"]) == 0:
        return "No magnet"
    if int(snapshot["follower_magnet_too_weak"]) != 0:
        return "Too weak"
    if int(snapshot["follower_magnet_too_strong"]) != 0:
        return "Too strong"
    return "OK"


def zone_text(snapshot: dict) -> str:
    if int(snapshot["test_mode"]) == TEST_MODE_VALUES["direction_probe"]:
        return f"Probe phase {snapshot['settle_active']}"

    state = int(snapshot["settle_active"])
    if state == 2:
        return "Urgent brake"
    if state == 1:
        return "Settle zone"
    return "Tracking"


def phase_text(snapshot: dict) -> str:
    if int(snapshot["test_mode"]) == TEST_MODE_VALUES["direction_probe"]:
        return str(snapshot["settle_active"])
    return zone_text(snapshot)


def board_mode_text(snapshot: dict) -> str:
    return TEST_MODE_NAMES.get(int(snapshot["test_mode"]), str(snapshot["test_mode"]))


def format_profile_summary(profile: dict) -> str:
    lines = [
        f"Mode: {profile['test_mode']}",
        f"Position gain: {profile['position_current_ma_num']} / {profile['position_current_ma_den']}",
        f"Damping: {profile['damping_current_ma_per_rpm']} mA/rpm",
        f"Hold enter/exit: {profile['hold_enter_deg10']} / {profile['hold_exit_deg10']} deg10",
        f"Current window: {profile['settle_min_current_ma']} .. {profile['settle_max_current_ma']} / {profile['max_current_ma']} mA",
        f"Slew max: {profile['target_angle_slew_max_deg10_per_tick']} deg10/tick",
        f"Breakaway: {profile['breakaway_current_ma']} mA",
        f"Sensored: dir {profile['user_sensored_direction_sign']}, trim {profile['user_sensored_elec_trim_deg10']} deg10",
    ]
    return "\n".join(lines)


class FollowControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Follow Control App")
        self.geometry("1280x900")
        self.minsize(960, 680)
        self.configure(bg="#efe7da")

        self.event_queue: queue.Queue = queue.Queue()
        self.live_operation_lock = threading.Lock()
        self.live_client: AspepMcpClient | None = None
        self.live_poll_enabled = False
        self.live_poll_inflight = False
        self.preset_files: dict[str, Path] = {}
        self.port_map: dict[str, str] = {}
        self.runtime_profile = load_profile()
        self.latest_snapshot: dict | None = None
        self.connection_info: dict | None = None
        self.error_history_deg10 = deque(maxlen=VISUAL_HISTORY_LIMIT)
        self.current_history_ma = deque(maxlen=VISUAL_HISTORY_LIMIT)
        self.speed_history_rpm = deque(maxlen=VISUAL_HISTORY_LIMIT)

        self.port_var = tk.StringVar(value="")
        self.preset_var = tk.StringVar(value=DEFAULT_PRESET_NAME)
        self.status_var = tk.StringVar(value="Ready")
        self.quick_vars = {key: tk.StringVar(value="") for key, _label in QUICK_FIELDS}
        self.metric_vars = {
            "mode": tk.StringVar(value="-"),
            "zone": tk.StringVar(value="-"),
            "controller": tk.StringVar(value="-"),
            "hold": tk.StringVar(value="-"),
            "source": tk.StringVar(value="-"),
            "follower": tk.StringVar(value="-"),
            "knob": tk.StringVar(value="-"),
            "command": tk.StringVar(value="-"),
            "target": tk.StringVar(value="-"),
            "actual": tk.StringVar(value="-"),
            "error": tk.StringVar(value="-"),
            "phase": tk.StringVar(value="-"),
            "target_rpm": tk.StringVar(value="-"),
            "speed": tk.StringVar(value="-"),
            "current": tk.StringVar(value="-"),
            "active": tk.StringVar(value="-"),
            "knob_online": tk.StringVar(value="-"),
            "sensored": tk.StringVar(value="-"),
            "follower_status": tk.StringVar(value="-"),
            "magnet": tk.StringVar(value="-"),
            "direction": tk.StringVar(value="-"),
            "trim": tk.StringVar(value="-"),
            "protocol": tk.StringVar(value="-"),
            "board_mode": tk.StringVar(value="-"),
        }

        self._configure_styles()
        self._build_ui()
        self._refresh_preset_list()
        self._refresh_port_list()
        self._sync_quick_fields(self.runtime_profile)
        self._update_profile_summary(self.runtime_profile)
        self._reset_dashboard()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(100, self._poll_event_queue)

    def _configure_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("App.TFrame", background="#efe7da")
        style.configure("Card.TLabelframe", background="#fbf8f2", borderwidth=1, relief="solid")
        style.configure("Card.TLabelframe.Label", background="#fbf8f2", foreground="#27404b", font=("Segoe UI Semibold", 11))
        style.configure("Section.TLabelframe", background="#f6f0e5", borderwidth=1, relief="solid")
        style.configure("Section.TLabelframe.Label", background="#f6f0e5", foreground="#27404b", font=("Segoe UI Semibold", 11))
        style.configure("Accent.TButton", font=("Segoe UI Semibold", 10))
        style.configure("MetricName.TLabel", background="#fbf8f2", foreground="#6a645d", font=("Segoe UI", 10))
        style.configure("MetricValue.TLabel", background="#fbf8f2", foreground="#1b2830", font=("Consolas", 11, "bold"))
        style.configure("HeroValue.TLabel", background="#fbf8f2", foreground="#8f2d2d", font=("Segoe UI Semibold", 20))
        style.configure("Header.TLabel", background="#17313b", foreground="#f8f5ef", font=("Segoe UI Semibold", 22))
        style.configure("SubHeader.TLabel", background="#17313b", foreground="#d9d0c2", font=("Segoe UI", 10))

    def _build_ui(self):
        header = tk.Frame(self, bg="#17313b", padx=18, pady=14)
        header.pack(fill=tk.X)
        ttk.Label(header, text="Follow Control App", style="Header.TLabel").pack(anchor=tk.W)
        ttk.Label(
            header,
            text="MCP/ASPEP runtime console for follow-control operation, live status, and scenario switching.",
            style="SubHeader.TLabel",
        ).pack(anchor=tk.W, pady=(4, 0))

        body = ttk.Frame(self, style="App.TFrame", padding=14)
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        dashboard = ttk.Frame(body, style="App.TFrame")
        dashboard.grid(row=0, column=0, rowspan=2, sticky=tk.NSEW, padx=(0, 12))
        dashboard.columnconfigure(0, weight=1)
        dashboard.columnconfigure(1, weight=1)
        dashboard.rowconfigure(0, weight=0)
        dashboard.rowconfigure(1, weight=3, minsize=MOTION_CANVAS_MIN_HEIGHT)
        dashboard.rowconfigure(2, weight=1)
        dashboard.rowconfigure(3, weight=1)

        hero = ttk.LabelFrame(dashboard, text="Live Summary", style="Card.TLabelframe", padding=14)
        hero.grid(row=0, column=0, columnspan=2, sticky=tk.EW, pady=(0, 12))
        hero.columnconfigure(0, weight=1)
        hero.columnconfigure(1, weight=1)
        hero.columnconfigure(2, weight=1)
        hero.columnconfigure(3, weight=1)
        self._build_hero_metric(hero, 0, "Error", self.metric_vars["error"])
        self._build_hero_metric(hero, 1, "Speed", self.metric_vars["speed"])
        self._build_hero_metric(hero, 2, "Current", self.metric_vars["current"])
        self._build_hero_metric(hero, 3, "Zone", self.metric_vars["zone"])

        visual_frame = ttk.LabelFrame(dashboard, text="Motion Studio", style="Card.TLabelframe", padding=10)
        visual_frame.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW, pady=(0, 12))
        visual_frame.rowconfigure(0, weight=1)
        visual_frame.columnconfigure(0, weight=1)
        self.motion_canvas = tk.Canvas(
            visual_frame,
            height=MOTION_CANVAS_MIN_HEIGHT,
            highlightthickness=0,
            bg=VISUAL_BG_BOTTOM,
        )
        self.motion_canvas.grid(row=0, column=0, sticky=tk.NSEW)
        self.motion_canvas.bind("<Configure>", self._on_motion_canvas_configure)

        card_titles = list(CARD_SPECS.keys())
        for index, title in enumerate(card_titles):
            row = 2 + (index // 2)
            column = index % 2
            card = ttk.LabelFrame(dashboard, text=title, style="Card.TLabelframe", padding=14)
            card.grid(row=row, column=column, sticky=tk.NSEW, padx=(0, 12) if column == 0 else 0, pady=(0, 12))
            card.columnconfigure(1, weight=1)
            for line, (label_text, key) in enumerate(CARD_SPECS[title]):
                ttk.Label(card, text=label_text, style="MetricName.TLabel").grid(
                    row=line, column=0, sticky=tk.W, padx=(0, 10), pady=4
                )
                ttk.Label(card, textvariable=self.metric_vars[key], style="MetricValue.TLabel").grid(
                    row=line, column=1, sticky=tk.E, pady=4
                )

        control_panel = ttk.Frame(body, style="App.TFrame")
        control_panel.grid(row=0, column=1, sticky=tk.NSEW)
        control_panel.columnconfigure(0, weight=1)

        connection = ttk.LabelFrame(control_panel, text="Connection", style="Section.TLabelframe", padding=12)
        connection.grid(row=0, column=0, sticky=tk.EW)
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="COM Port").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        self.port_combo = ttk.Combobox(connection, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.grid(row=0, column=1, sticky=tk.EW, pady=4)
        ttk.Button(connection, text="Refresh", command=self._refresh_port_list).grid(row=0, column=2, sticky=tk.W, padx=(8, 0), pady=4)
        ttk.Button(connection, text="Connect", style="Accent.TButton", command=self.connect_live).grid(
            row=1, column=0, sticky=tk.EW, pady=(8, 4)
        )
        ttk.Button(connection, text="Disconnect", command=self.disconnect_live).grid(row=1, column=1, sticky=tk.EW, pady=(8, 4))
        ttk.Button(connection, text="Read Board", command=self.read_board_state).grid(row=1, column=2, sticky=tk.EW, padx=(8, 0), pady=(8, 4))

        scenarios = ttk.LabelFrame(control_panel, text="Scenarios", style="Section.TLabelframe", padding=12)
        scenarios.grid(row=1, column=0, sticky=tk.EW, pady=(12, 0))
        scenarios.columnconfigure(0, weight=1)
        scenarios.columnconfigure(1, weight=1)
        for index, name in enumerate(SCENARIOS):
            ttk.Button(
                scenarios,
                text=name,
                command=lambda scenario_name=name: self.apply_scenario(scenario_name),
            ).grid(row=index // 2, column=index % 2, sticky=tk.EW, padx=(0, 8) if index % 2 == 0 else 0, pady=4)

        presets = ttk.LabelFrame(control_panel, text="Presets", style="Section.TLabelframe", padding=12)
        presets.grid(row=2, column=0, sticky=tk.EW, pady=(12, 0))
        presets.columnconfigure(0, weight=1)
        self.preset_combo = ttk.Combobox(presets, textvariable=self.preset_var, state="readonly")
        self.preset_combo.grid(row=0, column=0, sticky=tk.EW, pady=4)
        ttk.Button(presets, text="Preview", command=self.preview_selected_preset).grid(row=0, column=1, sticky=tk.EW, padx=(8, 0), pady=4)
        ttk.Button(presets, text="Apply Preset", style="Accent.TButton", command=self.apply_selected_preset).grid(
            row=1, column=0, columnspan=2, sticky=tk.EW, pady=4
        )
        self.preset_hint_label = ttk.Label(presets, text="", wraplength=320, justify=tk.LEFT)
        self.preset_hint_label.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(4, 0))

        tuning = ttk.LabelFrame(control_panel, text="Response Tuning", style="Section.TLabelframe", padding=12)
        tuning.grid(row=3, column=0, sticky=tk.EW, pady=(12, 0))
        tuning.columnconfigure(1, weight=1)
        for row, (key, label) in enumerate(QUICK_FIELDS):
            ttk.Label(tuning, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=4)
            ttk.Entry(tuning, textvariable=self.quick_vars[key], width=18).grid(row=row, column=1, sticky=tk.EW, pady=4)
        ttk.Button(tuning, text="Apply Quick Tuning", style="Accent.TButton", command=self.apply_quick_tuning).grid(
            row=len(QUICK_FIELDS), column=0, columnspan=2, sticky=tk.EW, pady=(10, 0)
        )

        actions = ttk.LabelFrame(control_panel, text="Board Actions", style="Section.TLabelframe", padding=12)
        actions.grid(row=4, column=0, sticky=tk.EW, pady=(12, 0))
        actions.columnconfigure(0, weight=1)
        ttk.Button(actions, text="Reset Runtime Profile", command=self.reset_runtime_profile).grid(row=0, column=0, sticky=tk.EW, pady=4)

        profile_frame = ttk.LabelFrame(body, text="Active Runtime Profile", style="Section.TLabelframe", padding=12)
        profile_frame.grid(row=1, column=1, sticky=tk.NSEW, pady=(12, 0))
        profile_frame.rowconfigure(0, weight=1)
        profile_frame.columnconfigure(0, weight=1)
        self.profile_text = tk.Text(profile_frame, height=11, wrap=tk.WORD, font=("Consolas", 10))
        self.profile_text.grid(row=0, column=0, sticky=tk.NSEW)
        self.profile_text.configure(state=tk.DISABLED)

        bottom = ttk.Frame(self, style="App.TFrame", padding=(14, 0, 14, 14))
        bottom.pack(fill=tk.BOTH, expand=False)
        bottom.columnconfigure(0, weight=1)
        bottom.columnconfigure(1, weight=1)

        snapshot_frame = ttk.LabelFrame(bottom, text="Live Snapshot", style="Section.TLabelframe", padding=10)
        snapshot_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 12))
        snapshot_frame.rowconfigure(0, weight=1)
        snapshot_frame.columnconfigure(0, weight=1)
        self.snapshot_text = tk.Text(snapshot_frame, height=14, wrap=tk.WORD, font=("Consolas", 10))
        self.snapshot_text.grid(row=0, column=0, sticky=tk.NSEW)
        self.snapshot_text.configure(state=tk.DISABLED)

        log_frame = ttk.LabelFrame(bottom, text="Event Log", style="Section.TLabelframe", padding=10)
        log_frame.grid(row=0, column=1, sticky=tk.NSEW)
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=14, wrap=tk.WORD, font=("Consolas", 10))
        self.log_text.grid(row=0, column=0, sticky=tk.NSEW)

        status = ttk.Label(self, textvariable=self.status_var, anchor=tk.W, background="#efe7da", padding=(14, 6))
        status.pack(fill=tk.X)

    def _build_hero_metric(self, parent, column: int, label_text: str, value_var: tk.StringVar):
        frame = ttk.Frame(parent, style="App.TFrame")
        frame.grid(row=0, column=column, sticky=tk.EW, padx=(0, 12) if column < 3 else 0)
        ttk.Label(frame, text=label_text, style="MetricName.TLabel").pack(anchor=tk.W)
        ttk.Label(frame, textvariable=value_var, style="HeroValue.TLabel").pack(anchor=tk.W, pady=(2, 0))

    def _append_log(self, message: str):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)

    def _set_snapshot_text(self, content: str):
        self.snapshot_text.configure(state=tk.NORMAL)
        self.snapshot_text.delete("1.0", tk.END)
        self.snapshot_text.insert(tk.END, content)
        self.snapshot_text.configure(state=tk.DISABLED)

    def _update_profile_summary(self, profile: dict):
        self.profile_text.configure(state=tk.NORMAL)
        self.profile_text.delete("1.0", tk.END)
        self.profile_text.insert(tk.END, format_profile_summary(profile))
        self.profile_text.configure(state=tk.DISABLED)

    def _blend_color(self, start: str, end: str, ratio: float) -> str:
        ratio = max(0.0, min(1.0, ratio))
        start_rgb = tuple(int(start[index:index + 2], 16) for index in (1, 3, 5))
        end_rgb = tuple(int(end[index:index + 2], 16) for index in (1, 3, 5))
        mixed = tuple(int(start_value + (end_value - start_value) * ratio) for start_value, end_value in zip(start_rgb, end_rgb))
        return f"#{mixed[0]:02x}{mixed[1]:02x}{mixed[2]:02x}"

    def _polar_point(self, center_x: float, center_y: float, radius: float, angle_deg10: int) -> tuple[float, float]:
        angle_rad = math.radians((angle_deg10 / 10.0) - 90.0)
        return (
            center_x + math.cos(angle_rad) * radius,
            center_y + math.sin(angle_rad) * radius,
        )

    def _draw_pointer(self, center_x: float, center_y: float, radius: float, angle_deg10: int, color: str, width: int):
        end_x, end_y = self._polar_point(center_x, center_y, radius, angle_deg10)
        self.motion_canvas.create_line(center_x, center_y, end_x, end_y, fill=color, width=width, capstyle=tk.ROUND)
        self.motion_canvas.create_oval(end_x - 5, end_y - 5, end_x + 5, end_y + 5, fill=color, outline="")

    def _draw_dial_shell(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        accent: str,
        title: str | None = None,
        title_y: float | None = None,
        title_font: tuple | None = None,
    ):
        outer_color = self._blend_color(accent, "#ffffff", 0.18)
        inner_color = self._blend_color(VISUAL_PANEL, accent, 0.12)
        self.motion_canvas.create_oval(
            center_x - radius - 18,
            center_y - radius - 18,
            center_x + radius + 18,
            center_y + radius + 18,
            outline=self._blend_color(accent, VISUAL_BG_BOTTOM, 0.45),
            width=2,
        )
        self.motion_canvas.create_oval(
            center_x - radius,
            center_y - radius,
            center_x + radius,
            center_y + radius,
            fill=inner_color,
            outline=outer_color,
            width=3,
        )
        for tick_index in range(12):
            tick_angle = tick_index * 300
            inner_x, inner_y = self._polar_point(center_x, center_y, radius - 12, tick_angle)
            outer_x, outer_y = self._polar_point(center_x, center_y, radius + 6, tick_angle)
            self.motion_canvas.create_line(inner_x, inner_y, outer_x, outer_y, fill=self._blend_color(accent, "#ffffff", 0.22), width=2)
        if title is not None:
            self.motion_canvas.create_text(
                center_x,
                title_y if title_y is not None else (center_y - radius - 30),
                text=title,
                fill="#f5efe2",
                font=title_font if title_font is not None else ("Segoe UI Semibold", 13),
            )

    def _draw_knob_visual(self, center_x: float, center_y: float, radius: float, snapshot: dict, compact: bool = False):
        knob_angle = int(snapshot["knob_angle_deg10"])
        knob_online = int(snapshot["knob_online"]) != 0
        accent = KNOB_COLOR if knob_online else "#73808a"
        pointer_width = 4 if compact else 5
        marker_radius = 5 if compact else 6
        primary_text_gap = 14 if compact else 26
        self._draw_dial_shell(center_x, center_y, radius, accent)
        self.motion_canvas.create_oval(
            center_x - radius * 0.40,
            center_y - radius * 0.40,
            center_x + radius * 0.40,
            center_y + radius * 0.40,
            fill=self._blend_color(VISUAL_PANEL, accent, 0.18),
            outline=self._blend_color(accent, "#ffffff", 0.12),
            width=2,
        )
        self._draw_pointer(center_x, center_y, radius * 0.82, knob_angle, accent, pointer_width)
        marker_x, marker_y = self._polar_point(center_x, center_y, radius + (10 if compact else 14), knob_angle)
        self.motion_canvas.create_oval(
            marker_x - marker_radius,
            marker_y - marker_radius,
            marker_x + marker_radius,
            marker_y + marker_radius,
            fill=accent,
            outline="",
        )
        self.motion_canvas.create_text(
            center_x,
            center_y + radius + primary_text_gap,
            text=format_deg10(knob_angle),
            fill="#f5efe2",
            font=("Consolas", 10 if compact else 12, "bold"),
        )
        self.motion_canvas.create_text(
            center_x,
            center_y,
            text="ONLINE" if knob_online else "OFFLINE",
            fill=self._blend_color(accent, "#ffffff", 0.22),
            font=("Segoe UI Semibold", 9 if compact else 11),
        )

    def _draw_motor_visual(self, center_x: float, center_y: float, radius: float, snapshot: dict, compact: bool = False):
        target_angle = int(snapshot["target_angle_deg10"])
        actual_angle = int(snapshot["actual_angle_deg10"])
        command_angle = int(snapshot["command_angle_deg10"])
        follower_online = int(snapshot["follower_online"]) != 0
        accent = ACTUAL_COLOR if follower_online else "#73808a"
        target_marker_radius = 5 if compact else 6
        actual_marker_radius = 6 if compact else 7
        self._draw_dial_shell(center_x, center_y, radius, accent)

        for blade_offset in (0, 1200, 2400):
            blade_angle = actual_angle + blade_offset
            tip_x, tip_y = self._polar_point(center_x, center_y, radius * 0.68, blade_angle)
            root_x, root_y = self._polar_point(center_x, center_y, radius * 0.14, blade_angle + 1800)
            self.motion_canvas.create_line(root_x, root_y, tip_x, tip_y, fill=accent, width=4 if compact else 5, capstyle=tk.ROUND)

        self.motion_canvas.create_oval(
            center_x - radius * 0.18,
            center_y - radius * 0.18,
            center_x + radius * 0.18,
            center_y + radius * 0.18,
            fill=self._blend_color(accent, "#ffffff", 0.15),
            outline="",
        )

        target_x, target_y = self._polar_point(center_x, center_y, radius + (10 if compact else 16), target_angle)
        command_x, command_y = self._polar_point(center_x, center_y, radius + (0 if compact else 2), command_angle)
        actual_x, actual_y = self._polar_point(center_x, center_y, radius + (18 if compact else 30), actual_angle)
        self.motion_canvas.create_oval(
            target_x - target_marker_radius,
            target_y - target_marker_radius,
            target_x + target_marker_radius,
            target_y + target_marker_radius,
            fill=TARGET_COLOR,
            outline="",
        )
        self.motion_canvas.create_oval(
            command_x - target_marker_radius,
            command_y - target_marker_radius,
            command_x + target_marker_radius,
            command_y + target_marker_radius,
            fill=COMMAND_COLOR,
            outline="",
        )
        self.motion_canvas.create_oval(
            actual_x - actual_marker_radius,
            actual_y - actual_marker_radius,
            actual_x + actual_marker_radius,
            actual_y + actual_marker_radius,
            fill=accent,
            outline="",
        )
        self._draw_pointer(center_x, center_y, radius * 0.90, target_angle, TARGET_COLOR, 3)
        self._draw_pointer(center_x, center_y, radius * 0.76, command_angle, COMMAND_COLOR, 3)
        self._draw_pointer(center_x, center_y, radius * 0.62, actual_angle, accent, 4 if compact else 5)

        self.motion_canvas.create_text(
            center_x,
            center_y + radius + (12 if compact else 16),
            text=(
                f"T {format_deg10(target_angle)}   A {format_deg10(actual_angle)}"
                if compact
                else f"Target {format_deg10(target_angle)}   Actual {format_deg10(actual_angle)}"
            ),
            fill="#f5efe2",
            font=("Consolas", 9 if compact else 11, "bold"),
        )
        if not compact:
            self.motion_canvas.create_text(
                center_x,
                center_y + radius + 38,
                text=f"Command {format_deg10(command_angle)}",
                fill="#d8e8ee",
                font=("Segoe UI", 10),
            )

    def _sparkline_points(self, values: deque[int], x0: float, y0: float, width: float, height: float, max_abs: int) -> list[float]:
        if len(values) < 2:
            return []
        center_y = y0 + height / 2.0
        step_x = width / max(len(values) - 1, 1)
        points: list[float] = []
        scale = max(1, max_abs)
        for index, value in enumerate(values):
            x = x0 + (index * step_x)
            normalized = max(-1.0, min(1.0, value / scale))
            y = center_y - normalized * (height * 0.42)
            points.extend((x, y))
        return points

    def _draw_history_panel(
        self,
        panel_x0: float,
        panel_y0: float,
        panel_x1: float,
        panel_y1: float,
        snapshot: dict | None,
        compact: bool,
    ):
        panel_height = panel_y1 - panel_y0
        if panel_height < 54:
            return

        self.motion_canvas.create_rectangle(
            panel_x0,
            panel_y0,
            panel_x1,
            panel_y1,
            fill=self._blend_color(VISUAL_PANEL, "#1b4150", 0.35),
            outline=self._blend_color(VISUAL_GRID, "#ffffff", 0.10),
            width=2,
        )
        self.motion_canvas.create_text(
            panel_x0 + 16,
            panel_y0 + 14,
            text="Live Trend",
            anchor=tk.W,
            fill="#f5efe2",
            font=("Segoe UI Semibold", 10 if compact else 11),
        )

        if panel_height < 84:
            if snapshot is not None:
                self.motion_canvas.create_text(
                    panel_x1 - 14,
                    panel_y0 + 14,
                    text=f"Err {int(snapshot['angle_error_deg10'])}   Cur {int(snapshot['current_command_ma'])}   Spd {int(snapshot['filtered_speed_rpm'])}",
                    anchor=tk.E,
                    fill="#d8e8ee",
                    font=("Consolas", 8 if compact else 9, "bold"),
                )
            return

        upper_y0 = panel_y0 + 28
        upper_height = (panel_y1 - upper_y0 - 10) * 0.48
        lower_y0 = upper_y0 + upper_height + 10
        lower_height = (panel_y1 - lower_y0 - 8)
        line_left = panel_x0 + 20
        line_width = panel_x1 - line_left - 18

        label_font = ("Segoe UI", 8 if compact else 9)
        self.motion_canvas.create_text(line_left, upper_y0 - 8, text="Error (deg10)", anchor=tk.W, fill=TARGET_COLOR, font=label_font)
        self.motion_canvas.create_text(line_left, lower_y0 - 8, text="Current (mA) / Speed (rpm)", anchor=tk.W, fill="#d8e8ee", font=label_font)

        for band_y in (upper_y0 + upper_height / 2, lower_y0 + lower_height / 2):
            self.motion_canvas.create_line(line_left, band_y, panel_x1 - 14, band_y, fill=self._blend_color(VISUAL_GRID, "#ffffff", 0.08), dash=(4, 4))

        error_scale = max(180, max((abs(value) for value in self.error_history_deg10), default=180))
        current_scale = max(120, max((abs(value) for value in self.current_history_ma), default=120))
        speed_scale = max(60, max((abs(value) for value in self.speed_history_rpm), default=60))

        error_points = self._sparkline_points(self.error_history_deg10, line_left, upper_y0, line_width, upper_height, error_scale)
        current_points = self._sparkline_points(self.current_history_ma, line_left, lower_y0, line_width, lower_height, current_scale)
        speed_points = self._sparkline_points(self.speed_history_rpm, line_left, lower_y0, line_width, lower_height, speed_scale)

        if error_points:
            self.motion_canvas.create_line(*error_points, fill=TARGET_COLOR, width=3, smooth=True)
        if current_points:
            self.motion_canvas.create_line(*current_points, fill=COMMAND_COLOR, width=2, smooth=True)
        if speed_points:
            self.motion_canvas.create_line(*speed_points, fill=SPEED_COLOR, width=2, smooth=True)

        if snapshot is not None:
            self.motion_canvas.create_text(
                panel_x1 - 14,
                panel_y0 + 14,
                text=f"Err {int(snapshot['angle_error_deg10'])}   Cur {int(snapshot['current_command_ma'])}   Spd {int(snapshot['filtered_speed_rpm'])}",
                anchor=tk.E,
                fill="#d8e8ee",
                font=("Consolas", 8 if compact else 10, "bold"),
            )

    def _draw_live_visualization(self, snapshot: dict | None):
        if not hasattr(self, "motion_canvas"):
            return

        width = max(self.motion_canvas.winfo_width(), 1)
        height = max(self.motion_canvas.winfo_height(), 1)
        if (width < 120) or (height < 120):
            return

        compact = (width < 760) or (height < 320)
        header_height = 54 if compact else 72
        trend_height = max(70 if compact else 92, int(height * (0.24 if compact else 0.28)))
        side_margin = 18 if compact else 28
        dial_gap = 40 if compact else 52
        dial_spread = 0.40 if compact else 0.34
        dial_title_gap = 20 if compact else 28
        dial_footer_gap = 28 if compact else 52
        max_radius_by_width = (width - (side_margin * 2) - dial_gap - 24) / 4.0
        max_radius_by_height = (height - header_height - trend_height - dial_title_gap - dial_footer_gap - 28) / 2.0
        dial_radius = max(34.0, min(max_radius_by_width, max_radius_by_height, 92.0 if not compact else 72.0))
        if max_radius_by_width < 34.0 or max_radius_by_height < 34.0:
            compact = True
            header_height = 50
            trend_height = max(66, int(height * 0.22))
            side_margin = 16
            dial_gap = 30
            dial_spread = 0.44
            dial_title_gap = 18
            dial_footer_gap = 24
            max_radius_by_width = (width - (side_margin * 2) - dial_gap - 16) / 4.0
            max_radius_by_height = (height - header_height - trend_height - dial_title_gap - dial_footer_gap - 22) / 2.0
            dial_radius = max(28.0, min(max_radius_by_width, max_radius_by_height, 68.0))

        center_anchor_x = width / 2.0
        left_edge_center_x = side_margin + dial_radius + 8
        right_edge_center_x = width - side_margin - dial_radius - 8
        centered_left_x = center_anchor_x - dial_radius - (dial_gap / 2.0)
        centered_right_x = center_anchor_x + dial_radius + (dial_gap / 2.0)
        left_center_x = centered_left_x - ((centered_left_x - left_edge_center_x) * dial_spread)
        right_center_x = centered_right_x + ((right_edge_center_x - centered_right_x) * dial_spread)
        if left_center_x < left_edge_center_x or right_center_x > right_edge_center_x:
            left_center_x = left_edge_center_x
            right_center_x = right_edge_center_x
        center_y = header_height + dial_title_gap + dial_radius
        max_center_y = height - trend_height - dial_footer_gap - dial_radius - 10
        if center_y > max_center_y:
            dial_radius = max(26.0, max_center_y - header_height - dial_title_gap)
            left_edge_center_x = side_margin + dial_radius + 8
            right_edge_center_x = width - side_margin - dial_radius - 8
            centered_left_x = center_anchor_x - dial_radius - (dial_gap / 2.0)
            centered_right_x = center_anchor_x + dial_radius + (dial_gap / 2.0)
            left_center_x = centered_left_x - ((centered_left_x - left_edge_center_x) * dial_spread)
            right_center_x = centered_right_x + ((right_edge_center_x - centered_right_x) * dial_spread)
            if left_center_x < left_edge_center_x or right_center_x > right_edge_center_x:
                left_center_x = left_edge_center_x
                right_center_x = right_edge_center_x
            center_y = header_height + dial_title_gap + dial_radius

        dial_title_y = header_height + (8 if compact else 10)
        trend_y0 = max(center_y + dial_radius + dial_footer_gap, height - trend_height)
        bridge_margin = max(10.0, min(22.0 if compact else 28.0, dial_radius * 0.24))
        self.motion_canvas.delete("all")

        gradient_steps = 14 if compact else 20
        for step in range(gradient_steps):
            ratio = step / max(gradient_steps - 1, 1)
            color = self._blend_color(VISUAL_BG_TOP, VISUAL_BG_BOTTOM, ratio)
            y0 = height * ratio
            y1 = height * ((step + 1) / gradient_steps)
            self.motion_canvas.create_rectangle(0, y0, width, y1, fill=color, outline="")

        self.motion_canvas.create_rectangle(0, 0, width, header_height, fill=self._blend_color(VISUAL_BG_TOP, "#214457", 0.35), outline="")
        self.motion_canvas.create_oval(
            left_center_x - dial_radius - (12 if compact else 18),
            center_y - dial_radius - (12 if compact else 18),
            left_center_x + dial_radius + (12 if compact else 18),
            center_y + dial_radius + (12 if compact else 18),
            outline=self._blend_color(KNOB_COLOR, VISUAL_BG_BOTTOM, 0.55),
            width=2,
        )
        self.motion_canvas.create_oval(
            right_center_x - dial_radius - (12 if compact else 18),
            center_y - dial_radius - (12 if compact else 18),
            right_center_x + dial_radius + (12 if compact else 18),
            center_y + dial_radius + (12 if compact else 18),
            outline=self._blend_color(ACTUAL_COLOR, VISUAL_BG_BOTTOM, 0.58),
            width=2,
        )
        self.motion_canvas.create_line(0, trend_y0, width, trend_y0, fill=self._blend_color(VISUAL_GRID, "#ffffff", 0.10))

        if snapshot is None:
            self.motion_canvas.create_text(
                width / 2,
                height / 2 - 12,
                text="Connect to MCP / ASPEP to animate the knob and follower motor.",
                fill="#f5efe2",
                font=("Segoe UI Semibold", 12 if compact else 16),
            )
            self.motion_canvas.create_text(
                width / 2,
                height / 2 + 18,
                text="Live angle, command, target, actual, error, current, and speed trends will appear here.",
                fill="#b9c8cf",
                font=("Segoe UI", 9 if compact else 10),
            )
            return

        zone_state = int(snapshot["settle_active"])
        zone_color = ZONE_COLORS.get(zone_state, ZONE_COLORS[0])
        zone_label = zone_text(snapshot).upper()
        link_y = center_y - (8 if compact else 10)

        self.motion_canvas.create_line(
            left_center_x + dial_radius + bridge_margin,
            link_y,
            (left_center_x + right_center_x) / 2.0,
            link_y - (8 if compact else 14),
            right_center_x - dial_radius - bridge_margin,
            link_y,
            smooth=True,
            width=6 if compact else 10,
            fill=self._blend_color(zone_color, "#ffffff", 0.18),
        )
        self.motion_canvas.create_text(
            left_center_x,
            dial_title_y,
            text="Knob",
            fill="#f5efe2",
            font=("Segoe UI Semibold", 10 if compact else 13),
        )
        self.motion_canvas.create_text(
            right_center_x,
            dial_title_y,
            text="Follower Motor",
            fill="#f5efe2",
            font=("Segoe UI Semibold", 10 if compact else 13),
        )
        self.motion_canvas.create_text(
            width / 2,
            header_height * 0.34,
            text=zone_label,
            fill=zone_color,
            font=("Segoe UI Semibold", 13 if compact else 18),
        )
        self.motion_canvas.create_text(
            width / 2,
            header_height * 0.72,
            text=f"Error {format_signed_deg10(int(snapshot['angle_error_deg10']))}   Speed {int(snapshot['filtered_speed_rpm'])} rpm   Current {int(snapshot['current_command_ma'])} mA",
            fill="#d8e8ee",
            font=("Consolas", 8 if compact else 11, "bold"),
        )

        self._draw_knob_visual(left_center_x, center_y, dial_radius, snapshot, compact=compact)
        self._draw_motor_visual(right_center_x, center_y, dial_radius, snapshot, compact=compact)
        self._draw_history_panel(18, trend_y0 + 6, width - 18, height - 12, snapshot, compact)

    def _on_motion_canvas_configure(self, _event):
        self._draw_live_visualization(self.latest_snapshot)

    def _reset_visual_history(self):
        self.error_history_deg10.clear()
        self.current_history_ma.clear()
        self.speed_history_rpm.clear()

    def _record_snapshot_history(self, snapshot: dict):
        self.error_history_deg10.append(int(snapshot["angle_error_deg10"]))
        self.current_history_ma.append(int(snapshot["current_command_ma"]))
        self.speed_history_rpm.append(int(snapshot["filtered_speed_rpm"]))

    def _reset_dashboard(self):
        self.latest_snapshot = None
        for variable in self.metric_vars.values():
            variable.set("-")
        self._reset_visual_history()
        self._set_snapshot_text("Disconnected\n")
        self._draw_live_visualization(None)

    def _refresh_preset_list(self):
        preset_paths = list_preset_paths()
        self.preset_files = {path.stem: path for path in preset_paths}
        preset_names = list(self.preset_files.keys())
        self.preset_combo["values"] = preset_names
        if preset_names:
            if self.preset_var.get() not in self.preset_files:
                self.preset_var.set(DEFAULT_PRESET_NAME if DEFAULT_PRESET_NAME in self.preset_files else preset_names[0])
            self._update_preset_hint()
        else:
            self.preset_var.set("")
            self.preset_hint_label.configure(text="No preset files found.")

    def _refresh_port_list(self):
        if serial is None:
            self.port_map = {}
            self.port_combo["values"] = []
            self.port_var.set("")
            self.status_var.set("pyserial is not installed in the current Python environment")
            return

        ports = list_serial_ports()
        self.port_map = {f"{port.device} - {port.description}": port.device for port in ports}
        labels = list(self.port_map.keys())
        self.port_combo["values"] = labels
        if labels:
            if self.port_var.get() not in self.port_map:
                preferred = next((label for label in labels if self.port_map[label] == "COM4"), labels[0])
                self.port_var.set(preferred)
            self.status_var.set(f"Detected {len(labels)} serial port(s)")
        else:
            self.port_var.set("")
            self.status_var.set("No serial ports detected")

    def _update_preset_hint(self):
        preset_name = self.preset_var.get()
        self.preset_hint_label.configure(text=PRESET_HINTS.get(preset_name, ""))

    def _selected_port_device(self) -> str:
        selected = self.port_var.get()
        return self.port_map.get(selected, selected.strip())

    def _sync_quick_fields(self, profile: dict):
        for key, _label in QUICK_FIELDS:
            self.quick_vars[key].set(str(profile.get(key, "")))

    def _base_profile(self) -> dict:
        return dict(self.runtime_profile)

    def _collect_quick_overrides(self) -> dict:
        overrides = {}
        for key, _label in QUICK_FIELDS:
            text = self.quick_vars[key].get().strip()
            if not text:
                raise ValueError(f"{key} cannot be empty")
            overrides[key] = int(text)
        return overrides

    def _apply_quick_overrides(self, profile: dict) -> dict:
        updated = dict(profile)
        updated.update(self._collect_quick_overrides())

        max_current = max(0, int(updated["max_current_ma"]))
        updated["max_current_ma"] = max_current
        updated["settle_max_current_ma"] = min(int(updated.get("settle_max_current_ma", max_current)), max_current)
        updated["settle_min_current_ma"] = min(max(0, int(updated["settle_min_current_ma"])), max_current)
        updated["hold_exit_deg10"] = max(0, int(updated["hold_exit_deg10"]))
        if int(updated["hold_enter_deg10"]) > int(updated["hold_exit_deg10"]):
            updated["hold_enter_deg10"] = int(updated["hold_exit_deg10"])
        updated["target_angle_slew_max_deg10_per_tick"] = max(0, int(updated["target_angle_slew_max_deg10_per_tick"]))
        if int(updated["target_angle_slew_min_deg10_per_tick"]) > int(updated["target_angle_slew_max_deg10_per_tick"]):
            updated["target_angle_slew_min_deg10_per_tick"] = int(updated["target_angle_slew_max_deg10_per_tick"])
        return updated

    def _require_connection(self) -> bool:
        if self.live_client is not None:
            return True
        messagebox.showerror("Board Not Connected", "Connect to the board first.")
        return False

    def _start_live_worker(self, worker, *, busy_message: str = "Another live operation is running", error_context: str):
        thread = threading.Thread(
            target=self._run_live_worker,
            args=(worker, busy_message, error_context),
            daemon=True,
        )
        thread.start()

    def _run_live_worker(self, worker, busy_message: str, error_context: str):
        if not self.live_operation_lock.acquire(blocking=False):
            if busy_message:
                self.event_queue.put(("status", busy_message))
            return
        try:
            worker()
        except Exception as exc:
            self.event_queue.put(("error", error_context, str(exc)))
        finally:
            self.live_operation_lock.release()

    def connect_live(self):
        if serial is None:
            messagebox.showerror("Connect Failed", "pyserial is not installed")
            return
        if self.live_client is not None:
            self.status_var.set("Board already connected")
            return

        port = self._selected_port_device()
        if not port:
            messagebox.showerror("Connect Failed", "Please choose a COM port")
            return

        self.status_var.set(f"Connecting to {port}...")
        self._set_snapshot_text(f"Connecting to {port}...\n")

        def worker():
            client = AspepMcpClient(port)
            hello = client.connect()
            profile = client.read_profile()
            snapshot = client.read_snapshot()
            self.event_queue.put(("connected", client, port, hello))
            self.event_queue.put(("profile", profile))
            self.event_queue.put(("snapshot", snapshot))
            self.event_queue.put(("log", f"Connected to {port}; protocol v{hello['protocol_version']}"))
            self.event_queue.put(("status", f"Connected to {port}"))

        self._start_live_worker(worker, error_context="connect")

    def disconnect_live(self):
        self.live_poll_enabled = False
        self.live_poll_inflight = False
        client = self.live_client
        self.live_client = None
        self.connection_info = None
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        self.status_var.set("Disconnected")
        self._append_log("Disconnected from board")
        self._reset_dashboard()

    def read_board_state(self):
        if not self._require_connection():
            return

        self.status_var.set("Reading runtime profile and snapshot...")

        def worker():
            assert self.live_client is not None
            profile = self.live_client.read_profile()
            snapshot = self.live_client.read_snapshot()
            self.event_queue.put(("profile", profile))
            self.event_queue.put(("snapshot", snapshot))
            self.event_queue.put(("status", "Runtime profile and snapshot refreshed"))

        self._start_live_worker(worker, error_context="read")

    def preview_selected_preset(self):
        preset_name = self.preset_var.get()
        self._update_preset_hint()
        if preset_name not in self.preset_files:
            messagebox.showerror("Preset Preview Failed", "Choose a valid preset first.")
            return
        profile = load_json(self.preset_files[preset_name])
        self._sync_quick_fields(profile)
        self._update_profile_summary(profile)
        self.status_var.set(f"Previewing preset: {preset_name}")
        self._append_log(f"Preview preset {preset_name}")

    def _apply_profile_async(self, profile: dict, success_message: str, log_message: str):
        if not self._require_connection():
            return

        def worker():
            assert self.live_client is not None
            applied_profile = self.live_client.write_profile(profile)
            snapshot = self.live_client.read_snapshot()
            self.event_queue.put(("profile", applied_profile))
            self.event_queue.put(("snapshot", snapshot))
            self.event_queue.put(("log", log_message))
            self.event_queue.put(("status", success_message))

        self._start_live_worker(worker, error_context="apply")

    def apply_selected_preset(self):
        preset_name = self.preset_var.get()
        self._update_preset_hint()
        if preset_name not in self.preset_files:
            messagebox.showerror("Apply Preset Failed", "Choose a valid preset first.")
            return

        profile = load_json(self.preset_files[preset_name])
        profile = self._apply_quick_overrides(profile)
        self._apply_profile_async(profile, f"Applied preset: {preset_name}", f"Applied preset {preset_name} over MCP")

    def apply_scenario(self, scenario_name: str):
        if scenario_name not in SCENARIOS:
            messagebox.showerror("Scenario Failed", f"Unknown scenario: {scenario_name}")
            return

        scenario = SCENARIOS[scenario_name]
        if "preset" in scenario:
            preset_name = scenario["preset"]
            if preset_name not in self.preset_files:
                messagebox.showerror("Scenario Failed", f"Preset not found: {preset_name}")
                return
            profile = load_json(self.preset_files[preset_name])
        else:
            profile = self._base_profile()
            profile.update(scenario)

        profile = self._apply_quick_overrides(profile)
        self._apply_profile_async(profile, f"Applied scenario: {scenario_name}", f"Scenario applied: {scenario_name}")

    def apply_quick_tuning(self):
        try:
            profile = self._apply_quick_overrides(self._base_profile())
        except Exception as exc:
            messagebox.showerror("Quick Tuning Failed", str(exc))
            return

        self._apply_profile_async(profile, "Quick tuning applied", "Applied quick tuning overrides")

    def reset_runtime_profile(self):
        if not self._require_connection():
            return

        def worker():
            assert self.live_client is not None
            profile = self.live_client.reset_profile()
            snapshot = self.live_client.read_snapshot()
            self.event_queue.put(("profile", profile))
            self.event_queue.put(("snapshot", snapshot))
            self.event_queue.put(("log", "Reset runtime profile to firmware defaults"))
            self.event_queue.put(("status", "Runtime profile reset"))

        self._start_live_worker(worker, error_context="reset")

    def _poll_live_snapshot_loop(self):
        if (self.live_client is None) or (not self.live_poll_enabled):
            self.live_poll_inflight = False
            return

        if not self.live_poll_inflight:
            self.live_poll_inflight = True

            def worker():
                if self.live_client is None:
                    self.event_queue.put(("poll-finished",))
                    return
                try:
                    snapshot = self.live_client.read_snapshot()
                    self.event_queue.put(("snapshot", snapshot))
                except Exception as exc:
                    self.event_queue.put(("disconnected", str(exc)))
                finally:
                    self.event_queue.put(("poll-finished",))

            self._start_live_worker(worker, busy_message="", error_context="poll")

        self.after(POLL_INTERVAL_MS, self._poll_live_snapshot_loop)

    def _update_dashboard(self, snapshot: dict):
        self.latest_snapshot = snapshot
        self._record_snapshot_history(snapshot)
        self.metric_vars["mode"].set(TEST_MODE_NAMES.get(int(snapshot["test_mode"]), str(snapshot["test_mode"])))
        self.metric_vars["zone"].set(zone_text(snapshot))
        self.metric_vars["controller"].set(active_idle(snapshot["active"]))
        self.metric_vars["hold"].set(hold_state(snapshot["hold_active"]))
        self.metric_vars["source"].set(on_off(snapshot["source_online"]))
        self.metric_vars["follower"].set(on_off(snapshot["follower_online"]))
        self.metric_vars["knob"].set(format_deg10(int(snapshot["knob_angle_deg10"])))
        self.metric_vars["command"].set(format_deg10(int(snapshot["command_angle_deg10"])))
        self.metric_vars["target"].set(format_deg10(int(snapshot["target_angle_deg10"])))
        self.metric_vars["actual"].set(format_deg10(int(snapshot["actual_angle_deg10"])))
        self.metric_vars["error"].set(format_signed_deg10(int(snapshot["angle_error_deg10"])))
        self.metric_vars["phase"].set(phase_text(snapshot))
        self.metric_vars["target_rpm"].set(f"{int(snapshot['target_rpm'])} rpm")
        self.metric_vars["speed"].set(f"{int(snapshot['filtered_speed_rpm'])} rpm")
        self.metric_vars["current"].set(f"{int(snapshot['current_command_ma'])} mA")
        self.metric_vars["active"].set(active_idle(snapshot["active"]))
        self.metric_vars["knob_online"].set(on_off(snapshot["knob_online"]))
        self.metric_vars["sensored"].set(on_off(snapshot["sensored_online"], "Online", "Offline"))
        self.metric_vars["follower_status"].set(f"0x{int(snapshot['follower_status']) & 0xFF:02X}")
        self.metric_vars["magnet"].set(follower_magnet_text(snapshot))
        self.metric_vars["direction"].set(str(int(snapshot["sensored_runtime_direction_sign"])))
        self.metric_vars["trim"].set(f"{int(snapshot['sensored_runtime_electrical_trim_deg10'])} deg10")
        protocol = self.connection_info["protocol_version"] if self.connection_info is not None else snapshot["protocol_version"]
        self.metric_vars["protocol"].set(f"v{protocol}")
        self.metric_vars["board_mode"].set(board_mode_text(snapshot))
        self._set_snapshot_text(format_snapshot(snapshot))
        self._draw_live_visualization(snapshot)

    def _poll_event_queue(self):
        while not self.event_queue.empty():
            item = self.event_queue.get()
            kind = item[0]
            if kind == "status":
                self.status_var.set(item[1])
            elif kind == "log":
                self._append_log(item[1])
            elif kind == "connected":
                _kind, client, port, hello = item
                self.live_client = client
                self.connection_info = hello
                self.live_poll_enabled = True
                self.live_poll_inflight = False
                self.after(POLL_INTERVAL_MS, self._poll_live_snapshot_loop)
                self._append_log(f"Live polling started on {port}")
            elif kind == "profile":
                profile = item[1]
                self.runtime_profile = profile
                self._sync_quick_fields(profile)
                self._update_profile_summary(profile)
            elif kind == "snapshot":
                self._update_dashboard(item[1])
            elif kind == "disconnected":
                error_message = item[1]
                self.disconnect_live()
                self._append_log(f"Live link closed: {error_message}")
                self.status_var.set(f"Live link closed: {error_message}")
            elif kind == "poll-finished":
                self.live_poll_inflight = False
            elif kind == "error":
                _kind, error_context, error_message = item
                self._append_log(f"{error_context} failed: {error_message}")
                self.status_var.set(f"{error_context} failed: {error_message}")
                if error_context == "connect":
                    self._set_snapshot_text(f"Connect failed\n\n{error_message}\n")
                    messagebox.showerror("Connect Failed", error_message)
                elif error_context == "read":
                    messagebox.showerror("Read Failed", error_message)
                elif error_context == "apply":
                    messagebox.showerror("Apply Failed", error_message)
                elif error_context == "reset":
                    messagebox.showerror("Reset Failed", error_message)
                elif error_context != "poll":
                    messagebox.showerror("Operation Failed", error_message)
        self.after(100, self._poll_event_queue)

    def _on_close(self):
        try:
            self.disconnect_live()
        finally:
            self.destroy()


if __name__ == "__main__":
    app = FollowControlApp()
    try:
        app.mainloop()
    except KeyboardInterrupt:
        try:
            app.destroy()
        except tk.TclError:
            pass