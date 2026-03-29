from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Callable

try:
    from .controller import (
        DEFAULT_BAUD_RATE,
        SpecimenDimensions,
        TesterController,
        TesterEvent,
        get_serial_support_error,
        list_serial_port_infos,
        validate_specimen_dimensions,
    )
    from .deployment import (
        copy_circuitpython_tree,
        find_circuitpython_drive_candidates,
        get_default_firmware_source_dir,
        is_probable_circuitpython_drive,
    )
    from .plotting import PLOT_MODE_FORCE, PLOT_MODE_STRESS, LivePlot, create_live_plot
except ImportError:
    from controller import (
        DEFAULT_BAUD_RATE,
        SpecimenDimensions,
        TesterController,
        TesterEvent,
        get_serial_support_error,
        list_serial_port_infos,
        validate_specimen_dimensions,
    )
    from deployment import (
        copy_circuitpython_tree,
        find_circuitpython_drive_candidates,
        get_default_firmware_source_dir,
        is_probable_circuitpython_drive,
    )
    from plotting import PLOT_MODE_FORCE, PLOT_MODE_STRESS, LivePlot, create_live_plot

try:
    from .debug_logging import configure_debug_logging, get_app_logger, get_configured_log_path
except ImportError:
    from debug_logging import configure_debug_logging, get_app_logger, get_configured_log_path


POLL_INTERVAL_MS = 100
LOG_LINE_LIMIT = 300
DEFAULT_WINDOW_SIZE = "1380x860"
LOGGER = get_app_logger("gui")
BAUD_RATE_OPTIONS = (
    "9600",
    "19200",
    "38400",
    "57600",
    "115200",
    "230400",
    "460800",
    "921600",
)
STATE_COLORS = {
    "connected": "#1d4ed8",
    "idle": "#0f766e",
    "homing": "#0f766e",
    "jogging": "#b45309",
    "running": "#a16207",
    "estop": "#b91c1c",
    "fault": "#991b1b",
    "disconnected": "#475569",
}


@dataclass(frozen=True)
class RunSample:
    timestamp_s: float
    force_n: float
    displacement_mm: float
    stress_mpa: float | None
    strain_percent: float | None
    state: str


@dataclass(frozen=True)
class RunMetadata:
    sample_id: str
    area_mm2: float
    gauge_length_mm: float
    connection_label: str
    started_at: str
    plot_mode: str

    def as_rows(self) -> list[tuple[str, str]]:
        return [
            ("sample_id", self.sample_id),
            ("area_mm2", f"{self.area_mm2:.6f}"),
            ("gauge_length_mm", f"{self.gauge_length_mm:.6f}"),
            ("connection_label", self.connection_label),
            ("started_at", self.started_at),
            ("plot_mode", self.plot_mode),
        ]


def sanitize_filename_component(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    cleaned = cleaned.strip("._-")
    return cleaned or "sample"


def build_run_filename(sample_id: str, timestamp: datetime | None = None) -> str:
    timestamp_value = timestamp or datetime.now()
    sample_component = sanitize_filename_component(sample_id)
    return f"{timestamp_value:%Y%m%d_%H%M%S}_{sample_component}.csv"


def validate_specimen_inputs(
    sample_id: str,
    area_text: str,
    gauge_length_text: str,
) -> tuple[str, SpecimenDimensions]:
    sample_id_value = sample_id.strip()
    if not sample_id_value:
        raise ValueError("Sample ID is required.")

    specimen = validate_specimen_dimensions(
        area_mm2=float(area_text),
        gauge_length_mm=float(gauge_length_text),
    )
    return sample_id_value, specimen


def export_run_csv(path: str | Path, metadata: RunMetadata, samples: list[RunSample]) -> None:
    output_path = Path(path)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        for key, value in metadata.as_rows():
            handle.write(f"# {key},{value}\n")

        writer = csv.writer(handle)
        writer.writerow(
            [
                "timestamp_s",
                "force_n",
                "displacement_mm",
                "stress_mpa",
                "strain_percent",
                "state",
            ]
        )

        for sample in samples:
            writer.writerow(
                [
                    f"{sample.timestamp_s:.6f}",
                    f"{sample.force_n:.6f}",
                    f"{sample.displacement_mm:.6f}",
                    "" if sample.stress_mpa is None else f"{sample.stress_mpa:.6f}",
                    "" if sample.strain_percent is None else f"{sample.strain_percent:.6f}",
                    sample.state,
                ]
            )


def format_metric(value: float | None, precision: int = 3, fallback: str = "--") -> str:
    if value is None:
        return fallback
    return f"{value:.{precision}f}"


def mousewheel_delta_to_units(delta: int) -> int:
    if delta == 0:
        return 0

    magnitude = max(1, abs(int(delta)) // 120) if abs(int(delta)) >= 120 else 1
    return -magnitude if delta > 0 else magnitude


def parse_baud_rate(raw_value: str) -> int:
    try:
        baud_rate = int(raw_value)
    except ValueError as exc:
        raise ValueError("Baud rate must be an integer.") from exc

    if baud_rate <= 0:
        raise ValueError("Baud rate must be greater than zero.")
    return baud_rate


def format_connection_error(exc: Exception, port: str) -> str:
    message = str(exc).strip()

    if isinstance(exc, PermissionError) or "permission denied" in message.lower() or "[errno 13]" in message.lower():
        return (
            f"Could not open {port} because Linux denied access.\n\n"
            "Close any serial monitor or IDE using the device, then add your user to the serial-access group "
            "and sign out/in.\n"
            "Example: sudo usermod -aG dialout $USER\n"
            "Some distributions use 'uucp' instead of 'dialout'."
        )

    if isinstance(exc, FileNotFoundError) or "no such file or directory" in message.lower():
        return (
            f"{port} is no longer available.\n\n"
            "Reconnect the board, click Refresh Ports, and select the current serial device before connecting."
        )

    return message or f"Could not open {port}."


class TensileTesterApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.controller = TesterController()
        self.live_plot: LivePlot | None = None

        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD_RATE))
        self.test_mode_var = tk.BooleanVar(value=False)
        self.sample_id_var = tk.StringVar(value="sample-001")
        self.area_var = tk.StringVar(value="20.0")
        self.gauge_length_var = tk.StringVar(value="25.0")
        self.jog_distance_var = tk.StringVar(value="1.0")
        self.jog_speed_var = tk.StringVar(value="5.0")
        self.test_speed_var = tk.StringVar(value="10.0")
        self.plot_mode_var = tk.StringVar(value=PLOT_MODE_FORCE)
        self.specimen_status_var = tk.StringVar(value="Enter specimen details to enable Start Test.")
        self.connection_message_var = tk.StringVar(value="Select a serial port.")
        self.state_text_var = tk.StringVar(value="Disconnected")
        self.force_var = tk.StringVar(value="-- N")
        self.displacement_var = tk.StringVar(value="-- mm")
        self.stress_var = tk.StringVar(value="-- MPa")
        self.strain_var = tk.StringVar(value="-- %")
        self.elapsed_var = tk.StringVar(value="-- s")
        self.state_metric_var = tk.StringVar(value="disconnected")

        self.available_ports: list[str] = []
        self.port_display_map: dict[str, str] = {}
        self.current_metadata: RunMetadata | None = None
        self.run_samples: list[RunSample] = []
        self.run_active = False
        self._log_lines: list[str] = []
        self._left_panel_window_id: int | None = None

        self.root.title("Tensile Tester")
        self.root.geometry(DEFAULT_WINDOW_SIZE)
        self.root.minsize(1180, 760)
        self.root.option_add("*tearOff", False)
        self.root.protocol("WM_DELETE_WINDOW", self._handle_close)

        self._configure_style()
        self._build_layout()
        self._bind_scroll_behaviors()
        self._refresh_port_list()
        self._bind_validation()
        self._sync_specimen()
        self._refresh_ui_state()
        self._poll_controller()

    def _configure_style(self) -> None:
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(".", font=("TkDefaultFont", 10))
        style.configure("Title.TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))
        style.configure("MetricValue.TLabel", font=("TkDefaultFont", 17, "bold"))
        style.configure("MetricName.TLabel", font=("TkDefaultFont", 9))
        style.configure("Danger.TButton", foreground="#ffffff")

    def _build_layout(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        connection_frame = ttk.Frame(self.root, padding=(14, 14, 14, 10))
        connection_frame.grid(row=0, column=0, sticky="ew")
        connection_frame.columnconfigure(1, weight=1)
        connection_frame.columnconfigure(8, weight=1)

        ttk.Label(connection_frame, text="Connection").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.port_combo = ttk.Combobox(connection_frame, textvariable=self.port_var, state="readonly", width=34)
        self.port_combo.grid(row=0, column=1, sticky="ew")
        self.refresh_ports_button = ttk.Button(connection_frame, text="Refresh Ports", command=self._refresh_port_list)
        self.refresh_ports_button.grid(
            row=0,
            column=2,
            padx=(8, 0),
        )
        self.update_board_files_button = ttk.Button(
            connection_frame,
            text="Update Board Files",
            command=self._update_board_files,
        )
        self.update_board_files_button.grid(row=0, column=3, padx=(8, 0))
        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self._toggle_connection)
        self.connect_button.grid(row=0, column=4, padx=(8, 0))
        self.test_mode_check = ttk.Checkbutton(
            connection_frame,
            text="Virtual Simulation",
            variable=self.test_mode_var,
            command=self._handle_test_mode_change,
        )
        self.test_mode_check.grid(row=0, column=5, padx=(12, 0), sticky="w")
        ttk.Label(connection_frame, text="Baud").grid(row=0, column=6, padx=(14, 4), sticky="e")
        self.baud_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.baud_var,
            state="readonly",
            values=BAUD_RATE_OPTIONS,
            width=10,
        )
        self.baud_combo.grid(row=0, column=7, sticky="w")
        self.state_indicator = tk.Label(
            connection_frame,
            textvariable=self.state_text_var,
            width=18,
            anchor="center",
            fg="white",
            bg=STATE_COLORS["disconnected"],
            padx=10,
            pady=6,
        )
        self.state_indicator.grid(row=0, column=8, sticky="e", padx=(14, 0))
        ttk.Label(connection_frame, textvariable=self.connection_message_var).grid(
            row=1,
            column=0,
            columnspan=9,
            sticky="w",
            pady=(8, 0),
        )

        content = ttk.Frame(self.root, padding=(14, 0, 14, 14))
        content.grid(row=1, column=0, sticky="nsew")
        content.columnconfigure(0, weight=2, minsize=380)
        content.columnconfigure(1, weight=3, minsize=480)
        content.rowconfigure(0, weight=1)

        left_container = ttk.Frame(content)
        left_container.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        left_container.columnconfigure(0, weight=1)
        left_container.rowconfigure(0, weight=1)

        self.left_scroll_canvas = tk.Canvas(
            left_container,
            highlightthickness=0,
            borderwidth=0,
            background=self.root.cget("bg"),
        )
        self.left_scroll_canvas.grid(row=0, column=0, sticky="nsew")
        self.left_scrollbar = ttk.Scrollbar(
            left_container,
            orient="vertical",
            command=self.left_scroll_canvas.yview,
        )
        self.left_scrollbar.grid(row=0, column=1, sticky="ns")
        self.left_scroll_canvas.configure(yscrollcommand=self.left_scrollbar.set)

        self.left_panel = ttk.Frame(self.left_scroll_canvas)
        self.left_panel.columnconfigure(0, weight=1)
        self._left_panel_window_id = self.left_scroll_canvas.create_window(
            (0, 0),
            window=self.left_panel,
            anchor="nw",
        )
        self.left_panel.bind("<Configure>", self._handle_left_panel_configure)
        self.left_scroll_canvas.bind("<Configure>", self._handle_left_canvas_configure)

        right_panel = ttk.Frame(content)
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(1, weight=1)

        self._build_specimen_panel(self.left_panel)
        self._build_manual_panel(self.left_panel)
        self._build_run_panel(self.left_panel)
        self._build_metrics_panel(self.left_panel)
        self._build_log_panel(self.left_panel)
        self._build_plot_panel(right_panel)

    def _build_specimen_panel(self, parent: ttk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Specimen", style="Title.TLabelframe", padding=12)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Sample ID").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Entry(frame, textvariable=self.sample_id_var, width=20).grid(row=0, column=1, sticky="ew", pady=3)
        ttk.Label(frame, text="Area (mm^2)").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(frame, textvariable=self.area_var).grid(row=1, column=1, sticky="ew", pady=3)
        ttk.Label(frame, text="Gauge Length (mm)").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(frame, textvariable=self.gauge_length_var).grid(row=2, column=1, sticky="ew", pady=3)
        ttk.Label(frame, textvariable=self.specimen_status_var, wraplength=290).grid(
            row=3,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(8, 0),
        )

    def _build_manual_panel(self, parent: ttk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Manual Control", style="Title.TLabelframe", padding=12)
        frame.grid(row=1, column=0, sticky="ew", pady=(14, 0))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        self.home_button = ttk.Button(frame, text="Home", command=self._home)
        self.home_button.grid(row=0, column=0, columnspan=2, sticky="ew", pady=3)

        self.tare_button = ttk.Button(frame, text="Tare Force", command=self._tare_force)
        self.tare_button.grid(row=1, column=0, sticky="ew", pady=(10, 3), padx=(0, 6))
        self.zero_button = ttk.Button(frame, text="Zero Displacement", command=self._zero_displacement)
        self.zero_button.grid(row=1, column=1, sticky="ew", pady=(10, 3), padx=(6, 0))

        ttk.Label(frame, text="Jog Distance (mm)").grid(row=2, column=0, sticky="w", pady=(10, 3))
        ttk.Entry(frame, textvariable=self.jog_distance_var).grid(row=3, column=0, sticky="ew", pady=3, padx=(0, 6))
        ttk.Label(frame, text="Jog Speed (mm/min)").grid(row=2, column=1, sticky="w", pady=(10, 3))
        ttk.Entry(frame, textvariable=self.jog_speed_var).grid(row=3, column=1, sticky="ew", pady=3, padx=(6, 0))

        self.jog_reverse_button = ttk.Button(frame, text="Jog -", command=lambda: self._jog("reverse"))
        self.jog_reverse_button.grid(row=4, column=0, sticky="ew", pady=(10, 0), padx=(0, 6))
        self.jog_forward_button = ttk.Button(frame, text="Jog +", command=lambda: self._jog("forward"))
        self.jog_forward_button.grid(row=4, column=1, sticky="ew", pady=(10, 0), padx=(6, 0))

    def _build_run_panel(self, parent: ttk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Run Control", style="Title.TLabelframe", padding=12)
        frame.grid(row=2, column=0, sticky="ew", pady=(14, 0))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Test Speed (mm/min)").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 3))
        ttk.Entry(frame, textvariable=self.test_speed_var).grid(row=1, column=0, columnspan=2, sticky="ew")

        self.start_button = ttk.Button(frame, text="Start Test", command=self._start_test)
        self.start_button.grid(row=2, column=0, sticky="ew", pady=(12, 0), padx=(0, 6))
        self.stop_button = ttk.Button(frame, text="Stop", command=self._stop_test)
        self.stop_button.grid(row=2, column=1, sticky="ew", pady=(12, 0), padx=(6, 0))

        self.estop_button = tk.Button(
            frame,
            text="E-Stop",
            command=self._estop,
            bg="#b91c1c",
            fg="white",
            activebackground="#991b1b",
            activeforeground="white",
            relief="flat",
            padx=12,
            pady=8,
        )
        self.estop_button.grid(row=3, column=0, sticky="ew", pady=(10, 0), padx=(0, 6))
        self.save_button = ttk.Button(frame, text="Save Run", command=self._save_run)
        self.save_button.grid(row=3, column=1, sticky="ew", pady=(10, 0), padx=(6, 0))

    def _build_metrics_panel(self, parent: ttk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Live Metrics", style="Title.TLabelframe", padding=12)
        frame.grid(row=3, column=0, sticky="ew", pady=(14, 0))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        metrics = [
            ("Force", self.force_var),
            ("Displacement", self.displacement_var),
            ("Stress", self.stress_var),
            ("Strain", self.strain_var),
            ("Elapsed", self.elapsed_var),
            ("Device State", self.state_metric_var),
        ]

        for index, (label, variable) in enumerate(metrics):
            row = (index // 2) * 2
            column = index % 2
            cell = ttk.Frame(frame, padding=(4, 2))
            cell.grid(row=row, column=column, sticky="nsew", padx=4, pady=3)
            ttk.Label(cell, text=label, style="MetricName.TLabel").grid(row=0, column=0, sticky="w")
            ttk.Label(cell, textvariable=variable, style="MetricValue.TLabel").grid(row=1, column=0, sticky="w")

    def _build_log_panel(self, parent: ttk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Event Log", style="Title.TLabelframe", padding=12)
        frame.grid(row=4, column=0, sticky="nsew", pady=(14, 0))
        parent.rowconfigure(4, weight=1, minsize=180)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1, minsize=140)

        self.log_widget = tk.Text(frame, height=8, wrap="word", state="disabled")
        self.log_widget.grid(row=0, column=0, sticky="nsew")
        log_scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.log_widget.yview)
        log_scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_widget.configure(yscrollcommand=log_scrollbar.set)
        self.log_widget.bind("<MouseWheel>", self._handle_log_mousewheel, add="+")
        self.log_widget.bind("<Button-4>", self._handle_log_mousewheel, add="+")
        self.log_widget.bind("<Button-5>", self._handle_log_mousewheel, add="+")

    def _bind_scroll_behaviors(self) -> None:
        self.root.bind_all("<MouseWheel>", self._handle_global_mousewheel, add="+")
        self.root.bind_all("<Button-4>", self._handle_global_mousewheel, add="+")
        self.root.bind_all("<Button-5>", self._handle_global_mousewheel, add="+")

    def _handle_left_panel_configure(self, _event: tk.Event) -> None:
        self.left_scroll_canvas.configure(scrollregion=self.left_scroll_canvas.bbox("all"))

    def _handle_left_canvas_configure(self, event: tk.Event) -> None:
        if self._left_panel_window_id is None:
            return
        self.left_scroll_canvas.itemconfigure(self._left_panel_window_id, width=event.width)

    def _handle_global_mousewheel(self, event: tk.Event) -> str | None:
        try:
            target_widget = self.root.winfo_containing(
                self.root.winfo_pointerx(),
                self.root.winfo_pointery(),
            )
        except (KeyError, tk.TclError):
            return None
        if target_widget is None or not self._is_left_panel_widget(target_widget):
            return None

        if getattr(event, "num", None) == 4:
            units = -1
        elif getattr(event, "num", None) == 5:
            units = 1
        else:
            units = mousewheel_delta_to_units(int(getattr(event, "delta", 0)))

        if units == 0:
            return None

        self.left_scroll_canvas.yview_scroll(units, "units")
        return "break"

    def _handle_log_mousewheel(self, event: tk.Event) -> str:
        if getattr(event, "num", None) == 4:
            units = -1
        elif getattr(event, "num", None) == 5:
            units = 1
        else:
            units = mousewheel_delta_to_units(int(getattr(event, "delta", 0)))

        if units != 0:
            self.log_widget.yview_scroll(units, "units")
        return "break"

    def _is_left_panel_widget(self, widget: tk.Misc) -> bool:
        current: tk.Misc | None = widget
        while current is not None:
            if current is self.left_panel or current is self.left_scroll_canvas or current is self.left_scrollbar:
                return True
            try:
                parent_name = current.winfo_parent()
            except tk.TclError:
                return False
            if not parent_name:
                return False
            try:
                current = current.nametowidget(parent_name)
            except (KeyError, tk.TclError):
                return False
        return False

    def _build_plot_panel(self, parent: ttk.Widget) -> None:
        controls = ttk.Frame(parent)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Label(controls, text="Live Plot").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(
            controls,
            text="Force vs Displacement",
            value=PLOT_MODE_FORCE,
            variable=self.plot_mode_var,
            command=self._redraw_plot,
        ).grid(row=0, column=1, padx=(16, 0), sticky="w")
        ttk.Radiobutton(
            controls,
            text="Stress vs Strain",
            value=PLOT_MODE_STRESS,
            variable=self.plot_mode_var,
            command=self._redraw_plot,
        ).grid(row=0, column=2, padx=(12, 0), sticky="w")

        plot_frame = ttk.Frame(parent, padding=(0, 10, 0, 0))
        plot_frame.grid(row=1, column=0, sticky="nsew")
        plot_frame.columnconfigure(0, weight=1)
        plot_frame.rowconfigure(0, weight=1)

        self.live_plot = create_live_plot(plot_frame)

    def _bind_validation(self) -> None:
        for variable in (
            self.sample_id_var,
            self.area_var,
            self.gauge_length_var,
            self.test_speed_var,
            self.jog_distance_var,
            self.jog_speed_var,
        ):
            variable.trace_add("write", self._handle_input_change)

    def _handle_input_change(self, *_args: object) -> None:
        self._sync_specimen()
        self._refresh_ui_state()

    def _sync_specimen(self) -> bool:
        try:
            sample_id, specimen = validate_specimen_inputs(
                self.sample_id_var.get(),
                self.area_var.get(),
                self.gauge_length_var.get(),
            )
        except ValueError as exc:
            self.controller.clear_specimen_dimensions()
            self.specimen_status_var.set(str(exc))
            return False

        self.controller.set_specimen_dimensions(
            area_mm2=specimen.area_mm2,
            gauge_length_mm=specimen.gauge_length_mm,
        )
        self.specimen_status_var.set(f"Specimen ready for {sample_id}.")
        return True

    def _refresh_port_list(self) -> None:
        serial_support_error = get_serial_support_error()
        port_infos = list_serial_port_infos()
        LOGGER.debug(
            "Port refresh found %d port(s): %s",
            len(port_infos),
            [port.device for port in port_infos],
        )

        self.available_ports = [port.label for port in port_infos]
        self.port_display_map = {}
        for port in port_infos:
            self.port_display_map[port.label] = port.device

        self.port_combo.configure(values=self.available_ports)
        current_value = self.port_var.get()
        if port_infos and current_value not in self.available_ports:
            self.port_var.set(port_infos[0].label)
        elif current_value not in self.available_ports:
            self.port_var.set("")

        if serial_support_error:
            self.connection_message_var.set(serial_support_error)
        elif port_infos:
            self.connection_message_var.set(self.port_var.get())
        else:
            self.connection_message_var.set(
                "No serial ports detected. Reconnect the board and look for a COM port, not just CIRCUITPY."
            )

    def _toggle_connection(self) -> None:
        if self.controller.connected:
            LOGGER.info("Disconnect requested from the GUI.")
            self.controller.disconnect()
            self._refresh_ui_state()
            return

        selected_choice = self.port_var.get().strip()
        selected_port = self.port_display_map.get(selected_choice, selected_choice)
        if not selected_choice:
            LOGGER.warning("Connect requested without selecting a port.")
            messagebox.showerror("Connect", "Choose a serial port before connecting.")
            return

        connected = False
        try:
            baud_rate = parse_baud_rate(self.baud_var.get())
            LOGGER.info(
                "Attempting GUI connection to %s at %d baud (simulation=%s).",
                selected_port,
                baud_rate,
                self.test_mode_var.get(),
            )
            self.controller.connect(port=selected_port, baud=baud_rate)
            connected = True
            self.controller.set_device_mode(self.test_mode_var.get())
        except Exception as exc:
            LOGGER.exception("GUI connection attempt failed for %s.", selected_port)
            if connected and self.controller.connected:
                self.controller.disconnect()
            messagebox.showerror(
                "Connect",
                format_connection_error(exc, selected_port),
            )
            return

        LOGGER.info("GUI connected to %s.", selected_port)
        self._append_log(f"Connected to {selected_choice}.")
        self._refresh_ui_state()

    def _handle_test_mode_change(self) -> None:
        if not self.controller.connected:
            self._refresh_ui_state()
            return
        self._send_controller_command(
            "set device mode",
            lambda: self.controller.set_device_mode(self.test_mode_var.get()),
        )

    def _tare_force(self) -> None:
        self._send_controller_command("tare force", self.controller.tare_force)

    def _zero_displacement(self) -> None:
        self._send_controller_command("zero displacement", self.controller.zero_displacement)

    def _home(self) -> None:
        self._send_controller_command("home", self.controller.home)

    def _jog(self, direction: str) -> None:
        try:
            distance_mm = self._parse_positive_number(self.jog_distance_var.get(), "Jog distance")
            speed_mm_per_min = self._parse_positive_number(self.jog_speed_var.get(), "Jog speed")
        except ValueError as exc:
            messagebox.showerror("Jog", str(exc))
            return

        self._send_controller_command(
            f"jog {direction}",
            lambda: self.controller.jog(
                direction=direction,
                distance_mm=distance_mm,
                speed_mm_per_min=speed_mm_per_min,
            )
        )

    def _start_test(self) -> None:
        try:
            sample_id, specimen = validate_specimen_inputs(
                self.sample_id_var.get(),
                self.area_var.get(),
                self.gauge_length_var.get(),
            )
            speed_mm_per_min = self._parse_positive_number(self.test_speed_var.get(), "Test speed")
        except ValueError as exc:
            messagebox.showerror("Start Test", str(exc))
            return

        metadata = RunMetadata(
            sample_id=sample_id,
            area_mm2=specimen.area_mm2,
            gauge_length_mm=specimen.gauge_length_mm,
            connection_label=self.port_var.get().strip() or self.controller.connection_label,
            started_at=datetime.now().isoformat(timespec="seconds"),
            plot_mode=self.plot_mode_var.get(),
        )

        try:
            LOGGER.info(
                "Starting test for sample %s at %.3f mm/min on %s.",
                sample_id,
                speed_mm_per_min,
                metadata.connection_label,
            )
            self.controller.set_specimen_dimensions(
                area_mm2=specimen.area_mm2,
                gauge_length_mm=specimen.gauge_length_mm,
            )
            self.controller.start_test(speed_mm_per_min)
        except Exception as exc:
            LOGGER.exception("Failed to start test for sample %s.", sample_id)
            messagebox.showerror("Start Test", str(exc))
            return

        self.current_metadata = metadata
        self.run_samples = []
        self.run_active = True
        self._append_log(
            f"Run started for {metadata.sample_id} at {speed_mm_per_min:.3f} mm/min on {metadata.connection_label}."
        )
        self._redraw_plot()
        self._refresh_ui_state()

    def _stop_test(self) -> None:
        self._send_controller_command("stop test", self.controller.stop)

    def _estop(self) -> None:
        confirm = messagebox.askyesno("Emergency Stop", "Trigger emergency stop?")
        if not confirm:
            return
        self._send_controller_command("emergency stop", self.controller.estop)

    def _save_run(self) -> None:
        if not self.current_metadata or not self.run_samples:
            messagebox.showerror("Save Run", "No completed run is available to save.")
            return

        filename = build_run_filename(self.current_metadata.sample_id)
        target_path = filedialog.asksaveasfilename(
            title="Save Run Data",
            defaultextension=".csv",
            initialfile=filename,
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
        )
        if not target_path:
            return

        try:
            export_run_csv(target_path, self.current_metadata, self.run_samples)
        except Exception as exc:
            LOGGER.exception("Failed to save run data to %s.", target_path)
            messagebox.showerror("Save Run", str(exc))
            return

        LOGGER.info("Saved run data to %s.", target_path)
        self._append_log(f"Saved run data to {target_path}.")

    def _update_board_files(self) -> None:
        try:
            source_dir = get_default_firmware_source_dir()
        except Exception as exc:
            LOGGER.exception("Failed to resolve the CircuitPython source directory.")
            messagebox.showerror("Update Board Files", str(exc))
            return

        if self.controller.connected:
            should_disconnect = messagebox.askyesno(
                "Update Board Files",
                "Updating code.py or boot.py will restart the board.\n\nDisconnect and continue?",
            )
            if not should_disconnect:
                return

            LOGGER.info("Disconnecting before deploying CircuitPython files.")
            self.controller.disconnect()
            self._refresh_ui_state()

        target_dir = self._select_board_files_target()
        if target_dir is None:
            return

        if not is_probable_circuitpython_drive(target_dir):
            confirm_copy = messagebox.askyesno(
                "Update Board Files",
                f"{target_dir} does not look like a CIRCUITPY drive.\n\nCopy the files there anyway?",
            )
            if not confirm_copy:
                return

        try:
            result = copy_circuitpython_tree(source_dir, target_dir)
        except Exception as exc:
            LOGGER.exception("Failed to deploy CircuitPython files to %s.", target_dir)
            messagebox.showerror("Update Board Files", str(exc))
            return

        summary = f"Copied {result.file_count} file(s) to {result.destination_dir}."
        LOGGER.info(summary)
        self.connection_message_var.set(summary)
        self._append_log(summary)
        self._append_log("Press reset once if the board does not reload automatically after boot.py changes.")
        messagebox.showinfo(
            "Update Board Files",
            f"{summary}\n\nPress reset once if the board does not reload automatically.",
        )
        self._refresh_port_list()
        self._refresh_ui_state()

    def _select_board_files_target(self) -> Path | None:
        candidates = find_circuitpython_drive_candidates()
        if len(candidates) == 1:
            return candidates[0]

        initial_dir = str(candidates[0]) if candidates else str(Path.home())
        selected_dir = filedialog.askdirectory(
            title="Select the CIRCUITPY drive",
            initialdir=initial_dir,
            mustexist=True,
        )
        if not selected_dir:
            return None

        return Path(selected_dir)

    def _send_controller_command(self, action: str, callback: Callable[[], None]) -> None:
        LOGGER.debug("Executing controller action: %s.", action)
        try:
            callback()
        except Exception as exc:
            LOGGER.exception("Controller action failed: %s.", action)
            messagebox.showerror("Controller Error", str(exc))
        finally:
            self._refresh_ui_state()

    def _parse_positive_number(self, raw_value: str, label: str) -> float:
        try:
            value = float(raw_value)
        except ValueError as exc:
            raise ValueError(f"{label} must be a number.") from exc

        if value <= 0:
            raise ValueError(f"{label} must be greater than zero.")
        return value

    def _poll_controller(self) -> None:
        for event in self.controller.poll_events():
            self._handle_event(event)

        self._refresh_ui_state()
        self.root.after(POLL_INTERVAL_MS, self._poll_controller)

    def _handle_event(self, event: TesterEvent) -> None:
        if event.kind == "status":
            if event.message:
                self.connection_message_var.set(event.message)
                self._append_log(event.message)
            self._set_state_display(event.state or self.controller.state)

            if self.run_active and event.state in {"idle", "estop", "fault", "disconnected"}:
                self.run_active = False
                LOGGER.info("Run ended with state %s.", event.state)
                self._append_log(f"Run ended with state {event.state}.")

        elif event.kind == "sample":
            self._update_metrics_from_sample(event)
            if self.run_active:
                sample = RunSample(
                    timestamp_s=event.timestamp_s or 0.0,
                    force_n=event.force_n or 0.0,
                    displacement_mm=event.displacement_mm or 0.0,
                    stress_mpa=event.stress_mpa,
                    strain_percent=event.strain_percent,
                    state=event.state or self.controller.state,
                )
                self.run_samples.append(sample)
                self._redraw_plot()

        elif event.kind == "error":
            self.connection_message_var.set(event.message)
            self._append_log(f"ERROR {event.code}: {event.message}")
            self._set_state_display("fault")

    def _update_metrics_from_sample(self, event: TesterEvent) -> None:
        self.force_var.set(f"{format_metric(event.force_n)} N")
        self.displacement_var.set(f"{format_metric(event.displacement_mm)} mm")
        self.stress_var.set(f"{format_metric(event.stress_mpa)} MPa")
        self.strain_var.set(f"{format_metric(event.strain_percent)} %")
        self.elapsed_var.set(f"{format_metric(event.timestamp_s)} s")
        self.state_metric_var.set(event.state or self.controller.state or "--")

    def _set_state_display(self, state: str) -> None:
        normalized_state = state or "unknown"
        color = STATE_COLORS.get(normalized_state, "#334155")
        self.state_text_var.set(normalized_state.title())
        self.state_indicator.configure(bg=color)
        self.state_metric_var.set(normalized_state)

    def _refresh_ui_state(self) -> None:
        connected = self.controller.connected
        controller_state = self.controller.state
        active_motion = controller_state in {"running", "homing", "jogging"} or self.run_active
        specimen_ready = self._sync_specimen()
        run_available = bool(self.current_metadata and self.run_samples and not active_motion)
        port_selected = bool(self.port_var.get().strip())

        connect_state = "normal" if connected or port_selected else "disabled"
        self.connect_button.configure(text="Disconnect" if connected else "Connect", state=connect_state)
        self.port_combo.configure(state="disabled" if connected else "readonly")
        self.test_mode_check.configure(state="disabled" if connected and active_motion else "normal")
        self.baud_combo.configure(state="disabled" if connected else "readonly")
        self.refresh_ports_button.configure(state="disabled" if connected else "normal")
        self.update_board_files_button.configure(state="disabled" if active_motion else "normal")

        idle_state = connected and controller_state == "idle"
        manual_state = "normal" if idle_state else "disabled"
        tare_zero_state = "normal" if idle_state else "disabled"
        start_state = "normal" if idle_state and specimen_ready and not active_motion else "disabled"
        stop_state = "normal" if connected and active_motion else "disabled"
        estop_state = "normal" if connected else "disabled"
        save_state = "normal" if run_available else "disabled"

        self.home_button.configure(state=manual_state)
        self.tare_button.configure(state=tare_zero_state)
        self.zero_button.configure(state=tare_zero_state)
        self.jog_reverse_button.configure(state=manual_state)
        self.jog_forward_button.configure(state=manual_state)
        self.start_button.configure(state=start_state)
        self.stop_button.configure(state=stop_state)
        self.estop_button.configure(state=estop_state)
        self.save_button.configure(state=save_state)

        if not connected and self.controller.state == "disconnected":
            self._set_state_display("disconnected")
        elif connected:
            self._set_state_display(self.controller.state)

    def _append_log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}"
        self._log_lines.append(line)
        if len(self._log_lines) > LOG_LINE_LIMIT:
            self._log_lines = self._log_lines[-LOG_LINE_LIMIT:]

        self.log_widget.configure(state="normal")
        self.log_widget.delete("1.0", tk.END)
        self.log_widget.insert(tk.END, "\n".join(self._log_lines))
        self.log_widget.see(tk.END)
        self.log_widget.configure(state="disabled")

    def _redraw_plot(self) -> None:
        if self.live_plot is None:
            return
        self.live_plot.redraw(self.run_samples, self.plot_mode_var.get())

    def _handle_close(self) -> None:
        LOGGER.info("Application close requested.")
        try:
            if self.controller.connected:
                self.controller.disconnect()
        finally:
            self.root.destroy()


def launch_gui() -> None:
    log_path = configure_debug_logging()
    LOGGER.info("Launching GUI.")
    root = tk.Tk()
    original_report_callback_exception = root.report_callback_exception

    def report_callback_exception(exc: type[BaseException], val: BaseException, tb) -> None:
        LOGGER.critical(
            "Unhandled Tkinter callback exception.",
            exc_info=(exc, val, tb),
        )
        original_report_callback_exception(exc, val, tb)

    root.report_callback_exception = report_callback_exception
    app = TensileTesterApp(root)
    active_log_path = log_path or get_configured_log_path()
    if active_log_path is not None:
        app._append_log(f"Debug log file: {active_log_path}.")
    else:
        app._append_log("Debug log file could not be created.")
    app._append_log("Application ready.")
    LOGGER.info("GUI ready.")
    root.mainloop()
    LOGGER.info("GUI main loop exited.")
