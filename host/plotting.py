from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, Sequence

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError:
    Figure = None
    FigureCanvasTkAgg = None


PLOT_MODE_FORCE = "force_displacement"
PLOT_MODE_STRESS = "stress_strain"


class PlotSample(Protocol):
    force_n: float
    displacement_mm: float
    stress_mpa: float | None
    strain_percent: float | None


@dataclass
class LivePlot:
    figure: Any
    axes: Any
    canvas: Any

    def redraw(self, samples: Sequence[PlotSample], plot_mode: str) -> None:
        self.axes.clear()
        self.axes.grid(True, linestyle="--", linewidth=0.7, alpha=0.4)

        if not samples:
            if plot_mode == PLOT_MODE_STRESS:
                self.axes.set_xlabel("Strain (%)")
                self.axes.set_ylabel("Stress (MPa)")
                self.axes.set_title("No stress-strain data yet")
            else:
                self.axes.set_xlabel("Displacement (mm)")
                self.axes.set_ylabel("Force (N)")
                self.axes.set_title("No force-displacement data yet")
            self.figure.tight_layout()
            self.canvas.draw_idle()
            return

        if plot_mode == PLOT_MODE_STRESS:
            points = [
                (sample.strain_percent, sample.stress_mpa)
                for sample in samples
                if sample.strain_percent is not None and sample.stress_mpa is not None
            ]
            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]
            self.axes.set_xlabel("Strain (%)")
            self.axes.set_ylabel("Stress (MPa)")
            self.axes.set_title("Stress vs Strain")
            line_color = "#9a3412"
        else:
            x_values = [sample.displacement_mm for sample in samples]
            y_values = [sample.force_n for sample in samples]
            self.axes.set_xlabel("Displacement (mm)")
            self.axes.set_ylabel("Force (N)")
            self.axes.set_title("Force vs Displacement")
            line_color = "#1d4ed8"

        if x_values and y_values:
            self.axes.plot(x_values, y_values, color=line_color, linewidth=2.0)
            self.axes.scatter([x_values[-1]], [y_values[-1]], color=line_color, s=28, zorder=3)
        else:
            self.axes.text(0.5, 0.5, "Derived values unavailable", ha="center", va="center")

        self.figure.tight_layout()
        self.canvas.draw_idle()


def create_live_plot(parent: Any) -> LivePlot:
    if Figure is None or FigureCanvasTkAgg is None:
        raise RuntimeError(
            "matplotlib is required for the GUI. Install it with 'pip install matplotlib'."
        )

    figure = Figure(figsize=(8.0, 6.0), dpi=100)
    axes = figure.add_subplot(111)
    axes.grid(True, linestyle="--", linewidth=0.7, alpha=0.4)
    axes.set_title("No run data yet")
    axes.set_xlabel("Displacement (mm)")
    axes.set_ylabel("Force (N)")

    canvas = FigureCanvasTkAgg(figure, master=parent)
    canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    return LivePlot(figure=figure, axes=axes, canvas=canvas)
