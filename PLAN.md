# Tkinter Operator GUI for Tensile Tester MVP

## Summary
- Build a Python 3.12 Tkinter desktop app in [host/gui.py](/home/jonas/Desktop/Programming/TensileTester/host/gui.py) backed by a mockable controller in [host/tensile_tester.py](/home/jonas/Desktop/Programming/TensileTester/host/tensile_tester.py).
- First version supports connect/disconnect, tare force, zero displacement, jog motion, constant-speed pull start/stop, emergency stop, live numeric readouts, a selectable live plot, and CSV export.
- The GUI must run without hardware by selecting a simulator transport; real device support uses readable JSON-lines messages over serial.

## Interfaces
- `TesterController` exposes `connect(port, baud=115200, use_mock=False)`, `disconnect()`, `tare_force()`, `zero_displacement()`, `jog(direction, distance_mm, speed_mm_per_min)`, `start_test(speed_mm_per_min)`, `stop()`, `estop()`, and `poll_events() -> list[TesterEvent]`.
- `TesterEvent` has three shapes: `status(state, message)`, `sample(timestamp_s, force_n, displacement_mm, stress_mpa, strain_percent)`, and `error(code, message)`.
- The host computes `stress_mpa = force_n / (width_mm * thickness_mm)` and `strain_percent = displacement_mm / gauge_length_mm * 100`; specimen dimensions stay on the host and are not sent to the device.
- Serial wire format is one JSON object per line. Host commands are `{"cmd":"tare_force"}`, `{"cmd":"zero_displacement"}`, `{"cmd":"jog","direction":"forward|reverse","distance_mm":1.0,"speed_mm_per_min":5.0}`, `{"cmd":"start_test","speed_mm_per_min":5.0}`, `{"cmd":"stop"}`, and `{"cmd":"estop"}`. Device messages are `{"type":"status",...}`, `{"type":"sample",...}`, and `{"type":"error",...}`.

## Implementation
- In [host/tensile_tester.py](/home/jonas/Desktop/Programming/TensileTester/host/tensile_tester.py), create a transport layer with `SerialTransport` using `pyserial`, `MockTransport` for simulator mode, and a background reader thread that pushes parsed events into a thread-safe queue consumed by the GUI.
- In [host/gui.py](/home/jonas/Desktop/Programming/TensileTester/host/gui.py), build a single-window operator layout with a connection bar, specimen section, manual control section, run control section, live metric readouts, event log, and embedded Matplotlib plot.
- Connection controls include serial port dropdown, `Simulator` option, connect/disconnect button, and visible state/fault indicator.
- Specimen inputs are `sample_id`, `width_mm`, `thickness_mm`, and `gauge_length_mm`; `Start Test` stays disabled until connected and these fields validate as positive numbers.
- Manual controls include `Tare Force`, `Zero Displacement`, jog `-` and `+` buttons, jog distance input, and jog speed input. Run controls include test speed input, `Start Test`, `Stop`, and `E-Stop`.
- The plot defaults to force vs displacement and offers a mode toggle to stress vs strain; numeric readouts always show force, displacement, stress, strain, elapsed time, and device state.
- Each run buffers all samples in memory from `Start Test` until `Stop`, `E-Stop`, or fault. `Save Run` exports one CSV named `YYYYMMDD_HHMMSS_<sample_id>.csv` with leading `# key,value` metadata rows followed by columns `timestamp_s,force_n,displacement_mm,stress_mpa,strain_percent,state`.
- Add a minimal dependency manifest or brief [README.md](/home/jonas/Desktop/Programming/TensileTester/README.md) setup section covering `matplotlib` and `pyserial`, while noting Tkinter comes from the system Python.

## Test Plan
- Unit test specimen validation and derived calculations, including zero/negative dimensions and correct MPa/percent conversions.
- Unit test controller parsing for `status`, `sample`, malformed JSON, disconnects, and error propagation.
- Unit test CSV export for metadata rows, column order, filename format, and preservation of raw plus derived data.
- Manual acceptance in simulator mode: connect, jog both directions, tare/zero, run a pull at constant speed, confirm live plot updates, stop, save CSV, and confirm the exported file contents.
- Manual acceptance with hardware once firmware exists: verify JSON-line compatibility, disconnect/fault handling, button state transitions, and `E-Stop` behavior.

## Assumptions
- One tester is controlled from one Linux desktop session running Python 3.12.
- Raw device telemetry is force in newtons and displacement in millimeters.
- Stress/strain are engineering values only; no true stress/strain or machine-compliance correction is included in the MVP.
- If firmware message details change later, the GUI contract stays stable and only the transport/parser layer needs adjustment.
