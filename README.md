# Tensile Tester

Simple host GUI plus CircuitPython firmware for a benchtop tensile tester.

## What You Need

- Python with `tkinter`
- `matplotlib`
- `pyserial`
- A CircuitPython board

## Quick Start

1. Copy everything from `circuitpython/` to the `CIRCUITPY` drive.
2. Reset the board after copying the files.
3. You can also launch the host app and click `Update Board Files` to copy every file from `circuitpython/` to the mounted board.
4. Run the host program:

```bash
python host/tensile_tester.py
```

5. In the GUI, choose the board COM port.
6. Turn on `Virtual Simulation` if you want the microcontroller to generate simulated data.
7. Leave `Virtual Simulation` off if you want the microcontroller to use the hardware backend.

## Notes

- `boot.py` is required for clean USB serial communication.
- The baud rate can be selected in the GUI.
- `Virtual Simulation` still uses the selected board COM port. The simulation runs on the microcontroller, not on the host PC.
- The host app writes a rotating debug log to `logs/tensile_tester.log`.

  This file includes startup details, controller commands, device messages, and uncaught exceptions to help debug problems.

## Run Tests

```bash
python -m unittest discover -s tests
```
