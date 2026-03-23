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
3. Run the host program:

```bash
python host/tensile_tester.py
```

4. In the GUI, choose the board COM port.
5. Turn on `Virtual Simulation` if you want to test communication without real hardware.
6. Leave `Virtual Simulation` off if you want to use the hardware backend.

## Notes

- `boot.py` is required for clean USB serial communication.
- The baud rate can be selected in the GUI.

## Run Tests

```bash
python -m unittest discover -s tests
```
