# Tensile Tester

Host-side operator GUI and controller for a benchtop tensile tester.

## Requirements

- `tkinter`
- `matplotlib`
- `pyserial`

## Run the GUI

```bash
python3 host/gui.py
```

Use `Simulator` from the connection dropdown to exercise the full operator workflow without hardware.

## Run Tests

```bash
python3 -m unittest discover -s tests
```
