# Tensile Tester

Host-side operator GUI and controller for a benchtop tensile tester.

## Requirements

- Python 3.12
- Tkinter from the system Python installation
- `matplotlib`
- `pyserial`

Install the Python dependencies with:

```bash
python3 -m pip install -r requirements.txt
```

## Run the GUI

```bash
python3 host/gui.py
```

Use `Simulator` from the connection dropdown to exercise the full operator workflow without hardware.

## Run Tests

```bash
python3 -m unittest discover -s tests
```
