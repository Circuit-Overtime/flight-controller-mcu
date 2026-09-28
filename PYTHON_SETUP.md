# Python environment setup

The simulator and telemetry tools use a repository-local virtual environment
named `.venv`. Python 3.11 is the validated interpreter version.

## Reinitialize the environment

From the repository root:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip check
```

If `.venv` already exists, move or remove that directory before running the
first command. The directory is ignored by Git and contains no project source.

## Validate imports

```bash
python -c "import numpy, pygame, serial, OpenGL; print('Python environment OK')"
```

## Run the guided receiver capture

Keep all propellers removed:

```bash
source .venv/bin/activate
python simulator/guided_rx_capture.py /dev/ttyACM0 460800 100 logs/rx-guided.csv
```

The serial port may be named `/dev/ttyUSB0` instead. The active user must have
permission to access the selected serial device.
