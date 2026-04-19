# City Sensor Anomaly Demo

Streamlit demo app for city sensor telemetry that:
- receives sensor data over a TCP socket,
- stores all readings in both SQLite and CSV,
- uses scikit-learn Isolation Forest to detect anomalous readings,
- surfaces danger/early-warning alerts in the dashboard.

Currently supported sensor values per reading:
- temperature
- humidity
- pressure
- light

## Features
- TCP socket ingestion for multi-cell sensor payloads
- Dual persistence (`sensors.db` + `sensors.csv`)
- Isolation Forest anomaly detection
- Live auto-refresh chart in the center
- Alert `ragnarok nadszedl` when anomaly is detected or weather-risk rules trigger

## Quick start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
streamlit run app.py
```

Then open the Streamlit URL shown in the terminal (usually `http://localhost:8501`).

## Sensor socket endpoint

The app starts a TCP socket listener at:

- `10.42.0.1:19001`

You can override this with environment variables:
- `SENSOR_SOCKET_HOST`
- `SENSOR_SOCKET_PORT`

## Quick socket test (zsh)

```bash
python - <<'PY'
import socket, struct
host, port = "10.42.0.1", 19001
packet = struct.pack("<dddd", 24.3, 51.2, 1012.4, 420.0)
with socket.create_connection((host, port), timeout=5) as sock:
	sock.sendall(packet)
PY
```

Forwarded binary frame format:
- `temperature` (`double`)
- `humidity` (`double`)
- `pressure` (`double`)
- `light` (`double`)

Byte order is little-endian (`<dddd`), configurable by env `FORWARDED_FRAME_FORMAT`.

## Sensor simulator

`demo_app/sensor_simulator.py` sends continuous synthetic readings for many cells/sensors.

Run a finite simulation:

```bash
python demo_app/sensor_simulator.py --host 10.42.0.1 --port 19001 --cells 8 --sensors-per-cell 4 --count 150 --interval 0.5
```

Run continuously until you stop it (`Ctrl+C`):

```bash
python demo_app/sensor_simulator.py --host 10.42.0.1 --port 19001 --count 0 --interval 0.8
```

## Notes

- The anomaly model trains on existing stored data. Use the sidebar to set contamination and minimum samples.
- If there are not enough samples yet, readings are stored but marked as non-anomalous until training can run.
- `ragnarok nadszedl` appears for Isolation Forest anomalies, temperature crash, sudden light drop, or pressure+humidity storm signal.
- `sensors.db` and `sensors.csv` are created in the project root.
