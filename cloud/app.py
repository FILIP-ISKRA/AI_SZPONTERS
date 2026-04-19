import json
import os
import queue
import socket
import sqlite3
import struct
import threading
from datetime import datetime

import pandas as pd
import streamlit as st
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

APP_TITLE = "City Sensor Anomaly Demo"
DB_PATH = "sensors.db"
CSV_PATH = "sensors.csv"
TABLE_NAME = "sensor_readings"
FEATURE_COLUMNS = ["temperature", "moisture", "pressure", "light"]
SENSOR_SOCKET_HOST = os.getenv("SENSOR_SOCKET_HOST", "10.42.0.1")
SENSOR_SOCKET_PORT = int(os.getenv("SENSOR_SOCKET_PORT", "19001"))
FORWARDED_FRAME_FORMAT = os.getenv("FORWARDED_FRAME_FORMAT", "<dddd")
FORWARDED_FRAME_SIZE = struct.calcsize(FORWARDED_FRAME_FORMAT)

socket_queue = queue.Queue()
socket_status = {
    "running": False,
    "host": SENSOR_SOCKET_HOST,
    "port": SENSOR_SOCKET_PORT,
    "received": 0,
    "accepted": 0,
    "active_clients": 0,
    "errors": 0,
    "last_error": "",
}
socket_status_lock = threading.Lock()
socket_thread = None


def get_connection():
    return sqlite3.connect(DB_PATH, check_same_thread=False)


def init_db():
    with get_connection() as connection:
        connection.execute(
            f"""
            CREATE TABLE IF NOT EXISTS {TABLE_NAME} (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                cell_id TEXT NOT NULL,
                sensor_id TEXT NOT NULL,
                temperature REAL NOT NULL,
                moisture REAL NOT NULL,
                pressure REAL NOT NULL,
                light REAL NOT NULL DEFAULT 100,
                is_anomaly INTEGER NOT NULL
            )
            """
        )
        columns = {
            row[1] for row in connection.execute(f"PRAGMA table_info({TABLE_NAME})").fetchall()
        }
        if "light" not in columns:
            connection.execute(
                f"ALTER TABLE {TABLE_NAME} ADD COLUMN light REAL NOT NULL DEFAULT 100"
            )
        connection.commit()


def parse_sensor_payload(payload):
    required = ["cell_id", "sensor_id", "temperature", "moisture", "pressure"]
    missing = [field for field in required if field not in payload]
    if missing:
        raise ValueError(f"Missing fields: {', '.join(missing)}")

    timestamp = payload.get("timestamp")
    if timestamp is None:
        timestamp = datetime.utcnow().isoformat()

    normalized = {
        "timestamp": str(timestamp),
        "cell_id": str(payload["cell_id"]),
        "sensor_id": str(payload["sensor_id"]),
        "temperature": float(payload["temperature"]),
        "moisture": float(payload["moisture"]),
        "pressure": float(payload["pressure"]),
        "light": float(payload.get("light", 100.0)),
    }
    return normalized


def parse_forwarded_frame(frame_bytes, client_address):
    try:
        temperature, humidity, pressure, light = struct.unpack(
            FORWARDED_FRAME_FORMAT,
            frame_bytes,
        )
    except struct.error as exc:
        raise ValueError(f"Invalid forwarded frame: {exc}") from exc

    source_host, source_port = client_address
    normalized = {
        "timestamp": datetime.utcnow().isoformat(),
        "cell_id": f"fwd-{source_host}",
        "sensor_id": f"fwd-{source_port}",
        "temperature": float(temperature),
        "moisture": float(humidity),
        "pressure": float(pressure),
        "light": float(light),
    }
    return normalized


def set_socket_error(message):
    with socket_status_lock:
        socket_status["errors"] += 1
        socket_status["last_error"] = message


def handle_client_connection(client_socket, client_address):
    with client_socket:
        with socket_status_lock:
            socket_status["active_clients"] += 1

        buffer = b""
        while True:
            chunk = client_socket.recv(4096)
            if not chunk:
                break

            buffer += chunk

            while len(buffer) >= FORWARDED_FRAME_SIZE:
                frame = buffer[:FORWARDED_FRAME_SIZE]
                buffer = buffer[FORWARDED_FRAME_SIZE:]
                try:
                    record = parse_forwarded_frame(frame, client_address)
                    socket_queue.put(record)
                    with socket_status_lock:
                        socket_status["received"] += 1
                except ValueError as exc:
                    set_socket_error(str(exc))

        if buffer:
            set_socket_error("Incomplete forwarded frame received")

        with socket_status_lock:
            socket_status["active_clients"] = max(0, socket_status["active_clients"] - 1)


def socket_server_loop():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((SENSOR_SOCKET_HOST, SENSOR_SOCKET_PORT))
            server_socket.listen(32)
            with socket_status_lock:
                socket_status["running"] = True

            while True:
                client_socket, client_address = server_socket.accept()
                with socket_status_lock:
                    socket_status["accepted"] += 1
                worker = threading.Thread(
                    target=handle_client_connection,
                    args=(client_socket, client_address),
                    daemon=True,
                )
                worker.start()
    except OSError as exc:
        set_socket_error(str(exc))
    finally:
        with socket_status_lock:
            socket_status["running"] = False


def ensure_socket_server():
    global socket_thread
    if socket_thread is not None and socket_thread.is_alive():
        return

    socket_thread = threading.Thread(target=socket_server_loop, daemon=True)
    socket_thread.start()


def ensure_csv_header():
    if not os.path.exists(CSV_PATH):
        header = [
            "timestamp",
            "cell_id",
            "sensor_id",
            "temperature",
            "moisture",
            "pressure",
            "light",
            "is_anomaly",
        ]
        pd.DataFrame(columns=header).to_csv(CSV_PATH, index=False)


def insert_record(record):
    with get_connection() as connection:
        connection.execute(
            f"""
            INSERT INTO {TABLE_NAME} (
                timestamp,
                cell_id,
                sensor_id,
                temperature,
                moisture,
                pressure,
                light,
                is_anomaly
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                record["timestamp"],
                record["cell_id"],
                record["sensor_id"],
                record["temperature"],
                record["moisture"],
                record["pressure"],
                record["light"],
                record["is_anomaly"],
            ),
        )
        connection.commit()

    ensure_csv_header()
    pd.DataFrame([record]).to_csv(CSV_PATH, mode="a", header=False, index=False)


def load_data(limit=None):
    query = f"SELECT * FROM {TABLE_NAME} ORDER BY id DESC"
    if limit:
        query += f" LIMIT {int(limit)}"
    with get_connection() as connection:
        data = pd.read_sql_query(query, connection)
    if not data.empty:
        data["timestamp"] = pd.to_datetime(data["timestamp"])
    return data


def train_anomaly_model(data, contamination):
    features = data[FEATURE_COLUMNS].astype(float)
    scaler = StandardScaler()
    scaled = scaler.fit_transform(features)
    model = IsolationForest(
        n_estimators=150,
        contamination=contamination,
        random_state=42,
    )
    model.fit(scaled)
    return model, scaler


def score_sample(model, scaler, sample):
    sample_df = pd.DataFrame([sample], columns=FEATURE_COLUMNS)
    scaled = scaler.transform(sample_df)
    prediction = model.predict(scaled)[0]
    score = model.decision_function(scaled)[0]
    return prediction, score


def render_header():
    st.title(APP_TITLE)
    st.caption(
        "Demo dashboard for city-scale sensors (temperature, moisture, pressure) with "
        "Isolation Forest anomaly detection."
    )


def render_sidebar():
    st.sidebar.header("Simulation & Settings")
    st.sidebar.slider("Anomaly contamination", 0.01, 0.2, 0.05, 0.01, key="contamination")
    st.sidebar.number_input("Minimum samples for model", 10, 500, 30, key="min_samples")
    st.sidebar.markdown("---")
    st.sidebar.subheader("Sensor socket")
    with socket_status_lock:
        running = socket_status["running"]
        errors = socket_status["errors"]
        last_error = socket_status["last_error"]

    st.sidebar.code(
        f"{SENSOR_SOCKET_HOST}:{SENSOR_SOCKET_PORT}",
        language="text",
    )
    st.sidebar.write(f"Status: {'running' if running else 'not running'}")
    if errors:
        st.sidebar.write(f"Socket errors: {errors}")
    if last_error:
        st.sidebar.error(last_error)
    st.sidebar.caption("Live stream aktywny: panel danych odświeża się automatycznie.")


def render_metrics(data):
    st.subheader("Live status")
    if data.empty:
        st.info("No sensor data stored yet.")
        return

    latest = data.sort_values("timestamp").iloc[-1]
    col1, col2, col3, col4, col5 = st.columns(5)
    col1.metric("Latest Temperature", f"{latest['temperature']:.2f} °C")
    col2.metric("Latest Moisture", f"{latest['moisture']:.2f} %")
    col3.metric("Latest Pressure", f"{latest['pressure']:.2f} hPa")
    col4.metric("Latest Light", f"{latest['light']:.1f} lx")
    col5.metric("Latest Anomaly", "YES" if latest["is_anomaly"] else "NO")


def evaluate_ragnarok_signals(data):
    if data.empty:
        return []

    sorted_data = data.sort_values("timestamp")
    latest = sorted_data.iloc[-1]
    reasons = []

    same_sensor = sorted_data[
        (sorted_data["cell_id"] == latest["cell_id"])
        & (sorted_data["sensor_id"] == latest["sensor_id"])
    ]
    previous = same_sensor.iloc[-2] if len(same_sensor) >= 2 else None

    latest_temp = float(latest["temperature"])
    latest_light = float(latest["light"])
    latest_pressure = float(latest["pressure"])
    latest_moisture = float(latest["moisture"])

    if latest_temp <= 2.0:
        reasons.append("temperatura jest krytycznie niska")

    if previous is not None:
        previous_temp = float(previous["temperature"])
        previous_light = float(previous["light"])
        previous_pressure = float(previous["pressure"])

        if latest_temp - previous_temp <= -5.0:
            reasons.append("temperatura gwałtownie spada")

        if previous_light > 0 and latest_light <= previous_light * 0.6:
            reasons.append("poziom światła gwałtownie spada")

        pressure_drop = previous_pressure - latest_pressure
        if latest_moisture >= 78.0 and (latest_pressure <= 995.0 or pressure_drop >= 8.0):
            reasons.append("ciśnienie i wilgotność wskazują nadejście burzy")
    else:
        if latest_moisture >= 82.0 and latest_pressure <= 995.0:
            reasons.append("ciśnienie i wilgotność wskazują nadejście burzy")

    return reasons


def render_alert(data):
    if data.empty:
        return
    latest = data.sort_values("timestamp").iloc[-1]
    reasons = evaluate_ragnarok_signals(data)
    if int(latest["is_anomaly"]) == 1 or reasons:
        detail = "; ".join(reasons) if reasons else "anomalia z Isolation Forest"
        st.error(f"🚨 ragnarok nadszedl — {detail}")
    else:
        st.success("Stabilnie: brak anomalii w ostatnim odczycie.")


def render_charts(data):
    if data.empty:
        return
    chart_data = data.sort_values("timestamp").set_index("timestamp")
    _, center, _ = st.columns([1, 3, 1])
    with center:
        st.subheader("Live sensor chart")
        st.line_chart(chart_data[["temperature", "moisture", "pressure", "light"]])


def process_socket_queue(contamination, min_samples):
    pending_records = []
    while True:
        try:
            pending_records.append(socket_queue.get_nowait())
        except queue.Empty:
            break

    if not pending_records:
        return 0

    existing = load_data()
    model = None
    scaler = None
    if len(existing) >= min_samples:
        model, scaler = train_anomaly_model(existing, contamination)

    for record in pending_records:
        if model is not None and scaler is not None:
            sample = [
                record["temperature"],
                record["moisture"],
                record["pressure"],
                record["light"],
            ]
            prediction, _ = score_sample(model, scaler, sample)
            record["is_anomaly"] = 1 if prediction == -1 else 0
        else:
            record["is_anomaly"] = 0
        insert_record(record)

    return len(pending_records)


@st.fragment(run_every="1s")
def render_live_stream_panel():
    contamination = float(st.session_state.get("contamination", 0.05))
    min_samples = int(st.session_state.get("min_samples", 30))

    process_socket_queue(contamination, min_samples)
    data = load_data(limit=500)

    render_metrics(data)
    render_alert(data)

    st.subheader("Sensor trends")
    render_charts(data)


def main():
    st.set_page_config(page_title=APP_TITLE, layout="wide")
    init_db()
    ensure_socket_server()

    render_header()
    render_sidebar()

    render_live_stream_panel()


if __name__ == "__main__":
    main()
