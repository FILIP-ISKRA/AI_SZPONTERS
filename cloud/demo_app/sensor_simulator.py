import argparse
import random
import socket
import struct
import time
from datetime import datetime, UTC

FRAME_FORMAT = "<dddd"


def build_sensor_ids(cell_id, sensors_per_cell):
    return [f"{cell_id}-sensor-{index + 1:02d}" for index in range(sensors_per_cell)]


def make_reading(cell_id, sensor_id, anomaly_rate):
    is_anomaly = random.random() < anomaly_rate

    temperature = random.gauss(24.0, 2.2)
    moisture = random.gauss(48.0, 7.0)
    pressure = random.gauss(1012.0, 5.0)
    light = random.gauss(420.0, 120.0)
    anomaly_type = "normal"

    if is_anomaly:
        anomaly_type = random.choice(["temp_drop", "light_drop", "storm_signal", "mixed"])
        if anomaly_type == "temp_drop":
            temperature = random.uniform(-8.0, 2.0)
        elif anomaly_type == "light_drop":
            light = random.uniform(5.0, 35.0)
        elif anomaly_type == "storm_signal":
            pressure = random.uniform(984.0, 995.0)
            moisture = random.uniform(82.0, 99.0)
        else:
            temperature = random.uniform(-6.0, 4.0)
            light = random.uniform(4.0, 28.0)
            pressure = random.uniform(982.0, 994.0)
            moisture = random.uniform(85.0, 99.0)

    return {
        "timestamp": datetime.now(UTC).isoformat(),
        "cell_id": cell_id,
        "sensor_id": sensor_id,
        "temperature": round(temperature, 2),
        "humidity": round(moisture, 2),
        "pressure": round(pressure, 2),
        "light": round(light, 2),
    }, is_anomaly, anomaly_type


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send simulated city sensor readings to the demo TCP socket server."
    )
    parser.add_argument("--host", default="10.42.0.1", help="Socket host")
    parser.add_argument("--port", type=int, default=19001, help="Socket port")
    parser.add_argument("--cells", type=int, default=6, help="Number of city cells")
    parser.add_argument(
        "--sensors-per-cell",
        type=int,
        default=3,
        help="Number of sensors to simulate per cell",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.8,
        help="Seconds between sent readings",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=200,
        help="How many readings to send (use 0 for infinite)",
    )
    parser.add_argument(
        "--anomaly-rate",
        type=float,
        default=0.08,
        help="Probability of generating an anomalous synthetic reading",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.cells <= 0 or args.sensors_per_cell <= 0:
        raise ValueError("--cells and --sensors-per-cell must be positive")
    if args.interval < 0:
        raise ValueError("--interval cannot be negative")
    if not 0.0 <= args.anomaly_rate <= 1.0:
        raise ValueError("--anomaly-rate must be between 0 and 1")

    cells = [f"cell-{index + 1:03d}" for index in range(args.cells)]
    sensors_by_cell = {
        cell_id: build_sensor_ids(cell_id, args.sensors_per_cell) for cell_id in cells
    }

    print(f"Connecting to {args.host}:{args.port} ...")
    with socket.create_connection((args.host, args.port), timeout=10) as sock:
        sent = 0
        synthetic_anomalies = 0

        while args.count == 0 or sent < args.count:
            cell_id = random.choice(cells)
            sensor_id = random.choice(sensors_by_cell[cell_id])
            payload, marked_anomaly, anomaly_type = make_reading(
                cell_id,
                sensor_id,
                args.anomaly_rate,
            )
            if marked_anomaly:
                synthetic_anomalies += 1

            packet = struct.pack(
                FRAME_FORMAT,
                float(payload["temperature"]),
                float(payload["humidity"]),
                float(payload["pressure"]),
                float(payload["light"]),
            )
            sock.sendall(packet)
            sent += 1

            summary = (
                f"sent={sent} cell={cell_id} sensor={sensor_id} "
                f"temp={payload['temperature']} humidity={payload['humidity']} "
                f"pressure={payload['pressure']} light={payload['light']} type={anomaly_type}"
            )
            print(summary)

            if args.interval > 0:
                time.sleep(args.interval)

    print(
        f"Completed. Sent {sent} readings. Synthetic anomaly injections: {synthetic_anomalies}."
    )


if __name__ == "__main__":
    main()
