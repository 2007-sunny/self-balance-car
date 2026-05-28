import argparse
import csv
import datetime as dt
import os
import queue
import signal
import threading
import time
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read self_balance_v2 Bluetooth telemetry and plot it live."
    )
    parser.add_argument("--port", help="Bluetooth COM port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=9600, help="HC-05 baud rate")
    parser.add_argument("--window", type=float, default=20.0, help="plot window seconds")
    parser.add_argument("--log-dir", default="logs", help="folder for CSV logs")
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="list serial ports and exit",
    )
    parser.add_argument(
        "--no-start-command",
        action="store_true",
        help="do not send LOG=1 automatically",
    )
    return parser.parse_args()


def make_log_writer(log_dir):
    os.makedirs(log_dir, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(log_dir, f"telemetry_{stamp}.csv")
    fp = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(fp)
    writer.writerow(
        [
            "pc_time",
            "t_ms",
            "angle_deg",
            "gyro_dps",
            "motor_cmd_sps",
            "motor_actual_sps",
            "move_deg",
            "fallen",
        ]
    )
    return path, fp, writer


def reader_thread(ser, out_q, stop_event):
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            out_q.put(("error", str(exc)))
            stop_event.set()
            return

        if not raw:
            continue

        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        out_q.put(("line", line))


def decode_data_line(line):
    parts = line.split(",")
    if len(parts) != 8 or parts[0] != "D":
        return None

    t_ms = int(parts[1])
    angle_deg = int(parts[2]) / 100.0
    gyro_dps = int(parts[3]) / 10.0
    motor_cmd_sps = int(parts[4])
    motor_actual_sps = int(parts[5])
    move_deg = int(parts[6]) / 100.0
    fallen = int(parts[7])
    return (
        t_ms,
        angle_deg,
        gyro_dps,
        motor_cmd_sps,
        motor_actual_sps,
        move_deg,
        fallen,
    )


def main():
    args = parse_args()
    if args.list_ports:
        for port in serial.tools.list_ports.comports():
            print(f"{port.device}\t{port.description}")
        return
    if not args.port:
        raise SystemExit("Missing --port. Use --list-ports to find the Bluetooth COM port.")

    log_path, log_fp, writer = make_log_writer(args.log_dir)

    stop_event = threading.Event()
    q = queue.Queue()

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(2.0)
    ser.reset_input_buffer()

    if not args.no_start_command:
        ser.write(b"LOG=1\n")
        ser.flush()

    thread = threading.Thread(target=reader_thread, args=(ser, q, stop_event), daemon=True)
    thread.start()

    max_points = max(50, int(args.window * 25))
    t_data = deque(maxlen=max_points)
    angle_data = deque(maxlen=max_points)
    gyro_data = deque(maxlen=max_points)
    cmd_data = deque(maxlen=max_points)
    actual_data = deque(maxlen=max_points)
    fallen_data = deque(maxlen=max_points)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    fig.canvas.manager.set_window_title("self_balance_v2 telemetry")
    angle_line, = axes[0].plot([], [], label="angle deg")
    gyro_line, = axes[1].plot([], [], label="gyro dps")
    cmd_line, = axes[2].plot([], [], label="cmd steps/s")
    actual_line, = axes[2].plot([], [], label="actual steps/s")

    axes[0].set_ylabel("Angle")
    axes[1].set_ylabel("Gyro")
    axes[2].set_ylabel("Motor")
    axes[2].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

    status = fig.text(0.01, 0.01, f"Logging to {log_path}", fontsize=9)

    def pump_queue():
        while True:
            try:
                kind, payload = q.get_nowait()
            except queue.Empty:
                break

            if kind == "error":
                status.set_text(f"Serial error: {payload}")
                continue

            line = payload
            if line.startswith("H,") or line.startswith("OK:"):
                status.set_text(f"{line} | logging to {log_path}")
                continue

            sample = decode_data_line(line)
            if sample is None:
                status.set_text(f"Ignored: {line}")
                continue

            (
                t_ms,
                angle_deg,
                gyro_dps,
                motor_cmd_sps,
                motor_actual_sps,
                move_deg,
                fallen,
            ) = sample
            t_s = t_ms / 1000.0
            t_data.append(t_s)
            angle_data.append(angle_deg)
            gyro_data.append(gyro_dps)
            cmd_data.append(motor_cmd_sps)
            actual_data.append(motor_actual_sps)
            fallen_data.append(fallen)

            writer.writerow(
                [
                    dt.datetime.now().isoformat(timespec="milliseconds"),
                    t_ms,
                    angle_deg,
                    gyro_dps,
                    motor_cmd_sps,
                    motor_actual_sps,
                    move_deg,
                    fallen,
                ]
            )
            log_fp.flush()

    def update(_frame):
        pump_queue()
        if not t_data:
            return angle_line, gyro_line, cmd_line, actual_line

        x0 = max(0.0, t_data[-1] - args.window)
        x1 = max(args.window, t_data[-1])

        angle_line.set_data(t_data, angle_data)
        gyro_line.set_data(t_data, gyro_data)
        cmd_line.set_data(t_data, cmd_data)
        actual_line.set_data(t_data, actual_data)

        axes[0].set_xlim(x0, x1)
        for ax in axes:
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        if fallen_data and fallen_data[-1]:
            status.set_text(f"FALLEN | logging to {log_path}")

        return angle_line, gyro_line, cmd_line, actual_line

    def shutdown(*_args):
        stop_event.set()
        try:
            ser.write(b"LOG=0\n")
            ser.flush()
        except serial.SerialException:
            pass
        ser.close()
        log_fp.close()

    signal.signal(signal.SIGINT, shutdown)

    try:
        ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
        _ = ani
        plt.show()
    finally:
        shutdown()
        print(f"Saved CSV: {log_path}")


if __name__ == "__main__":
    main()
