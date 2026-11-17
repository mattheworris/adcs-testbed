#!/usr/bin/env python3
"""Ground station: live plots of adcs-fsw telemetry (UDP, CSV lines).

    python3 ground.py                 # listen on 0.0.0.0:5005, plot live
    python3 ground.py --log run.csv   # also save every line

Send commands from another terminal:
    echo "point-deg 90" | nc -u -w0 <bbb-ip> 5006
    echo "damp"         | nc -u -w0 <bbb-ip> 5006
"""
import argparse
import collections
import math
import socket

import matplotlib.animation as animation
import matplotlib.pyplot as plt

FIELDS = [
    "t", "theta", "rate", "bias", "sigma", "target", "wheel_rpm", "torque",
    "st_status", "wheel_status", "truth_theta", "truth_omega", "accel_x",
    "frames_ok", "crc_errors", "bad_frames", "mode",
]
WINDOW = 600  # samples kept on screen (60 s at 10 Hz)


def parse(line):
    parts = line.strip().split(",")
    if len(parts) != len(FIELDS):
        return None
    row = dict(zip(FIELDS, parts))
    for k in FIELDS[:-1]:
        row[k] = float(row[k])
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--log")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.setblocking(False)
    log = open(args.log, "w") if args.log else None
    if log:
        log.write(",".join(FIELDS) + "\n")

    buf = collections.deque(maxlen=WINDOW)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig.canvas.manager.set_window_title("ADCS ground station")

    def update(_):
        while True:
            try:
                data, _addr = sock.recvfrom(1024)
            except BlockingIOError:
                break
            line = data.decode(errors="replace")
            row = parse(line)
            if row:
                buf.append(row)
                if log:
                    log.write(line.strip() + "\n")
        if not buf:
            return
        t = [r["t"] for r in buf]
        deg = math.degrees
        for ax in (ax1, ax2, ax3):
            ax.clear()
            ax.grid(True, alpha=0.3)

        ax1.plot(t, [deg(r["truth_theta"]) for r in buf], label="truth", lw=3, alpha=0.4)
        ax1.plot(t, [deg(r["theta"]) for r in buf], label="estimate")
        ax1.plot(t, [deg(r["target"]) for r in buf], "--", label="target")
        # Shade star tracker outages.
        for r in buf:
            if r["st_status"] != 0:
                ax1.axvspan(r["t"] - 0.05, r["t"] + 0.05, color="orange", alpha=0.15, lw=0)
        ax1.set_ylabel("angle [deg]")
        ax1.legend(loc="upper left")
        last = buf[-1]
        ax1.set_title(
            f"mode {last['mode']}  bias {deg(last['bias']):.3f} deg/s  "
            f"frames {int(last['frames_ok'])}  crc err {int(last['crc_errors'])}"
        )

        def err_arcsec(r):
            e = (r["theta"] - r["truth_theta"] + math.pi) % (2 * math.pi) - math.pi
            return deg(e) * 3600

        ax2.plot(t, [err_arcsec(r) for r in buf], label="estimate - truth")
        ax2.plot(t, [3 * deg(r["sigma"]) * 3600 for r in buf], "k:", label="+3 sigma")
        ax2.plot(t, [-3 * deg(r["sigma"]) * 3600 for r in buf], "k:")
        ax2.set_ylabel("error [arcsec]")
        ax2.legend(loc="upper left")

        ax3.plot(t, [r["wheel_rpm"] for r in buf], label="wheel speed")
        ax3.axhline(6000, color="r", ls=":")
        ax3.axhline(-6000, color="r", ls=":")
        ax3.set_ylabel("wheel [rpm]")
        ax3.set_xlabel("time [s]")

    _anim = animation.FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
