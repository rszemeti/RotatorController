import argparse
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Repeatedly send a test string to a serial port and optionally print replies."
    )
    parser.add_argument("--port", default="COM7", help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between sends")
    parser.add_argument(
        "--message",
        default="PING",
        help="Base message text. Counter is appended automatically.",
    )
    parser.add_argument(
        "--no-read",
        action="store_true",
        help="Disable reading and printing incoming serial data.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    with serial.Serial(port=args.port, baudrate=args.baud, timeout=0.2) as ser:
        print(f"Opened {args.port} @ {args.baud} baud")
        print("Press Ctrl+C to stop.")
        counter = 0
        while True:
            message = f"{args.message} {counter}"
            payload = (message + "\n").encode("ascii", errors="replace")
            ser.write(payload)
            ser.flush()
            print(f">> {message}")

            if not args.no_read:
                # Read whatever the device returns during the interval.
                deadline = time.time() + args.interval
                while time.time() < deadline:
                    if ser.in_waiting:
                        data = ser.read(ser.in_waiting)
                        text = data.decode("ascii", errors="replace").strip()
                        if text:
                            print(f"<< {text}")
                    time.sleep(0.05)
            else:
                time.sleep(args.interval)

            counter += 1

if __name__ == "__main__":
    main()
