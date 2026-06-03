import argparse
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send TEST n to a serial port and print any received data."
    )
    parser.add_argument("--port", default="COM7", help="Serial port (default: COM7)")
    parser.add_argument("--baud", type=int, default=19200, help="Baud rate (default: 19200)")
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between test transmissions (default: 1.0)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"Loopback testing {args.port} @ {args.baud}. Press Ctrl+C to stop.")
        counter = 1
        try:
            while True:
                message = f"TEST {counter}\n"
                ser.write(message.encode("ascii"))
                ser.flush()
                print(f"TX: {message.strip()}")

                deadline = time.time() + args.interval
                while time.time() < deadline:
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        print(data.decode("ascii", errors="replace"), end="", flush=True)
                    time.sleep(0.05)

                counter += 1
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
