import argparse
import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simple serial monitor that prints incoming bytes as text."
    )
    parser.add_argument("--port", default="COM4", help="Serial port (default: COM4)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        print(f"Monitoring {args.port} @ {args.baud}. Press Ctrl+C to stop.")
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    print(data.decode("ascii", errors="replace"), end="", flush=True)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
