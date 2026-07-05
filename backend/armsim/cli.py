"""Optional CLI entry point: `python -m armsim run config.yaml`."""

from __future__ import annotations

import argparse

import armsim


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Robotic Arm Simulator — start a config-defined arm and serve the frontend.",
    )
    sub = parser.add_subparsers(dest="command")

    run_parser = sub.add_parser("run", help="Load a config and start the server")
    run_parser.add_argument("config", help="Path to YAML/JSON arm config file")
    run_parser.add_argument("--mode", choices=["local", "lan"], default="local")
    run_parser.add_argument("--port", type=int, default=8080)
    run_parser.add_argument("--record-to", default=None, help="Trajectory output path")

    args = parser.parse_args()

    if args.command == "run":
        arm = armsim.load_arm_from_config(args.config)
        print(f"Loaded arm '{args.config}' with {len(arm.joints)} joints.")
        print(f"Starting server on port {args.port} (mode={args.mode})...")
        arm.start(mode=args.mode, port=args.port, record_to=args.record_to)
        print("Server running. Press Ctrl+C to stop.")
        try:
            import time
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            arm.stop_server()
        print("Server stopped.")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
