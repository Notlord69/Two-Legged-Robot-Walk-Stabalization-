"""
Siclo1 bipedal robot simulation — CLI entry point.

Usage:
    python3 main.py                                # headless, 1000 cycles
    python3 main.py --gui                          # GUI at 10 Hz, 1000 cycles
    python3 main.py --gui --viz-hz 33              # GUI at 33 Hz
    python3 main.py --gui --duration 2000 --hold   # 2000 cycles, inspect final pose
    python3 main.py --walk 2.0                     # walk 2.0 m headless
    python3 main.py --gui --walk 2.0               # walk 2.0 m with GUI
    python3 main.py --on                           # enable terminal telemetry output
"""
import argparse
import sys
import time
import pybullet as p
import sim.interface
from HeartBeat import Siclo1Controller


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Siclo1 bipedal robot simulation — 100 Hz physics heartbeat"
    )
    parser.add_argument("--gui", action="store_true",
                        help="Enable PyBullet GUI viewer and debug visualisation")
    parser.add_argument("--viz-hz", type=int, default=10, metavar="HZ",
                        help="GUI render rate Hz, integer only (default: 10, range: 1-100)")
    parser.add_argument("--duration", type=int, default=1000, metavar="CYCLES",
                        help="Number of active HeartBeat cycles to run (default: 1000)")
    parser.add_argument("--hold", action="store_true",
                        help="Keep GUI window open after summary for pose inspection")
    parser.add_argument("--walk", type=float, default=None, metavar="METRES",
                        help="Walk forward D metres then stop (e.g. --walk 2.0)")
    parser.add_argument("--on", action="store_true",
                        help="Enable telemetry terminal output (default: off; data always written to CSV/summary)")
    return parser


def _viz_decimation(viz_hz: int) -> int:
    """Convert render rate (Hz) to loop decimation factor.

    viz_hz: desired GUI frames per second (clamped to 1-100 Hz)
    Returns: loop cycles between GUI renders (physics runs at 100 Hz)
    """
    hz = max(1, min(100, viz_hz))   # clamp: 1 Hz min, 100 Hz max
    return max(1, 100 // hz)        # cycles/render; minimum 1


def main(argv=None) -> None:
    args = _make_parser().parse_args(argv)
    decimation = _viz_decimation(args.viz_hz)
    # Reconstruct the exact invocation so summary.txt is self-describing
    _argv = argv if argv is not None else sys.argv[1:]
    argv_command = "python3 main.py " + " ".join(_argv) if _argv else "python3 main.py"
    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=decimation,
                                  walk_distance=args.walk,
                                  argv_command=argv_command,
                                  quiet=not args.on)
    try:
        controller.run(max_cycles=args.duration)
    except KeyboardInterrupt:
        print("\n[Siclo1] Interrupted.")
    finally:
        controller.finalize_telemetry()

        if args.hold:
            if controller._viz_bridge is not None and controller._viz_bridge.is_alive:
                print("[Siclo1] --hold active. Inspect final pose. Ctrl-C to exit.")
                try:
                    while controller._viz_bridge.is_alive:
                        sim.interface.step_simulation(controller.physics_client)
                        time.sleep(0.01)
                except KeyboardInterrupt:
                    print("\n[Siclo1] Hold ended.")
            else:
                print("[Siclo1] --hold ignored: no GUI active (use --gui)")

        controller.shutdown()


if __name__ == "__main__":
    main()
