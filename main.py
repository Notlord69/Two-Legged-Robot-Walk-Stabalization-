"""
Siclo1 bipedal robot simulation — CLI entry point.

Usage:
    python3 main.py                    # headless, 10 Hz GUI render rate (default)
    python3 main.py --gui              # enable PyBullet GUI at 10 Hz
    python3 main.py --gui --viz-hz 33  # GUI at 33 Hz render rate
"""
import argparse
from HeartBeat import Siclo1Controller


def _make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Siclo1 bipedal robot simulation — 100 Hz physics heartbeat"
    )
    parser.add_argument("--gui", action="store_true",
                        help="Enable PyBullet GUI viewer and debug visualisation")
    parser.add_argument("--viz-hz", type=int, default=10, metavar="HZ",
                        help="GUI render rate Hz, integer only (default: 10, range: 1-100)")
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
    controller = Siclo1Controller(use_gui=args.gui, viz_decimation=decimation)
    try:
        controller.run(max_cycles=1000)
    except KeyboardInterrupt:
        print("\n[Siclo1] Interrupted.")
    finally:
        controller.finalize_telemetry()
        controller.shutdown()


if __name__ == "__main__":
    main()
