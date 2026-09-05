"""Application entry point: logging, config, GUI startup."""
import argparse
import logging
import os
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Real-time bionic hand controller (webcam -> servos)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="enable debug logging")
    parser.add_argument("--config", default=None,
                        help="path to config JSON (default: ./config.json)")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s [%(threadName)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    # Quiet the TensorFlow/absl noise MediaPipe produces on import.
    os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

    from .config_manager import ConfigManager
    cfg_mgr = ConfigManager(args.config)
    cfg_mgr.load()

    from .gui import BionicHandApp
    app = BionicHandApp(cfg_mgr)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        app.on_quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
