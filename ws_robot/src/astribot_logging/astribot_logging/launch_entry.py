"""Configure captured launch output before ros2cli creates its first file handler."""
from .output import configure_launch_logging


def main():
    configure_launch_logging()
    from ros2cli.cli import main as ros2_main
    return ros2_main()


if __name__ == '__main__':
    raise SystemExit(main())
