"""Compatibility entry point for the offline model viewer. The main app is python -m luxoiii."""
from .web import Handler, main as run_app


def main():
    # Existing simulation commands stay offline unless an observer is explicitly supplied.
    run_app(observer_default="")


if __name__ == "__main__":
    main()
