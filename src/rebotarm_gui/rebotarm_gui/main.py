"""Console entry point for the reBotArm GUI."""

from __future__ import annotations

import argparse
import sys


def main(argv: list[str] | None = None) -> int:
    """Entry point used by setup.py (console script)."""
    parser = argparse.ArgumentParser(
        prog="rebotarm_gui",
        description="Interface gráfica ROS 2 para o braço reBotArm.",
    )
    parser.add_argument(
        "--namespace",
        default="rebotarm",
        help="Namespace do braço (default: rebotarm)",
    )
    parser.add_argument(
        "--usability-test",
        action="store_true",
        help="Abre o roteiro cronometrado para avaliação moderada com operadores.",
    )
    args, _unknown = parser.parse_known_args(argv)

    # Import here so argparse --help works even if Qt is not installed yet.
    try:
        from .app import run
    except ImportError as exc:
        print(
            f"Erro ao importar a GUI: {exc}\n"
            "Verifique se o PySide6 está instalado:  pip install PySide6",
            file=sys.stderr,
        )
        return 2

    return run(namespace=args.namespace, usability_test=args.usability_test)


if __name__ == "__main__":
    raise SystemExit(main())
