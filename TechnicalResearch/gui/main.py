"""Pinky Robot GUI — entry point."""

import sys

from PyQt5.QtWidgets import QApplication

from app.styles import dark_palette
from app.window import MainWindow


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setPalette(dark_palette())

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
