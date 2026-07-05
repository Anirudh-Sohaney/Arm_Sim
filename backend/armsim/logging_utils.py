"""Logging setup for the armsim package.

Provides a module-level logger per submodule with consistent format.
Default level is WARNING so normal operation is silent; a script can
raise verbosity via ``armsim.set_log_level(logging.DEBUG)``.
"""

from __future__ import annotations

import logging
import sys

_log_format = logging.Formatter(
    fmt="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)

_package_logger = logging.getLogger("armsim")
_package_logger.setLevel(logging.WARNING)

# Prevent duplicate handlers if set_log_level is called more than once.
if not _package_logger.handlers:
    _handler = logging.StreamHandler(sys.stderr)
    _handler.setFormatter(_log_format)
    _package_logger.addHandler(_handler)


def set_log_level(level: int) -> None:
    """Set the log level for the entire ``armsim`` package.

    Parameters
    ----------
    level : int
        A standard logging level, e.g. ``logging.DEBUG`` or ``logging.INFO``.
    """
    _package_logger.setLevel(level)
    for h in _package_logger.handlers:
        h.setLevel(level)


def get_logger(name: str) -> logging.Logger:
    """Return a child logger for the given submodule name.

    Parameters
    ----------
    name : str
        Typically ``__name__`` from the calling module.

    Returns
    -------
    logging.Logger
    """
    return logging.getLogger(name)
