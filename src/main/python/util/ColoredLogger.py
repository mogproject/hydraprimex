import sys
import logging
import typing

__all__ = ['get_logger', 'ColoredLogger', 'LOG_LEVELS']

# Custom levels.
TRACE = 5
SUCCESS = 60
NONE = 100

LOG_LEVELS = {
    'success': SUCCESS,
    'crit': logging.CRITICAL,
    'error': logging.ERROR,
    'warn': logging.WARNING,
    'info': logging.INFO,
    'debug': logging.DEBUG,
    'trace': TRACE,
    'none': NONE,
    'all': TRACE
}


# Color definitions.
# ANSI ECMA-48 color codes (semicolon-separated):
# bold=1, half-bright=2, underscore=4, doubly-underlined=21, normal-intensity=22
# foreground:30+color, background:40+color
# colors: black=0, red=1, green=2, brown=3, blue=4, magenta=5, cyan=6, white=7
def NO_CHANGE(s): return s
def RED(s): return f'\x1b[31m{s}\x1b[0m'
def BOLD_RED(s): return f'\x1b[1;31m{s}\x1b[0m'
def BOLD_GREEN(s): return f'\x1b[1;32m{s}\x1b[0m'
def YELLOW(s): return f'\x1b[33m{s}\x1b[0m'
def GRAY(s): return f'\x1b[2;37m{s}\x1b[0m'
def CYAN(s): return f'\x1b[36m{s}\x1b[0m'


LOG_COLORS = {
    TRACE: CYAN,
    logging.DEBUG: GRAY,
    logging.INFO: NO_CHANGE,
    logging.WARNING: YELLOW,
    logging.ERROR: RED,
    logging.CRITICAL: BOLD_RED,
    SUCCESS: BOLD_GREEN,
}


class ColoredFormatter(logging.Formatter):
    def __init__(
            self,
            colored: bool = True,
            fmt: str | None = None,
            datefmt: str | None = None,
            style: typing.Literal['%', '{', '$'] = '%',
            validate: bool = True,
            defaults: dict[str, typing.Any] | None = None
    ) -> None:
        super().__init__(fmt, datefmt, style, validate, defaults=defaults)
        self.fmt = fmt
        self.formats = {k: v(fmt) for k, v in LOG_COLORS.items()} if colored else {}

    def format(self, record):
        fmt = self.formats.get(record.levelno, self.fmt)
        formatter = logging.Formatter(fmt)
        return formatter.format(record)


class ColoredLogger(logging.Logger):
    def __init__(self, name: str = '', level: int = 0, colored: bool = True, stream=sys.stderr) -> None:
        super().__init__(name, level)

        fmt = '%(asctime)s [%(levelname)-8s] %(message)s'
        handler = logging.StreamHandler(stream)
        handler.setFormatter(ColoredFormatter(colored, fmt))
        self.addHandler(handler)

    def trace(self, message, *args, **kws):
        if self.isEnabledFor(TRACE):
            self._log(TRACE, message, args, **kws)

    def success(self, message, *args, **kws):
        if self.isEnabledFor(SUCCESS):
            self._log(SUCCESS, message, args, **kws)


def get_logger(name: str = '', log_level=NONE, colored=True, stream=sys.stderr):
    """Logger settings."""

    # add custom levels
    logging.addLevelName(TRACE, 'TRACE')
    logging.addLevelName(SUCCESS, 'SUCCESS')

    # get logger
    return ColoredLogger(name, log_level, colored, stream)
