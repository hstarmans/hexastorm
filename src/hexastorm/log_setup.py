import logging
import sys


class ColoredFormatter(logging.Formatter):
    """Custom formatter to add ANSI colors to terminal logs."""

    # ANSI color codes
    grey = "\x1b[38;20m"
    blue = "\x1b[36;20m"
    green = "\x1b[32;20m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"

    # Helper function to build the format string with color strictly on the level name
    def _get_format(self, color_code):
        # %(name)-22s left-aligns the file name to 22 characters
        # %(levelname)-8s left-aligns the INFO/DEBUG text to 8 characters
        return f"%(asctime)s - %(module)-12s - {color_code}%(levelname)-8s{self.reset} - %(message)s"

    def __init__(self):
        super().__init__()
        self.FORMATS = {
            logging.DEBUG: self._get_format(self.blue),
            logging.INFO: self._get_format(self.green),
            logging.WARNING: self._get_format(self.yellow),
            logging.ERROR: self._get_format(self.red),
            logging.CRITICAL: self._get_format(self.bold_red),
        }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        # Using datefmt here to match your preferred timestamp style
        formatter = logging.Formatter(log_fmt, datefmt="%H:%M:%S")
        return formatter.format(record)


def configure_logging(level=logging.INFO):
    """
    Sets up the global logging configuration with colors.
    Should only be called once at the application's entry point.
    """
    logger = logging.getLogger()

    # Check if handlers already exist to prevent duplicate logs
    if not logger.hasHandlers():
        logger.setLevel(level)

        # Create console handler and set the custom colored formatter
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(level)
        ch.setFormatter(ColoredFormatter())

        logger.addHandler(ch)

    # Suppress chatty third-party libraries
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    logging.getLogger("numba").setLevel(logging.WARNING)
    logging.getLogger("PIL").setLevel(logging.WARNING)
