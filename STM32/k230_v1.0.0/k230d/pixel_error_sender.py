"""K230D -> C06B pixel-error sender.

The K230D does not run the motor PID.  It only sends the signed difference
between the detected ball center and the target screen center:

    error_px = ball_center_x - screen_center_x

Each camera frame is one ASCII integer followed by a newline.  When a frame
temporarily loses the ball, call ``ball_lost()``: it retransmits the most recent
valid pixel error until the next successful detection.  If the whole K230D
program stalls or the UART cable is disconnected, transmission stops and the
C06B watchdog still returns the motor smoothly to zero.

This module deliberately accepts an already-created UART object so it does not
hard-code board-specific FPIOA pins.
"""

try:
    import time
except ImportError:  # pragma: no cover - all supported runtimes have time
    time = None


def _ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.monotonic() * 1000)


def _ticks_diff(new_value, old_value):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(new_value, old_value)
    return new_value - old_value


class PixelErrorSender:
    """Non-blocking, rate-limited sender for signed pixel error values."""

    def __init__(self, uart, minimum_interval_ms=10, maximum_abs_error=2000):
        self.uart = uart
        self.minimum_interval_ms = int(minimum_interval_ms)
        self.maximum_abs_error = int(maximum_abs_error)
        self._last_send_ms = None
        self._last_error = 0
        self._have_valid_error = False

    def send(self, error_px, now_ms=None):
        """Send one valid signed pixel error.

        Returns True when a line was written and False when the configured
        minimum interval has not elapsed.  Even when rate-limited, the newest
        valid coordinate is remembered immediately, so a following lost frame
        repeats that newest observation instead of an older transmitted one.
        The C06B accepts 25--50 Hz camera updates; sending faster than every
        10 ms is unnecessary.
        """

        if now_ms is None:
            now_ms = _ticks_ms()

        value = int(round(error_px))
        if value > self.maximum_abs_error:
            value = self.maximum_abs_error
        elif value < -self.maximum_abs_error:
            value = -self.maximum_abs_error

        # Detection state and UART rate limiting are intentionally separate.
        # A valid frame always replaces the held coordinate, even if this
        # particular frame arrives too soon to write another UART line.
        self._last_error = value
        self._have_valid_error = True

        if self._last_send_ms is not None:
            elapsed = _ticks_diff(now_ms, self._last_send_ms)
            if elapsed < self.minimum_interval_ms:
                return False

        self.uart.write(("%d\n" % value).encode())
        self._last_send_ms = now_ms
        return True

    def send_from_centers(self, ball_center_x, screen_center_x, now_ms=None):
        """Convenience wrapper for the exact value required by the controller."""

        return self.send(ball_center_x - screen_center_x, now_ms=now_ms)

    def ball_lost(self, now_ms=None):
        """Retransmit the last valid error for a temporarily lost frame.

        Before the first valid detection this method sends nothing.  Once a
        valid coordinate exists, every lost frame keeps publishing that last
        coordinate.  A full K230D/UART failure is still distinguishable because
        no code runs to send the repeated value, so the C06B watchdog expires.
        """

        if not self._have_valid_error:
            return False
        return self.send(self._last_error, now_ms=now_ms)

    @property
    def last_error(self):
        return self._last_error

    @property
    def has_valid_error(self):
        return self._have_valid_error
