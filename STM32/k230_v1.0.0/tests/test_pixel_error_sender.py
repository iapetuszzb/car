import importlib.util
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "k230d" / "pixel_error_sender.py"
SPEC = importlib.util.spec_from_file_location("pixel_error_sender", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeUart:
    def __init__(self):
        self.writes = []

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)


class PixelErrorSenderTests(unittest.TestCase):
    def test_signed_ascii_protocol(self):
        uart = FakeUart()
        sender = MODULE.PixelErrorSender(uart, minimum_interval_ms=10)

        self.assertTrue(sender.send(-37, now_ms=100))
        self.assertEqual(uart.writes, [b"-37\n"])

        self.assertFalse(sender.send(22, now_ms=105))
        self.assertEqual(len(uart.writes), 1)

        self.assertTrue(sender.send(22, now_ms=110))
        self.assertEqual(uart.writes[-1], b"22\n")

    def test_center_and_clamp(self):
        uart = FakeUart()
        sender = MODULE.PixelErrorSender(
            uart, minimum_interval_ms=0, maximum_abs_error=2000
        )

        sender.send_from_centers(160, 160, now_ms=0)
        sender.send(99999, now_ms=1)
        sender.send(-99999, now_ms=2)

        self.assertEqual(uart.writes, [b"0\n", b"2000\n", b"-2000\n"])

    def test_ball_lost_holds_last_valid_error(self):
        uart = FakeUart()
        sender = MODULE.PixelErrorSender(uart, minimum_interval_ms=0)

        # No coordinate exists before the first successful detection.
        self.assertFalse(sender.ball_lost(now_ms=0))
        self.assertEqual(uart.writes, [])

        sender.send(-18, now_ms=1)
        self.assertTrue(sender.ball_lost(now_ms=2))
        self.assertTrue(sender.ball_lost(now_ms=3))
        self.assertEqual(uart.writes, [b"-18\n", b"-18\n", b"-18\n"])

        # A new valid coordinate immediately replaces the held value.
        sender.send(7, now_ms=4)
        sender.ball_lost(now_ms=5)
        self.assertEqual(uart.writes[-2:], [b"7\n", b"7\n"])

    def test_rate_limited_detection_still_replaces_held_coordinate(self):
        uart = FakeUart()
        sender = MODULE.PixelErrorSender(uart, minimum_interval_ms=10)

        self.assertTrue(sender.send(-18, now_ms=100))

        # This valid frame is too early for another UART write, but it is still
        # the newest recognized ball coordinate and must replace the held one.
        self.assertFalse(sender.send(7, now_ms=105))
        self.assertEqual(sender.last_error, 7)
        self.assertEqual(uart.writes, [b"-18\n"])

        # If the next frame loses the ball, the sender publishes 7, not -18.
        self.assertTrue(sender.ball_lost(now_ms=110))
        self.assertEqual(uart.writes, [b"-18\n", b"7\n"])


if __name__ == "__main__":
    unittest.main()
