"""Minimal integration example for an existing K230D ball detector.

Copy the initialization block into the start of your current CanMV program,
then call ``publish_ball_detection`` once per camera frame.  The existing ball
detection code remains responsible only for ``ball_center_x`` and ``valid``.

The example UART/FPIOA pin numbers are common choices, not facts recovered from
the empty workspace.  Set UART_TX_IO and UART_RX_IO to pins that are actually
free on your K230D carrier board.
"""

from machine import FPIOA, UART

from pixel_error_sender import PixelErrorSender


# ---------------------------- USER CONFIGURATION ---------------------------
UART_TX_IO = 11
UART_RX_IO = 12
UART_ID = UART.UART2
UART_BAUD = 115200

# If the camera/control ROI is 320 pixels wide, the target is normally 160.
# Passing image_width // 2 to publish_ball_detection is safer than hard-coding.
# ---------------------------------------------------------------------------


def create_motor_sender():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_IO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_IO, FPIOA.UART2_RXD)

    uart = UART(
        UART_ID,
        baudrate=UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    return PixelErrorSender(uart, minimum_interval_ms=10)


motor_sender = create_motor_sender()


def publish_ball_detection(ball_center_x, image_width, valid=True):
    """Call once after processing each new camera frame.

    Args:
        ball_center_x: detected ball-center x coordinate in pixels.
        image_width: width of the image/ROI that defines the target center.
        valid: False when this frame has no trustworthy ball detection. The
            most recent valid coordinate is automatically retransmitted.

    Returns:
        For a valid frame, the newly measured signed pixel error. For a lost
        frame after at least one valid detection, the held last error. Returns
        None only when no valid ball coordinate has ever been seen.
    """

    if not valid:
        motor_sender.ball_lost()
        return motor_sender.last_error if motor_sender.has_valid_error else None

    screen_center_x = image_width // 2
    error_px = int(ball_center_x) - screen_center_x
    motor_sender.send(error_px)
    return error_px


# Insert this into your existing vision loop:
#
# while True:
#     img = sensor.snapshot()
#     ball = detect_ball(img)            # your existing detector
#
#     if ball is not None:
#         error_px = publish_ball_detection(
#             ball_center_x=ball.cx(),
#             image_width=img.width(),
#             valid=True,
#         )
#         print(error_px)                 # should converge toward 0
#     else:
#         # Keeps using the last valid error until the next detection.
#         publish_ball_detection(0, img.width(), valid=False)
#
#     Display.show_image(img)
