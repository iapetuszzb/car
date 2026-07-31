# K230D BOX PORT2 UART loopback test.
#
# Hardware connection:
#   PORT2 TX / IO44 ---- PORT2 RX / IO45
#
# Use only one jumper wire between TX and RX. Do not connect a power pin.

import gc
import os
import sys
import time
import image

from machine import FPIOA, UART
from media.display import *
from media.media import *


DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

UART_TX_IO = 44
UART_RX_IO = 45
UART_BAUD = 115200

TX_INTERVAL_MS = 500
LCD_INTERVAL_MS = 500
LOOP_DELAY_MS = 5
RX_BUFFER_LIMIT = 1024
EXPECTED_HISTORY_LIMIT = 16


def ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.time() * 1000)


def ticks_diff(now, old):
    try:
        return time.ticks_diff(now, old)
    except AttributeError:
        return now - old


def decode_ascii(data):
    try:
        return data.decode("ascii")
    except Exception:
        try:
            return data.decode()
        except Exception:
            return repr(data)


def draw_text(img, x, y, text, color=(255, 255, 255), large=False):
    font_size = 32 if large else 24
    img.draw_string_advanced(
        int(x), int(y), font_size, str(text), color=color
    )


def draw_status(img, stats):
    img.clear()

    draw_text(img, 24, 18, "K230D PORT2 LOOPBACK", (80, 220, 255), True)
    draw_text(img, 24, 72, "UART2  115200  8N1")
    draw_text(img, 24, 106, "TX IO44  <->  RX IO45", (255, 220, 80))

    draw_text(img, 24, 158, "TX frames: %d" % stats["tx_frames"], (180, 255, 180))
    draw_text(img, 330, 158, "TX bytes: %d" % stats["tx_bytes"], (180, 255, 180))
    draw_text(img, 24, 198, "RX lines : %d" % stats["rx_lines"], (180, 220, 255))
    draw_text(img, 330, 198, "RX bytes: %d" % stats["rx_bytes"], (180, 220, 255))
    draw_text(img, 24, 238, "MATCH    : %d" % stats["matched"], (120, 255, 120))
    draw_text(img, 330, 238, "ERRORS  : %d" % stats["errors"], (255, 150, 150))

    if stats["matched"] > 0:
        status = "LOOPBACK OK"
        status_color = (80, 255, 100)
    elif stats["rx_bytes"] > 0:
        status = "RX DATA, WAIT MATCH"
        status_color = (255, 210, 80)
    else:
        status = "WAITING FOR RX"
        status_color = (255, 210, 80)

    draw_text(img, 24, 290, "STATUS: " + status, status_color)
    draw_text(img, 24, 340, "LAST TX:", (160, 160, 160))
    draw_text(img, 24, 372, stats["last_tx"][-38:])
    draw_text(img, 24, 412, "LAST RX:", (160, 160, 160))
    draw_text(img, 24, 444, stats["last_rx"][-38:])


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_IO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_IO, FPIOA.UART2_RXD)

    uart = UART(
        UART.UART2,
        baudrate=UART_BAUD,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    return fpioa, uart


def main():
    uart = None
    fpioa = None
    screen = None
    display_ready = False
    media_ready = False

    stats = {
        "tx_frames": 0,
        "tx_bytes": 0,
        "rx_lines": 0,
        "rx_bytes": 0,
        "matched": 0,
        "errors": 0,
        "last_tx": "-",
        "last_rx": "-",
    }

    rx_buffer = b""
    expected_lines = []
    sequence = 0

    try:
        if hasattr(os, "exitpoint"):
            os.exitpoint(os.EXITPOINT_ENABLE)

        print("K230D PORT2 UART loopback test")
        print("UART2: TX=IO44 RX=IO45, 115200 8N1")
        print("Short PORT2 TX and RX together with one jumper wire.")

        fpioa, uart = init_uart()
        print("[UART2] initialized")

        try:
            screen = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB888)
            Display.init(
                Display.ST7701,
                width=DISPLAY_WIDTH,
                height=DISPLAY_HEIGHT,
                to_ide=True,
            )
            display_ready = True
            MediaManager.init()
            media_ready = True
            print("[DISPLAY] initialized")
        except Exception as e:
            print("[DISPLAY] disabled:", e)
            if display_ready:
                try:
                    Display.deinit()
                except Exception:
                    pass
            display_ready = False
            media_ready = False
            screen = None

        now = ticks_ms()
        last_tx_ms = now - TX_INTERVAL_MS
        last_lcd_ms = now - LCD_INTERVAL_MS

        while True:
            if hasattr(os, "exitpoint"):
                os.exitpoint()

            now = ticks_ms()

            if ticks_diff(now, last_tx_ms) >= TX_INTERVAL_MS:
                last_tx_ms = now
                sequence += 1
                line = "K230D_PORT2_TEST,%06d" % sequence
                payload = (line + "\r\n").encode()

                try:
                    written = uart.write(payload)
                    if written is None:
                        written = len(payload)
                    written = int(written)

                    stats["tx_frames"] += 1
                    stats["tx_bytes"] += written
                    stats["last_tx"] = line

                    expected_lines.append(line)
                    if len(expected_lines) > EXPECTED_HISTORY_LIMIT:
                        expected_lines.pop(0)

                    if written != len(payload):
                        stats["errors"] += 1
                        print("[TX SHORT] %d/%d bytes" % (written, len(payload)))
                    else:
                        print("[TX] %s bytes=%d" % (line, written))
                except Exception as e:
                    stats["errors"] += 1
                    print("[TX ERROR]", e)

            try:
                data = uart.read(256)
                if data:
                    stats["rx_bytes"] += len(data)
                    rx_buffer += data
                    print("[RX RAW]", repr(data))

                    if len(rx_buffer) > RX_BUFFER_LIMIT:
                        rx_buffer = rx_buffer[-RX_BUFFER_LIMIT:]
                        stats["errors"] += 1
                        print("[RX ERROR] buffer trimmed")

                    while True:
                        newline_index = rx_buffer.find(b"\n")
                        if newline_index < 0:
                            break

                        raw_line = rx_buffer[:newline_index]
                        rx_buffer = rx_buffer[newline_index + 1:]
                        if raw_line.endswith(b"\r"):
                            raw_line = raw_line[:-1]

                        text = decode_ascii(raw_line)
                        stats["rx_lines"] += 1
                        stats["last_rx"] = text

                        if text in expected_lines:
                            stats["matched"] += 1
                            expected_lines.remove(text)
                            print("[RX MATCH] %s" % text)
                        else:
                            print("[RX LINE] %s" % text)
            except Exception as e:
                stats["errors"] += 1
                print("[RX ERROR]", e)

            if display_ready and media_ready and ticks_diff(now, last_lcd_ms) >= LCD_INTERVAL_MS:
                last_lcd_ms = now
                try:
                    draw_status(screen, stats)
                    Display.show_image(screen)
                except Exception as e:
                    stats["errors"] += 1
                    print("[DISPLAY ERROR]", e)

            time.sleep_ms(LOOP_DELAY_MS)

    except KeyboardInterrupt as e:
        print("user stop:", e)
    except BaseException as e:
        if "IDE interrupt" in str(e):
            print("user stop: IDE interrupt")
        else:
            print("fatal error:", e)
            try:
                sys.print_exception(e)
            except Exception:
                pass
    finally:
        print("deinit")
        if uart is not None:
            try:
                uart.deinit()
            except Exception:
                pass
        uart = None
        fpioa = None

        if display_ready:
            try:
                Display.deinit()
            except Exception:
                pass

        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass
        time.sleep_ms(100)

        if media_ready:
            try:
                MediaManager.deinit()
            except Exception:
                pass

        gc.collect()


if __name__ == "__main__":
    main()
