import gc
import os
import sys
import time


for _path in ("/sdcard", "/sdcard/libs"):
    try:
        if _path not in sys.path:
            sys.path.append(_path)
    except Exception:
        pass

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from machine import FPIOA, UART


APP_VERSION = "2026-07-31.1-target-hold-port2"

MODEL_NAME = "best1.kmodel"
MODEL_SIZE = 3343832
MODEL_PATHS = (
    "/sdcard/examples/kmodel/best1.kmodel",
    "/sdcard/best1.kmodel",
    "/sdcard/examples/kmodels/best1.kmodel",
    "/sdcard/kmodel/best1.kmodel",
    "/sdcard/models/best1.kmodel",
    "/data/best1.kmodel",
    "/flash/best1.kmodel",
)

# Original best1.kmodel project camera configuration for the normal lens.
# YOLO letterboxes the 16:9 camera frame to the 320x320 model input.
RGB888P_SIZE = [640, 360]
MODEL_INPUT_SIZE = [320, 320]
LABELS = ["ball"]
CONFIDENCE = 0.30
NMS_THRESHOLD = 0.45
MAX_BOXES = 20

# Normal DNK230D BOX camera configuration used by the model's source project.
CAMERA_SENSOR_ID = 2

# K230D BOX PORT2: IO44=UART2_TXD, IO45=UART2_RXD, 115200 8N1.
UART_TX_IO = 44
UART_RX_IO = 45
UART_BAUD = 115200

LAST_TARGET_HOLD_MS = 500
GC_INTERVAL_FRAMES = 120


def sleep_ms(ms):
    try:
        time.sleep_ms(ms)
    except Exception:
        time.sleep(ms / 1000)


def ticks_ms():
    try:
        return time.ticks_ms()
    except Exception:
        return int(time.time() * 1000)


def ticks_diff(now, old):
    try:
        return time.ticks_diff(now, old)
    except Exception:
        return now - old


def file_size(path):
    try:
        return int(os.stat(path)[6])
    except Exception:
        return -1


def find_model():
    for path in MODEL_PATHS:
        size = file_size(path)
        if size < 0:
            continue
        if size == MODEL_SIZE:
            print("[MODEL] best1:", path, size)
            return path
        print("[MODEL] reject wrong size:", path, size, "expected", MODEL_SIZE)

    print("[MODEL] best1.kmodel not found")
    for path in MODEL_PATHS:
        print("[MODEL] try:", path)
    return None


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
    print(
        "[UART] UART2 TX IO%d RX IO%d baud %d 8N1"
        % (UART_TX_IO, UART_RX_IO, UART_BAUD)
    )
    return uart, fpioa


def send_track(uart, err_x, err_y, valid):
    msg = "X%dY%dZ%dE" % (
        int(err_x),
        int(err_y),
        1 if valid else 0,
    )
    try:
        uart.write(msg + "\r\n")
    except Exception as e:
        print("[UART] write failed:", e)
    return msg


def choose_nearest_box(result, display_size):
    if not result or len(result) < 3:
        return -1

    boxes = result[0]
    scores = result[2]
    if boxes is None or len(boxes) == 0:
        return -1

    screen_x = display_size[0] / 2
    screen_y = display_size[1] / 2
    best_index = -1
    best_distance2 = None
    best_score = -1.0

    for i in range(len(boxes)):
        try:
            x, y, width, height = boxes[i]
            center_x = float(x) + float(width) / 2
            center_y = float(y) + float(height) / 2
            dx = center_x - screen_x
            dy = center_y - screen_y
            distance2 = dx * dx + dy * dy
            score = float(scores[i])
        except Exception:
            continue

        if (
            best_distance2 is None
            or distance2 < best_distance2
            or (distance2 == best_distance2 and score > best_score)
        ):
            best_index = i
            best_distance2 = distance2
            best_score = score

    return best_index


def draw_overlay(
    osd_img,
    result,
    selected_index,
    display_size,
    track_msg,
    track_state,
):
    osd_img.clear()

    if result and len(result) >= 3:
        boxes = result[0]
        if boxes is not None and len(boxes) > 0:
            for i in range(len(boxes)):
                try:
                    x, y, width, height = boxes[i]
                    x = int(round(x, 0))
                    y = int(round(y, 0))
                    width = int(round(width, 0))
                    height = int(round(height, 0))
                    if width <= 0 or height <= 0:
                        continue

                    selected = i == selected_index
                    box_color = (255, 255, 255, 0) if selected else (255, 0, 255, 0)
                    center_color = (255, 255, 0, 0) if selected else (255, 0, 255, 255)
                    thickness = 5 if selected else 3

                    osd_img.draw_rectangle(
                        x,
                        y,
                        width,
                        height,
                        color=box_color,
                        thickness=thickness,
                    )
                    osd_img.draw_circle(
                        x + width // 2,
                        y + height // 2,
                        6 if selected else 4,
                        color=center_color,
                        thickness=2,
                    )
                except Exception:
                    pass

    center_x = int(display_size[0] // 2)
    center_y = int(display_size[1] // 2)
    osd_img.draw_line(
        center_x - 12,
        center_y,
        center_x + 12,
        center_y,
        color=(255, 255, 255, 255),
        thickness=2,
    )
    osd_img.draw_line(
        center_x,
        center_y - 12,
        center_x,
        center_y + 12,
        color=(255, 255, 255, 255),
        thickness=2,
    )

    try:
        osd_img.draw_string_advanced(
            12,
            12,
            24,
            "Steel Ball %s  %s" % (track_state, track_msg),
            color=(255, 255, 255, 255),
        )
    except Exception:
        try:
            osd_img.draw_string(
                12,
                12,
                "Steel Ball %s %s" % (track_state, track_msg),
                color=(255, 255, 255, 255),
                scale=2,
            )
        except Exception:
            pass


def main():
    print("K230D steel-ball-only app", APP_VERSION)

    pipeline = None
    detector = None
    uart = None
    fpioa = None

    try:
        model_path = find_model()
        if model_path is None:
            while True:
                try:
                    os.exitpoint()
                except Exception:
                    pass
                sleep_ms(500)

        uart, fpioa = init_uart()
        for _ in range(3):
            send_track(uart, 0, 0, False)
            sleep_ms(20)

        print("[PIPELINE] create")
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode="lcd",
        )
        pipeline.create(sensor_id=CAMERA_SENSOR_ID)
        display_size = pipeline.get_display_size()
        print(
            "[CAMERA] normal sensor %d rgb %dx%d"
            % (CAMERA_SENSOR_ID, RGB888P_SIZE[0], RGB888P_SIZE[1])
        )
        print("[PIPELINE] display size:", display_size)

        print("[MODEL] load:", model_path)
        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=model_path,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        detector.config_preprocess()
        print("[MODEL] ready")

        frame_count = 0
        last_msg = "X0Y0Z0E"
        last_target_err_x = 0
        last_target_err_y = 0
        last_target_ms = 0
        has_last_target = False

        while True:
            os.exitpoint()
            frame_count += 1

            ai_frame = pipeline.get_frame()
            result = detector.run(ai_frame)
            selected_index = choose_nearest_box(result, display_size)
            now_ms = ticks_ms()

            if selected_index >= 0:
                box = result[0][selected_index]
                center_x = float(box[0]) + float(box[2]) / 2
                center_y = float(box[1]) + float(box[3]) / 2
                last_target_err_x = int(display_size[0] / 2 - center_x)
                last_target_err_y = int(display_size[1] / 2 - center_y)
                last_target_ms = now_ms
                has_last_target = True
                track_state = "LIVE"
                last_msg = send_track(
                    uart,
                    last_target_err_x,
                    last_target_err_y,
                    True,
                )
            elif (
                has_last_target
                and ticks_diff(now_ms, last_target_ms) <= LAST_TARGET_HOLD_MS
            ):
                track_state = "HOLD"
                last_msg = send_track(
                    uart,
                    last_target_err_x,
                    last_target_err_y,
                    True,
                )
            else:
                track_state = "LOST"
                last_msg = send_track(uart, 0, 0, False)

            draw_overlay(
                pipeline.osd_img,
                result,
                selected_index,
                display_size,
                last_msg,
                track_state,
            )
            pipeline.show_image()

            ai_frame = None
            result = None

            if frame_count % GC_INTERVAL_FRAMES == 0:
                print("[STEEL UART]", last_msg, "frames", frame_count)
                gc.collect()

    except KeyboardInterrupt as e:
        print("user stop:", e)
    except BaseException as e:
        print("steel ball app failed:", e)
        try:
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        if uart is not None:
            try:
                send_track(uart, 0, 0, False)
                sleep_ms(20)
                uart.deinit()
            except Exception:
                pass
        if detector is not None:
            try:
                detector.deinit()
            except Exception:
                pass
        if pipeline is not None:
            try:
                pipeline.destroy()
            except Exception:
                pass
        detector = None
        pipeline = None
        uart = None
        fpioa = None
        gc.collect()
        sleep_ms(100)


if __name__ == "__main__":
    main()
