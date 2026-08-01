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
from machine import FPIOA, TOUCH, UART


APP_VERSION = "2026-08-01.4-best2-touch-targets-port2"

MODEL_NAME = "best2.kmodel"
MODEL_SIZE = 3454960
MODEL_PATHS = (
    "/sdcard/examples/kmodel/best2.kmodel",
    "/sdcard/best2.kmodel",
    "/sdcard/examples/kmodels/best2.kmodel",
    "/sdcard/kmodel/best2.kmodel",
    "/sdcard/models/best2.kmodel",
    "/data/best2.kmodel",
    "/flash/best2.kmodel",
)

# Full camera frame delivered by PipeLine channel 2.
RGB888P_SIZE = [640, 360]
MODEL_INPUT_SIZE = [416, 416]
LABELS = ["ball"]
CONFIDENCE = 0.20
NMS_THRESHOLD = 0.45
MAX_BOXES = 20

# Only the vertically centered strip is displayed and sent into the model.
# A 60-pixel AI strip maps to 80 pixels on the 640x480 LCD.
ROI_DISPLAY_HEIGHT = 80
ROI_DIVISIONS = 25

# Normal DNK230D BOX camera configuration used by the model's source project.
CAMERA_SENSOR_ID = 2

# K230D BOX PORT2: IO44=UART2_TXD, IO45=UART2_RXD, 115200 8N1.
UART_TX_IO = 44
UART_RX_IO = 45
UART_BAUD = 115200

LAST_TARGET_HOLD_MS = 500
GC_INTERVAL_FRAMES = 120

MODE_CENTER = 1
MODE_LEFT_RIGHT = 2
MODE_CUSTOM = 3
TARGET_TOLERANCE_DIVISIONS = 1.0
LEFT_TARGET_DIVISIONS = 5.0
LEFT_STABLE_TIME_MS = 1000

BUTTON_MARGIN = 16
BUTTON_GAP = 12
BUTTON_Y = 36
BUTTON_HEIGHT = 64


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


def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def button_layout(display_width):
    button_width = int(
        (display_width - BUTTON_MARGIN * 2 - BUTTON_GAP * 2) / 3
    )
    return (
        (BUTTON_MARGIN, BUTTON_Y, button_width, BUTTON_HEIGHT, MODE_CENTER, "CENTER"),
        (
            BUTTON_MARGIN + button_width + BUTTON_GAP,
            BUTTON_Y,
            button_width,
            BUTTON_HEIGHT,
            MODE_LEFT_RIGHT,
            "L-R",
        ),
        (
            BUTTON_MARGIN + (button_width + BUTTON_GAP) * 2,
            BUTTON_Y,
            button_width,
            BUTTON_HEIGHT,
            MODE_CUSTOM,
            "CUSTOM",
        ),
    )


class TouchInput:
    def __init__(self):
        self.pressed = False

    def poll_down(self, touch):
        if touch is None:
            return None

        try:
            points = touch.read(1)
        except Exception as e:
            print("[TOUCH] read failed:", e)
            self.pressed = False
            return None

        if not points:
            self.pressed = False
            return None

        point = points[0]
        event = getattr(point, "event", None)
        event_up = getattr(TOUCH, "EVENT_UP", None)
        is_pressed = not (event_up is not None and event == event_up)
        is_new_press = is_pressed and not self.pressed
        self.pressed = is_pressed
        if not is_new_press:
            return None

        try:
            return int(point.x), int(point.y)
        except Exception:
            return None


class TargetController:
    def __init__(self):
        self.mode = MODE_CENTER
        self.left_right_phase = 0
        self.left_stable_start_ms = -1
        self.custom_waiting = False
        self.custom_valid = False
        self.custom_x = 0.0
        self.custom_y = 0.0

    def set_mode(self, mode):
        self.mode = mode
        self.left_stable_start_ms = -1
        if mode == MODE_LEFT_RIGHT:
            self.left_right_phase = 0
        if mode == MODE_CUSTOM:
            self.custom_waiting = True
            self.custom_valid = False
        else:
            self.custom_waiting = False
        print("[MODE]", self.status_text(ticks_ms()))

    def handle_touch(self, x, y, display_size, roi_y, roi_size):
        display_width = int(display_size[0])
        for bx, by, bw, bh, mode, label in button_layout(display_width):
            if x >= bx and x < bx + bw and y >= by and y < by + bh:
                self.set_mode(mode)
                return True

        if self.mode == MODE_CUSTOM and self.custom_waiting:
            roi_bottom = roi_y + int(roi_size[1])
            if y >= roi_y and y < roi_bottom:
                self.custom_x = float(clamp(x, 0, int(roi_size[0]) - 1))
                self.custom_y = float(clamp(y - roi_y, 0, int(roi_size[1]) - 1))
                self.custom_waiting = False
                self.custom_valid = True
                print(
                    "[MODE] custom target x=%d y=%d"
                    % (int(self.custom_x), int(self.custom_y))
                )
                return True
            print("[MODE] custom target must be inside image strip")
        return False

    def target(self, roi_size):
        roi_width = float(roi_size[0])
        roi_height = float(roi_size[1])
        center_x = roi_width / 2.0
        center_y = roi_height / 2.0
        section_width = roi_width / float(ROI_DIVISIONS)

        if self.mode == MODE_CENTER:
            return center_x, center_y
        if self.mode == MODE_LEFT_RIGHT:
            if self.left_right_phase == 0:
                return center_x - section_width * LEFT_TARGET_DIVISIONS, center_y
            return center_x + section_width * LEFT_TARGET_DIVISIONS, center_y
        if self.mode == MODE_CUSTOM and self.custom_valid:
            return self.custom_x, self.custom_y
        return None

    def tolerance_pixels(self, roi_size):
        return (
            float(roi_size[0])
            / float(ROI_DIVISIONS)
            * TARGET_TOLERANCE_DIVISIONS
        )

    def update_live(self, ball_x, now_ms, roi_size):
        if self.mode != MODE_LEFT_RIGHT or self.left_right_phase != 0:
            return False

        target = self.target(roi_size)
        if target is None:
            self.left_stable_start_ms = -1
            return False

        if abs(float(target[0]) - float(ball_x)) <= self.tolerance_pixels(roi_size):
            if self.left_stable_start_ms < 0:
                self.left_stable_start_ms = now_ms
            elif ticks_diff(now_ms, self.left_stable_start_ms) >= LEFT_STABLE_TIME_MS:
                self.left_right_phase = 1
                self.left_stable_start_ms = -1
                print("[MODE] left stable 1s, switch to right target")
                return True
        else:
            self.left_stable_start_ms = -1
        return False

    def update_missing(self):
        if self.mode == MODE_LEFT_RIGHT and self.left_right_phase == 0:
            self.left_stable_start_ms = -1

    def apply_x_deadband(self, error_x, roi_size):
        if abs(float(error_x)) <= self.tolerance_pixels(roi_size):
            return 0
        return int(error_x)

    def status_text(self, now_ms):
        if self.mode == MODE_CENTER:
            return "CENTER +/-1 DIV"
        if self.mode == MODE_LEFT_RIGHT:
            if self.left_right_phase != 0:
                return "L-R RIGHT HOLD"
            if self.left_stable_start_ms >= 0:
                elapsed = max(0, ticks_diff(now_ms, self.left_stable_start_ms))
                elapsed = min(LEFT_STABLE_TIME_MS, elapsed)
                return "L-R LEFT %dms" % elapsed
            return "L-R LEFT"
        if self.custom_waiting:
            return "CUSTOM: TAP STRIP"
        if self.custom_valid:
            return "CUSTOM X%d Y%d" % (int(self.custom_x), int(self.custom_y))
        return "CUSTOM"


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
            print("[MODEL] best2:", path, size)
            return path
        print("[MODEL] reject wrong size:", path, size, "expected", MODEL_SIZE)

    print("[MODEL] best2.kmodel not found")
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


def choose_nearest_box(result, roi_size):
    if not result or len(result) < 3:
        return -1

    boxes = result[0]
    scores = result[2]
    if boxes is None or len(boxes) == 0:
        return -1

    screen_x = roi_size[0] / 2
    screen_y = roi_size[1] / 2
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


def draw_text(osd_img, x, y, text, color, size=24):
    try:
        osd_img.draw_string_advanced(x, y, size, text, color=color)
        return
    except Exception:
        pass
    try:
        osd_img.draw_string(x, y, text, color=color, scale=2)
    except Exception:
        pass


def draw_overlay(
    osd_img,
    result,
    selected_index,
    display_size,
    roi_y,
    roi_size,
    target_controller,
    now_ms,
):
    osd_img.clear()

    display_width = int(display_size[0])
    display_height = int(display_size[1])
    roi_width = int(roi_size[0])
    roi_height = int(roi_size[1])
    roi_bottom = roi_y + roi_height

    # Hide everything outside the center strip while leaving the hardware
    # video layer visible inside it.
    mask_color = (255, 0, 0, 0)
    if roi_y > 0:
        osd_img.draw_rectangle(
            0,
            0,
            display_width,
            roi_y,
            color=mask_color,
            fill=True,
        )
    if roi_bottom < display_height:
        osd_img.draw_rectangle(
            0,
            roi_bottom,
            display_width,
            display_height - roi_bottom,
            color=mask_color,
            fill=True,
        )

    osd_img.draw_rectangle(
        0,
        roi_y,
        roi_width - 1,
        roi_height - 1,
        color=(255, 160, 160, 160),
        thickness=1,
    )

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

                    x2 = min(roi_width - 1, x + width)
                    y2 = min(roi_height - 1, y + height)
                    x = max(0, x)
                    y = max(0, y)
                    width = x2 - x
                    height = y2 - y
                    if width <= 0 or height <= 0:
                        continue

                    selected = i == selected_index
                    box_color = (255, 255, 255, 0) if selected else (255, 0, 255, 0)
                    center_color = (255, 255, 0, 0) if selected else (255, 0, 255, 255)
                    thickness = 5 if selected else 3

                    osd_img.draw_rectangle(
                        x,
                        roi_y + y,
                        width,
                        height,
                        color=box_color,
                        thickness=thickness,
                    )
                    osd_img.draw_circle(
                        x + width // 2,
                        roi_y + y + height // 2,
                        6 if selected else 4,
                        color=center_color,
                        thickness=2,
                    )
                except Exception:
                    pass

    # Five fixed lines derived from 25 equal horizontal divisions:
    # center, center +/- 1 division and center +/- 5 divisions.
    section_width = float(roi_width) / ROI_DIVISIONS
    center_x = float(roi_width) / 2
    line_specs = (
        (center_x - section_width * 5, (255, 0, 255, 255)),
        (center_x - section_width, (255, 255, 255, 0)),
        (center_x, (255, 255, 255, 255)),
        (center_x + section_width, (255, 255, 255, 0)),
        (center_x + section_width * 5, (255, 0, 255, 255)),
    )
    for line_x, line_color in line_specs:
        x = int(round(line_x, 0))
        osd_img.draw_line(
            x,
            roi_y + 1,
            x,
            roi_bottom - 2,
            color=line_color,
            thickness=2,
        )

    target = target_controller.target(roi_size)
    if target is not None:
        target_x = int(round(target[0], 0))
        target_y = int(round(target[1], 0))
        tolerance = int(round(target_controller.tolerance_pixels(roi_size), 0))
        target_color = (255, 0, 255, 0)
        tolerance_color = (255, 0, 120, 0)

        for boundary_x in (target_x - tolerance, target_x + tolerance):
            if boundary_x >= 0 and boundary_x < roi_width:
                osd_img.draw_line(
                    boundary_x,
                    roi_y + 1,
                    boundary_x,
                    roi_bottom - 2,
                    color=tolerance_color,
                    thickness=1,
                )
        osd_img.draw_line(
            target_x,
            roi_y + 1,
            target_x,
            roi_bottom - 2,
            color=target_color,
            thickness=4,
        )
        osd_img.draw_circle(
            target_x,
            roi_y + target_y,
            8,
            color=target_color,
            thickness=3,
        )

    for bx, by, bw, bh, mode, label in button_layout(display_width):
        active = target_controller.mode == mode
        fill_color = (255, 0, 105, 180) if active else (255, 38, 38, 38)
        border_color = (255, 0, 255, 255) if active else (255, 150, 150, 150)
        text_color = (255, 255, 255, 255)
        osd_img.draw_rectangle(bx, by, bw, bh, color=fill_color, fill=True)
        osd_img.draw_rectangle(
            bx,
            by,
            bw,
            bh,
            color=border_color,
            thickness=3 if active else 1,
        )
        text_x = bx + max(8, int((bw - len(label) * 12) / 2))
        draw_text(osd_img, text_x, by + 17, label, text_color, 24)

    status_y = min(display_height - 70, roi_bottom + 34)
    draw_text(
        osd_img,
        18,
        status_y,
        target_controller.status_text(now_ms),
        (255, 255, 255, 255),
        24,
    )


def main():
    print("K230D steel-ball-only app", APP_VERSION)

    pipeline = None
    detector = None
    uart = None
    fpioa = None
    touch = None
    touch_input = TouchInput()
    target_controller = TargetController()

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
        # The GC2093 source is 16:9 while the DNK230D LCD is 4:3. Crop the
        # sensor image centrally before scaling so circles are not widened.
        # PipeLine applies the same crop to the display and AI channels, so
        # detection boxes and UART center offsets remain aligned.
        pipeline.create(
            sensor_id=CAMERA_SENSOR_ID,
            crop_vertical=True,
        )
        display_size = pipeline.get_display_size()
        roi_display_height = min(ROI_DISPLAY_HEIGHT, int(display_size[1]))
        roi_display_y = (int(display_size[1]) - roi_display_height) // 2
        roi_display_size = [int(display_size[0]), roi_display_height]

        roi_ai_height = int(
            round(
                roi_display_height
                * RGB888P_SIZE[1]
                / float(display_size[1]),
                0,
            )
        )
        roi_ai_height = max(16, min(RGB888P_SIZE[1], roi_ai_height))
        roi_ai_height = (roi_ai_height // 2) * 2
        roi_ai_y = (RGB888P_SIZE[1] - roi_ai_height) // 2
        roi_ai_size = [RGB888P_SIZE[0], roi_ai_height]
        try:
            touch = TOUCH(0)
            print("[TOUCH] TOUCH(0) ready, default mode CENTER")
        except Exception as e:
            touch = None
            print("[TOUCH] init failed, CENTER mode remains active:", e)
        print(
            "[CAMERA] normal sensor %d rgb %dx%d aspect crop on"
            % (CAMERA_SENSOR_ID, RGB888P_SIZE[0], RGB888P_SIZE[1])
        )
        print("[PIPELINE] display size:", display_size)
        print(
            "[ROI] AI x=%d y=%d w=%d h=%d display y=%d w=%d h=%d conf=%.2f"
            % (
                0,
                roi_ai_y,
                roi_ai_size[0],
                roi_ai_size[1],
                roi_display_y,
                roi_display_size[0],
                roi_display_size[1],
                CONFIDENCE,
            )
        )

        print("[MODEL] load:", model_path)
        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=model_path,
            labels=LABELS,
            rgb888p_size=roi_ai_size,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=roi_display_size,
            conf_thresh=CONFIDENCE,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        detector.ai2d.crop(
            0,
            roi_ai_y,
            RGB888P_SIZE[0],
            roi_ai_height,
        )
        detector.config_preprocess(input_image_size=RGB888P_SIZE)
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

            touch_point = touch_input.poll_down(touch)
            if touch_point is not None:
                if target_controller.handle_touch(
                    touch_point[0],
                    touch_point[1],
                    display_size,
                    roi_display_y,
                    roi_display_size,
                ):
                    has_last_target = False
                    last_target_err_x = 0
                    last_target_err_y = 0
                    last_target_ms = 0

            ai_frame = pipeline.get_frame()
            result = detector.run(ai_frame)
            selected_index = choose_nearest_box(result, roi_display_size)
            now_ms = ticks_ms()

            if selected_index >= 0:
                box = result[0][selected_index]
                center_x = float(box[0]) + float(box[2]) / 2
                center_y = float(box[1]) + float(box[3]) / 2
                target_controller.update_live(center_x, now_ms, roi_display_size)
                target = target_controller.target(roi_display_size)
                if target is not None:
                    raw_error_x = float(target[0]) - center_x
                    raw_error_y = float(target[1]) - center_y
                    last_target_err_x = target_controller.apply_x_deadband(
                        raw_error_x,
                        roi_display_size,
                    )
                    last_target_err_y = int(raw_error_y)
                    last_target_ms = now_ms
                    has_last_target = True
                    track_state = "LIVE"
                    last_msg = send_track(
                        uart,
                        last_target_err_x,
                        last_target_err_y,
                        True,
                    )
                else:
                    has_last_target = False
                    track_state = "WAIT_TARGET"
                    last_msg = send_track(uart, 0, 0, False)
            elif (
                has_last_target
                and ticks_diff(now_ms, last_target_ms) <= LAST_TARGET_HOLD_MS
            ):
                target_controller.update_missing()
                track_state = "HOLD"
                last_msg = send_track(
                    uart,
                    last_target_err_x,
                    last_target_err_y,
                    True,
                )
            else:
                target_controller.update_missing()
                track_state = "LOST"
                last_msg = send_track(uart, 0, 0, False)

            draw_overlay(
                pipeline.osd_img,
                result,
                selected_index,
                display_size,
                roi_display_y,
                roi_display_size,
                target_controller,
                now_ms,
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
        touch = None
        gc.collect()
        sleep_ms(100)


if __name__ == "__main__":
    main()
