import sensor
import time
import pyb
from pyb import UART

# Boot diagnostic: blue LED solid the instant main.py reaches user code.
pyb.LED(3).on()

UART_BAUD = 115200
FRAME_WIDTH = 160
FRAME_HEIGHT = 120
CCD_WIDTH = 128
# ROI extended upward (y 24->8, h 92->108) so the camera sees further
# ahead and picks up the next line earlier on long-gap turns.
# ROI also extended rightward (w 132->144) so right-hand turns with a
# longer gap don't lose the next line off the right edge.
ROI = (14, 8, 144, 108)
ROI_X, ROI_Y, ROI_W, ROI_H = ROI

# Adaptive threshold: L_min = clamp(mean + K*std, MIN, MAX) per frame.
ADAPT_K = 2.5
ADAPT_L_MIN = 82
ADAPT_L_MAX = 95
A_MIN, A_MAX = -15, 15
B_MIN, B_MAX = -15, 15

# --- Line detection ---
MIN_PIXELS = 25
MIN_AREA = 25
MAX_PIXELS = 3200
MAX_BLOB_W = 105
MAX_BLOB_H = 110
MIN_ASPECT_X10 = 10

# --- Zebra detection ---
ZEBRA_STRIPE_MAX_W = 22
ZEBRA_STRIPE_MIN_H = 18
ZEBRA_H_OVER_W_X10 = 18
ZEBRA_MIN_STRIPES = 4
ZEBRA_X_SPREAD_X10 = 6
ZEBRA_CONFIRM_FRAMES = 2

# --- Traffic light detection ---
# Looks at the whole frame (not the line ROI) since the phone screen
# is held up in front of the camera.
# LAB thresholds: (L_min, L_max, A_min, A_max, B_min, B_max)
GREEN_THR = (30, 100, -90, -20, -10, 60)   # green phone screen
RED_THR_1 = (20, 90, 25, 90, 0, 70)        # red, +A side
RED_THR_2 = (20, 90, 15, 90, -20, 50)      # red fallback
LIGHT_MIN_PIXELS = 1500          # screen takes a big chunk of the frame
LIGHT_CONFIRM_FRAMES = 2         # need this many consecutive frames

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True)
sensor.skip_frames(time=1200)

clock = time.clock()
uart = UART(1, UART_BAUD, timeout_char=1000)
uart.init(UART_BAUD, 8, None, 1)

zebra_streak = 0
green_streak = 0
red_streak = 0


def send_openmv_frame(median, lost, stop=0, light=0):
    # light: 0 = none, 1 = red, 2 = green
    uart.write("$OMV,%03d,%d,%d,%d\n" % (median, lost, stop, light))


def is_line_blob(blob):
    try:
        rect = blob.rect()
        w = int(rect[2])
        h = int(rect[3])
    except Exception:
        return False
    area = w * h
    if area < MIN_AREA or area > MAX_PIXELS:
        return False
    if w > MAX_BLOB_W or h > MAX_BLOB_H:
        return False
    long_side = w if w > h else h
    short_side = h if w > h else w
    if short_side == 0:
        return False
    if long_side * 10 < short_side * MIN_ASPECT_X10:
        return False
    return True


def line_score(blob):
    rect = blob.rect()
    w = int(rect[2])
    h = int(rect[3])
    return h * 12 + w * 2


def best_blob(blobs):
    best = None
    best_score = -999999
    for b in blobs:
        if not is_line_blob(b):
            continue
        try:
            score = line_score(b)
        except Exception:
            continue
        if best is None or score > best_score:
            best = b
            best_score = score
    return best


def is_zebra_stripe(blob):
    try:
        rect = blob.rect()
        w = int(rect[2])
        h = int(rect[3])
    except Exception:
        return False
    if w == 0 or h == 0:
        return False
    if w > ZEBRA_STRIPE_MAX_W:
        return False
    if h < ZEBRA_STRIPE_MIN_H:
        return False
    if h * 10 < w * ZEBRA_H_OVER_W_X10:
        return False
    return True


def detect_zebra(blobs, img):
    xs = []
    for b in blobs:
        if is_zebra_stripe(b):
            try:
                xs.append(int(b.cx()))
                img.draw_rectangle(b.rect(), color=(255, 255, 0))
            except Exception:
                pass
    if not xs:
        return 0, 0
    spread = max(xs) - min(xs)
    return len(xs), spread


def detect_light(img):
    # Returns (best_color, best_pixels) where best_color is 'G','R' or 'N'.
    green_blobs = img.find_blobs(
        [GREEN_THR],
        pixels_threshold=LIGHT_MIN_PIXELS,
        area_threshold=LIGHT_MIN_PIXELS,
        merge=True,
    )
    red_blobs = img.find_blobs(
        [RED_THR_1, RED_THR_2],
        pixels_threshold=LIGHT_MIN_PIXELS,
        area_threshold=LIGHT_MIN_PIXELS,
        merge=True,
    )
    g_max = 0
    g_best = None
    for b in green_blobs:
        try:
            p = int(b.pixels())
        except Exception:
            continue
        if p > g_max:
            g_max = p
            g_best = b
    r_max = 0
    r_best = None
    for b in red_blobs:
        try:
            p = int(b.pixels())
        except Exception:
            continue
        if p > r_max:
            r_max = p
            r_best = b
    # Pick whichever color is bigger; ties go to green (safer = move).
    if g_max >= r_max and g_max >= LIGHT_MIN_PIXELS:
        if g_best is not None:
            img.draw_rectangle(g_best.rect(), color=(0, 255, 0), thickness=2)
        return 'G', g_max
    if r_max >= LIGHT_MIN_PIXELS:
        if r_best is not None:
            img.draw_rectangle(r_best.rect(), color=(255, 0, 0), thickness=2)
        return 'R', r_max
    return 'N', 0


while True:
    clock.tick()
    img = sensor.snapshot()
    img.draw_rectangle(ROI, color=(0, 0, 255))

    # --- Traffic light (whole frame) ---
    light_color, light_px = detect_light(img)
    if light_color == 'G':
        green_streak += 1
        red_streak = 0
    elif light_color == 'R':
        red_streak += 1
        green_streak = 0
    else:
        green_streak = 0
        red_streak = 0

    if green_streak >= LIGHT_CONFIRM_FRAMES:
        light_field = 2
    elif red_streak >= LIGHT_CONFIRM_FRAMES:
        light_field = 1
    else:
        light_field = 0

    # --- Line ROI processing ---
    stats = img.get_statistics(roi=ROI)
    l_mean = stats.l_mean()
    l_std = stats.l_stdev()
    l_min = int(l_mean + ADAPT_K * l_std)
    if l_min < ADAPT_L_MIN:
        l_min = ADAPT_L_MIN
    elif l_min > ADAPT_L_MAX:
        l_min = ADAPT_L_MAX
    thr = [(l_min, 100, A_MIN, A_MAX, B_MIN, B_MAX)]

    blobs = img.find_blobs(
        thr,
        roi=ROI,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=False
    )

    # --- Zebra detection ---
    zebra_count, zebra_spread = detect_zebra(blobs, img)
    spread_thresh = ROI_W * ZEBRA_X_SPREAD_X10 // 10
    if zebra_count >= ZEBRA_MIN_STRIPES and zebra_spread >= spread_thresh:
        zebra_streak += 1
    else:
        zebra_streak = 0
    is_zebra = zebra_streak >= ZEBRA_CONFIRM_FRAMES
    stop_flag = 1 if is_zebra else 0

    # --- Line median ---
    line = best_blob(blobs)
    if line:
        img.draw_rectangle(line.rect(), color=(255, 0, 0))
        img.draw_cross(line.cx(), line.cy(), color=(0, 255, 0))

        median = int(line.cx() * CCD_WIDTH / FRAME_WIDTH)
        if median < 0:
            median = 0
        elif median > CCD_WIDTH:
            median = CCD_WIDTH

        print("LINE,%03d,%d Z=%d sp=%d zebra=%d L=%s px=%d lf=%d" % (
            median, line.cx(), zebra_count, zebra_spread, 1 if is_zebra else 0,
            light_color, light_px, light_field))
        send_openmv_frame(median, 0, stop_flag, light_field)
    else:
        print("LOST Z=%d sp=%d zebra=%d L=%s px=%d lf=%d" % (
            zebra_count, zebra_spread, 1 if is_zebra else 0,
            light_color, light_px, light_field))
        send_openmv_frame(64, 1, stop_flag, light_field)

    print("FPS:%.1f Lm=%d Ls=%d Lmin=%d" % (
        clock.fps(), int(l_mean), int(l_std), l_min))
