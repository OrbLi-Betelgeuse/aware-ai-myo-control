#!/usr/bin/env python3
"""Steering-wheel window: shows the driver how far they twist and which way the robot turns.

Add it to any controller with two lines:
    import wheel_display                              # at the top of the controller
    wheel_display.show(steer, v=v, w=w, note=note)    # once per control tick (for example at the end of step())

    steer : -1 .. 1, + = left (the twist after calibration). Turns the wheel and moves the hand.
    v, w  : the command sent (m/s, rad/s): speed as text and a bar, turn rate as text (optional)
    note  : a short message, for example "ASSIST: obstacle ahead" (optional), shown in a yellow banner

The window opens on the first call. Closing it only hides the wheel; the controller keeps running.
Try it without a Myo:   python examples/wheel_display.py   (the wheel swings left and right)
Saves nothing. Needs pygame (already in the hackathon conda environment).
"""
import math
import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")   # no pygame banner in the terminal
import pygame
import pygame.gfxdraw

SIZE = (420, 570)                   # px, window width and height
MAX_ANGLE = 90                      # deg, wheel rotation at a full twist (steer = ±1)
V_SCALE = 2.0                       # m/s, speed that fills the speed bar (the controller's V_MAX)
FONT = "helveticaneue,helvetica,arial"   # the first font found on the computer is used

# Colors (R, G, B). Left / right / forward / back match the terminal colors of the controller.
BG_TOP, BG_BOTTOM = (34, 38, 52), (14, 15, 20)          # background gradient
CARD = (42, 46, 60)                                      # panels behind the text
RIM, RIM_LIGHT, FACE, HUB = (70, 74, 88), (150, 155, 170), (24, 26, 34), (95, 100, 115)
TEXT, DIM = (235, 237, 242), (130, 135, 150)
LEFT_COLOR, RIGHT_COLOR, CENTER_COLOR = (80, 140, 255), (230, 90, 230), (240, 240, 240)
FORWARD_COLOR, BACK_COLOR, NOTE_COLOR = (90, 220, 120), (240, 80, 80), (250, 210, 60)

WINDOW = {"screen": None, "bg": None, "fonts": {}, "closed": False}


def rim_point(cx: int, cy: int, radius: float, a: float) -> tuple[int, int]:
    """Point on a circle; a = 0 is the top (12 o'clock), a > 0 turns counter-clockwise (to the left)."""
    return round(cx - radius * math.sin(a)), round(cy - radius * math.cos(a))


def disc(screen, color, center, radius: int) -> None:
    """Filled circle with smooth (antialiased) edges."""
    pygame.gfxdraw.filled_circle(screen, center[0], center[1], radius, color)
    pygame.gfxdraw.aacircle(screen, center[0], center[1], radius, color)


def thick_line(screen, color, p1, p2, width: float) -> None:
    """Thick line with smooth edges (a filled four-corner shape), for the spokes and the scale."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = max(math.hypot(dx, dy), 1e-9)
    ox, oy = -dy / length * width / 2, dx / length * width / 2
    corners = [(round(p1[0] + ox), round(p1[1] + oy)), (round(p2[0] + ox), round(p2[1] + oy)),
               (round(p2[0] - ox), round(p2[1] - oy)), (round(p1[0] - ox), round(p1[1] - oy))]
    pygame.gfxdraw.filled_polygon(screen, corners, color)
    pygame.gfxdraw.aapolygon(screen, corners, color)


def text(screen, message: str, size: int, color, center, bold: bool = True) -> None:
    """Draw text centered on a point (fonts are loaded once and kept)."""
    if (size, bold) not in WINDOW["fonts"]:
        WINDOW["fonts"][(size, bold)] = pygame.font.SysFont(FONT, size, bold=bold)
    image = WINDOW["fonts"][(size, bold)].render(message, True, color)
    screen.blit(image, image.get_rect(center=center))


def show(steer: float, v: float | None = None, w: float | None = None, note: str = "") -> None:
    """Draw one frame of the wheel. Call it every control tick (20 Hz is plenty)."""
    if WINDOW["closed"]:
        return
    if WINDOW["screen"] is None:
        pygame.init()
        WINDOW["screen"] = pygame.display.set_mode(SIZE)
        pygame.display.set_caption("Steering wheel")
        WINDOW["bg"] = pygame.Surface(SIZE)
        for y in range(SIZE[1]):              # vertical color gradient, drawn once
            f = y / SIZE[1]
            pygame.draw.line(WINDOW["bg"], [round(a + (b - a) * f) for a, b in zip(BG_TOP, BG_BOTTOM)],
                             (0, y), (SIZE[0], y))
    for event in pygame.event.get():          # keeps the window responsive; closing it hides the wheel
        if event.type == pygame.QUIT:
            pygame.display.quit()
            WINDOW["closed"] = True
            return

    screen = WINDOW["screen"]
    steer = max(-1.0, min(1.0, float(steer)))
    turning = abs(steer) > 0.02
    color = LEFT_COLOR if steer > 0.02 else RIGHT_COLOR if steer < -0.02 else CENTER_COLOR
    cx, cy, r = SIZE[0] // 2, 250, 140        # wheel centre and radius, px
    angle = math.radians(MAX_ANGLE * steer)   # the whole wheel turns with the twist
    screen.blit(WINDOW["bg"], (0, 0))

    # Scale above the wheel: a tick every 30°, a longer one at straight ahead, L and R at the ends.
    for deg in range(-MAX_ANGLE, MAX_ANGLE + 1, 30):
        a = math.radians(deg)
        thick_line(screen, TEXT if deg == 0 else DIM, rim_point(cx, cy, r + 18, a),
                   rim_point(cx, cy, r + (36 if deg == 0 else 28), a), 4 if deg == 0 else 3)
    text(screen, "L", 24, LEFT_COLOR, rim_point(cx, cy, r + 46, math.radians(MAX_ANGLE)))
    text(screen, "R", 24, RIGHT_COLOR, rim_point(cx, cy, r + 46, math.radians(-MAX_ANGLE)))

    # Wheel: rim (dark ring with light edges) and the dark face.
    disc(screen, RIM_LIGHT, (cx, cy), r + 12)
    disc(screen, RIM, (cx, cy), r + 9)
    disc(screen, RIM_LIGHT, (cx, cy), r - 11)
    disc(screen, FACE, (cx, cy), r - 13)
    # How far you twist: a colored band on the rim, from straight ahead to the hand.
    if turning:
        steps = max(2, int(abs(MAX_ANGLE * steer) / 2))
        for i in range(steps + 1):
            disc(screen, color, rim_point(cx, cy, r - 1, angle * i / steps), 8)
    # Three spokes (left, right, bottom) and the hub; they turn with the wheel.
    for spoke in (angle + math.pi / 2, angle - math.pi / 2, angle + math.pi):
        thick_line(screen, HUB, (cx, cy), rim_point(cx, cy, r - 12, spoke), 16)
    disc(screen, RIM_LIGHT, (cx, cy), 40)
    disc(screen, HUB, (cx, cy), 37)
    text(screen, "MYO", 20, TEXT, (cx, cy))
    # The hand: a glowing dot on the rim that starts at the top and moves with the twist.
    hand = rim_point(cx, cy, r - 1, angle)
    glow = pygame.Surface((80, 80), pygame.SRCALPHA)
    pygame.draw.circle(glow, (*color, 70), (40, 40), 34)
    screen.blit(glow, (hand[0] - 40, hand[1] - 40))
    disc(screen, TEXT, hand, 22)
    disc(screen, color, hand, 18)

    # Panel 1: which way and how much.
    pygame.draw.rect(screen, CARD, (20, 418, SIZE[0] - 40, 52), border_radius=14)
    label = f"LEFT  {steer:.0%}" if steer > 0.02 else f"RIGHT  {-steer:.0%}" if steer < -0.02 else "STRAIGHT"
    text(screen, label, 34, color, (cx, 444))
    # Panel 2: speed (text and a rounded bar, green forward / red back, full at V_SCALE) and turn rate.
    if v is not None or w is not None:
        pygame.draw.rect(screen, CARD, (20, 480, SIZE[0] - 40, 78), border_radius=14)
    if v is not None:
        bar_color = FORWARD_COLOR if v >= 0 else BACK_COLOR
        text(screen, f"{'FORWARD' if v >= 0 else 'BACK'}  {abs(v):.2f} m/s", 26, bar_color, (cx, 500))
        pygame.draw.rect(screen, FACE, (40, 516, SIZE[0] - 80, 14), border_radius=7)
        fill = round((SIZE[0] - 80) * min(abs(v) / V_SCALE, 1.0))
        if fill > 0:
            pygame.draw.rect(screen, bar_color, (40, 516, fill, 14), border_radius=7)
    if w is not None:
        text(screen, f"turn rate {w:+.2f} rad/s", 20, DIM, (cx, 543), bold=False)
    # Yellow banner at the top for messages from the controller.
    if note:
        pygame.draw.rect(screen, NOTE_COLOR, (20, 10, SIZE[0] - 40, 34), border_radius=17)
        text(screen, note, 20, FACE, (cx, 27))
    pygame.display.flip()


if __name__ == "__main__":
    # Demo without a Myo: the wheel swings slowly left and right. Close the window to stop.
    import time
    t0 = time.monotonic()
    while not WINDOW["closed"]:
        s = math.sin(time.monotonic() - t0)
        show(s, v=0.3 + 0.7 * abs(s), w=s, note="demo: the wheel swings left and right")
        time.sleep(0.05)
