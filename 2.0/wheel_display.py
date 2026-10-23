#!/usr/bin/env python3
"""Steering-wheel window: shows the driver how far they twist and which way the robot turns.

Add it to any controller with two lines:
    import wheel_display                              # at the top of the controller
    wheel_display.show(steer, v=v, w=w, note=note)    # once per control tick (for example at the end of step())

    steer : -1 .. 1, + = left (the twist after calibration). Moves the hand on the wheel.
    v, w  : the command sent (m/s, rad/s): speed as text and a bar, turn rate as text (optional)
    note  : a short message, for example "ASSIST: obstacle ahead" (optional)

The window opens on the first call. Closing it only hides the wheel; the controller keeps running.
Try it without a Myo:   python examples/wheel_display.py   (the wheel swings left and right)
Saves nothing. Needs pygame (already in the hackathon conda environment).
"""
import math
import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")   # no pygame banner in the terminal
import pygame

SIZE = (420, 570)                   # px, window width and height
MAX_ANGLE = 90                      # deg, wheel rotation at a full twist (steer = ±1)
V_SCALE = 2.0                       # m/s, speed that fills the speed bar (the controller's V_MAX)
BG, RIM, HUB, TEXT = (24, 26, 32), (200, 200, 210), (90, 92, 100), (235, 235, 235)
LEFT_COLOR, RIGHT_COLOR, CENTER_COLOR = (80, 140, 255), (230, 90, 230), (240, 240, 240)   # blue / magenta as in the terminal
NOTE_COLOR = (250, 210, 60)
FORWARD_COLOR, BACK_COLOR = (90, 220, 120), (240, 80, 80)   # green / red as in the terminal

WINDOW = {"screen": None, "big": None, "mid": None, "small": None, "closed": False}


def rim_point(cx: int, cy: int, radius: float, a: float) -> tuple[int, int]:
    """Point on a circle; a = 0 is the top (12 o'clock), a > 0 turns counter-clockwise (to the left)."""
    return round(cx - radius * math.sin(a)), round(cy - radius * math.cos(a))


def show(steer: float, v: float | None = None, w: float | None = None, note: str = "") -> None:
    """Draw one frame of the wheel. Call it every control tick (20 Hz is plenty)."""
    if WINDOW["closed"]:
        return
    if WINDOW["screen"] is None:
        pygame.init()
        WINDOW["screen"] = pygame.display.set_mode(SIZE)
        pygame.display.set_caption("Steering wheel")
        WINDOW["big"] = pygame.font.SysFont(None, 60)
        WINDOW["mid"] = pygame.font.SysFont(None, 40)
        WINDOW["small"] = pygame.font.SysFont(None, 28)
    for event in pygame.event.get():          # keeps the window responsive; closing it hides the wheel
        if event.type == pygame.QUIT:
            pygame.display.quit()
            WINDOW["closed"] = True
            return

    screen = WINDOW["screen"]
    steer = max(-1.0, min(1.0, float(steer)))
    color = LEFT_COLOR if steer > 0.02 else RIGHT_COLOR if steer < -0.02 else CENTER_COLOR
    cx, cy, r = SIZE[0] // 2, 240, 150        # wheel centre and radius, px
    angle = math.radians(MAX_ANGLE * steer)   # the whole wheel turns with the twist

    screen.fill(BG)
    # Fixed mark at the top: where the hand sits when you drive straight.
    pygame.draw.polygon(screen, TEXT, [(cx, cy - r - 12), (cx - 10, cy - r - 30), (cx + 10, cy - r - 30)])
    # Wheel: rim, three spokes (left, right, bottom) and the hub. All turn by `angle`.
    pygame.draw.circle(screen, RIM, (cx, cy), r, 18)
    for spoke in (angle + math.pi / 2, angle - math.pi / 2, angle + math.pi):
        pygame.draw.line(screen, HUB, (cx, cy), rim_point(cx, cy, r - 9, spoke), 14)
    pygame.draw.circle(screen, HUB, (cx, cy), 34)
    # The hand: a dot on the rim that starts at the top and moves with the twist.
    pygame.draw.circle(screen, color, rim_point(cx, cy, r - 9, angle), 22)
    pygame.draw.circle(screen, TEXT, rim_point(cx, cy, r - 9, angle), 22, 3)

    label = f"LEFT {steer:.0%}" if steer > 0.02 else f"RIGHT {-steer:.0%}" if steer < -0.02 else "STRAIGHT"
    text = WINDOW["big"].render(label, True, color)
    screen.blit(text, text.get_rect(center=(cx, 435)))
    if v is not None:
        # Speed: text and a bar, green forward / red back; the bar is full at V_SCALE.
        bar_color = FORWARD_COLOR if v >= 0 else BACK_COLOR
        text = WINDOW["mid"].render(f"{'FORWARD' if v >= 0 else 'BACK'}  {abs(v):.2f} m/s", True, bar_color)
        screen.blit(text, text.get_rect(center=(cx, 482)))
        pygame.draw.rect(screen, HUB, (40, 505, SIZE[0] - 80, 22), 2)
        pygame.draw.rect(screen, bar_color, (42, 507, round((SIZE[0] - 84) * min(abs(v) / V_SCALE, 1.0)), 18))
    if w is not None:
        text = WINDOW["small"].render(f"turn rate {w:+.2f} rad/s", True, TEXT)
        screen.blit(text, text.get_rect(center=(cx, 550)))
    if note:
        text = WINDOW["small"].render(note, True, NOTE_COLOR)
        screen.blit(text, text.get_rect(center=(cx, 30)))
    pygame.display.flip()


if __name__ == "__main__":
    # Demo without a Myo: the wheel swings slowly left and right. Close the window to stop.
    import time
    t0 = time.monotonic()
    while not WINDOW["closed"]:
        s = math.sin(time.monotonic() - t0)
        show(s, v=0.3, w=s, note="demo: the wheel swings left and right")
        time.sleep(0.05)
