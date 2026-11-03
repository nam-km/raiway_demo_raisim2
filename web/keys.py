#!/usr/bin/env python3
"""Sends the held arrow/WASD keys of this computer to raiway_demo on another one.

    python3 web/keys.py <host of raiway_demo> [port, 8081]

Needs pynput (pip install pynput). Keys are read system-wide, so the browser viewer can keep the focus.
While up/down or W/S is held the robot drives, while left/right or A/D is held it turns; R resets.
The state goes out 20 times a second as "forward turn reset"; the server releases the keys half a second
after the last message, so closing this script stops the robot.
"""
import socket
import sys
import time

try:
    from pynput import keyboard
except ImportError:
    sys.exit("pynput is missing: pip install pynput")

if len(sys.argv) < 2:
    sys.exit(__doc__)
host, port = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 8081

held = set()
NAMES = {keyboard.Key.up: "up", keyboard.Key.down: "down", keyboard.Key.left: "left", keyboard.Key.right: "right"}


def name(key):
    if key in NAMES:
        return NAMES[key]
    char = getattr(key, "char", None)
    return char.lower() if char else None


def on_press(key):
    n = name(key)
    if n:
        held.add(n)


def on_release(key):
    held.discard(name(key))


keyboard.Listener(on_press=on_press, on_release=on_release).start()
out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("sending held keys to %s:%d; up/down or W/S drive, left/right or A/D turn, R resets, Ctrl+C quits" % (host, port))
try:
    while True:
        forward = (("up" in held) or ("w" in held)) - (("down" in held) or ("s" in held))
        turn = (("left" in held) or ("a" in held)) - (("right" in held) or ("d" in held))
        out.sendto(("%d %d %d" % (forward, turn, "r" in held)).encode(), (host, port))
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
