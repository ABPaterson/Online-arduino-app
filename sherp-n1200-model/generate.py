#!/usr/bin/env python3
"""
SHERP N 1200 - 1:22 scale AirLoom demo model, v0.1 (quick draft print)

Generates printable STL files for every part plus an assembled preview.
Run:  pip install manifold3d trimesh numpy matplotlib
      python3 generate.py
Output goes to ./stl and ./renders.

All dimensions in mm. Coordinate system (assembled model):
  +X = forward (nose), +Y = left side, +Z = up, origin = ground under the
  centre of the vehicle.

Real-vehicle reference (manufacturer data, see README for sources):
  length 4002, width 2520, height 2852, wheelbase 2118, tyre 1800 x 600.
"""
import math
import os

import manifold3d as mf
import numpy as np
import trimesh

M = mf.Manifold
CS = mf.CrossSection

# --------------------------------------------------------------------------
# Scale and key dimensions
# --------------------------------------------------------------------------
SCALE = 22.0
REAL = dict(length=4002, width=2520, height=2852, wheelbase=2118,
            tyre_d=1800, tyre_w=600)

WHEELBASE = REAL["wheelbase"] / SCALE          # 96.3
TYRE_R = REAL["tyre_d"] / SCALE / 2            # 40.9
TYRE_W = REAL["tyre_w"] / SCALE                # 27.3
AXLE_X = WHEELBASE / 2                         # 48.1
AXLE_Z = TYRE_R                                # wheel sits on the ground

WALL = 1.6          # 4 perimeters at 0.4 mm
BODY_HALF = 28.5    # body half-width (57 mm wide body between the tyres)
IN_HALF = BODY_HALF - WALL
BOSS_OUT = 1.5      # wheel spacer boss outside the body side
DECK_Z = 60.0       # underside of the floor / deck
DECK_TOP = DECK_Z + WALL   # split line between hull and cabin
HULL_BOTTOM = 27.0  # ground clearance ~600 mm real

LED_POCKET = (2.4, 1.6, 1.2)   # fits an 0603 LED (1.6 x 0.8) with glue room
WIRE_R = 0.9                   # 1.8 mm wire hole (litz pair fits easily)
DUCT_OR, DUCT_IR = 2.5, 1.5    # 3 mm bore wiring ducts

SEG = 48

# Side profile of the body (x forward, z up). Single outline; the hull is
# the part below DECK_TOP and the cabin the part above.
PROFILE = [
    (-80.0, HULL_BOTTOM),   # rear underside
    (72.0, HULL_BOTTOM),    # front underside
    (89.0, 40.0),           # bow slope
    (91.0, 50.0),           # top of bow / headlight panel
    (88.0, 62.0),           # base of front door / windscreen
    (64.0, 118.0),          # top of windscreen (raked)
    (58.0, 126.0),          # roof front
    (-86.0, 126.0),         # roof rear
    (-91.0, 119.0),         # rear roof edge
    (-91.0, 40.0),          # rear face
]
ROOF_Z = 126.0


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(x, y, z0, z1, r, r2=None, seg=SEG):
    return M.cylinder(z1 - z0, r, r if r2 is None else r2, seg).translate([x, y, z0])


def cyl_x(x0, x1, y, z, r, seg=SEG):
    return M.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


def cyl_y(y0, y1, x, z, r, seg=SEG):
    # cylinder along +Y from y0 to y1
    return M.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])


def side_prism(pts_xz, y0, y1):
    """Extrude a polygon drawn in the X-Z plane across Y from y0 to y1."""
    cs = CS([pts_xz])
    return side_prism_cs(cs, y0, y1)


def side_prism_cs(cs, y0, y1):
    # extrude along local z, then rotate so local y->Z and local z->-Y
    solid = cs.extrude(y1 - y0)
    solid = solid.rotate([90, 0, 0])   # (x, y, z) -> (x, -z, y)
    return solid.translate([0, y1, 0])


def union(parts):
    out = None
    for p in parts:
        out = p if out is None else out + p
    return out


def front_face_x(z):
    """X of the outer front (windscreen) face at height z, 62 <= z <= 118."""
    (x0, z0), (x1, z1) = (88.0, 62.0), (64.0, 118.0)
    return x0 + (z - z0) * (x1 - x0) / (z1 - z0)


# --------------------------------------------------------------------------
# Body (outer + inner solids)
# --------------------------------------------------------------------------
OUTER_CS = CS([PROFILE])
INNER_CS = OUTER_CS.offset(-WALL, mf.JoinType.Miter)
OUTER = side_prism_cs(OUTER_CS, -BODY_HALF, BODY_HALF)
INNER = side_prism_cs(INNER_CS, -IN_HALF, IN_HALF)

# Front A-pillar duct: straight line parallel to the raked windscreen, 4.1 mm
# behind its outer face, running from below the deck out through the roof
# just behind the front roof edge (where the light bar sits).
_d = np.array([64.0 - 88.0, 118.0 - 62.0]); _d /= np.linalg.norm(_d)
_n = np.array([-_d[1], _d[0]])                 # points back into the body
if _n[0] > 0:
    _n = -_n
FRONT_DUCT_BASE = np.array([88.0, 62.0]) + (WALL + DUCT_OR) * _n


def front_duct_x(z):
    t = (z - FRONT_DUCT_BASE[1]) / _d[1]
    return FRONT_DUCT_BASE[0] + t * _d[0]


DUCT_Y = IN_HALF - DUCT_OR                         # 24.4
REAR_DUCT_X = -91.0 + WALL + DUCT_OR               # -86.9
FRONT_DUCT_TOP_X = front_duct_x(ROOF_Z)            # where it exits the roof
FRONT_DUCT_DECK_X = front_duct_x(DECK_TOP)


def slanted_cyl(z0, z1, y, r):
    """Cylinder following the front duct line between heights z0..z1."""
    length = (z1 - z0) / _d[1]
    c = M.cylinder(length, r, r, 32)
    # tilt: rotate about Y so +Z aligns with (d_x, 0, d_z)
    ang = math.degrees(math.atan2(_d[0], _d[1]))   # negative (leans back)
    c = c.rotate([0, ang, 0])
    return c.translate([front_duct_x(z0), y, z0])


# --------------------------------------------------------------------------
# HULL  (z 27 .. 61.6, deck on top, electronics bay inside)
# --------------------------------------------------------------------------
def make_hull():
    hull_clip = box(-120, 120, -60, 60, HULL_BOTTOM - 1, DECK_TOP)
    bay_clip = box(-120, 120, -60, 60, HULL_BOTTOM + WALL, DECK_Z)
    shell = (OUTER ^ hull_clip) - (INNER ^ bay_clip)

    add = []
    # front bumper bar + rear bumper bar (towing bumpers on the real vehicle)
    add.append(box(78.0, 91.5, -25.0, 25.0, 33.0, 39.0))
    add.append(box(-92.5, -86.0, -25.0, 25.0, 33.0, 39.0))
    # spotlight pods sitting on the front bumper
    for y in (-16.0, 16.0):
        add.append(cyl_x(84.0, 93.0, y, 42.5, 3.5))
    # winch fairlead frame on the bow
    add.append(box(88.0, 92.0, -7.0, 7.0, 40.0, 49.0))
    # wheel spacer bosses (outside) and screw bosses (inside)
    for x in (-AXLE_X, AXLE_X):
        for s in (-1, 1):
            if s > 0:
                add.append(cyl_y(BODY_HALF - 0.5, BODY_HALF + BOSS_OUT, x, AXLE_Z, 4.0))
                add.append(cyl_y(22.5, IN_HALF + 0.1, x, AXLE_Z, 4.0))
            else:
                add.append(cyl_y(-BODY_HALF - BOSS_OUT, -BODY_HALF + 0.5, x, AXLE_Z, 4.0))
                add.append(cyl_y(-IN_HALF - 0.1, -22.5, x, AXLE_Z, 4.0))
    # belly-plate screw posts (full height, merged into the side walls)
    for x in (-55.0, 55.0):
        for y in (-DUCT_Y, DUCT_Y):
            add.append(cyl_z(x, y, HULL_BOTTOM + 0.5, DECK_Z + 0.1, 2.6))
    # USB-C collar inside the rear wall
    add.append(box(-91.0 + WALL - 0.1, -91.0 + WALL + 2.0, -6.6, 6.6, 42.3, 49.7))
    # cabin locating rim on top of the deck (cabin shell slides over it)
    rim_out = side_prism_cs(INNER_CS.offset(-0.25, mf.JoinType.Miter),
                            -IN_HALF + 0.25, IN_HALF - 0.25)
    rim_in = side_prism_cs(INNER_CS.offset(-1.45, mf.JoinType.Miter),
                           -IN_HALF + 1.45, IN_HALF - 1.45)
    rim = (rim_out - rim_in) ^ box(-120, 120, -60, 60, DECK_TOP - 0.1, DECK_TOP + 1.2)
    # keep the rim clear of the four duct corners
    for x in (FRONT_DUCT_DECK_X, REAR_DUCT_X):
        for y in (-DUCT_Y, DUCT_Y):
            rim = rim - cyl_z(x, y, DECK_TOP - 1, DECK_TOP + 3, 3.4)
    add.append(rim)

    hull = shell + union(add)

    cut = []
    # belly opening
    cut.append(box(-72.0, 68.0, -22.0, 22.0, HULL_BOTTOM - 2, HULL_BOTTOM + WALL + 0.5))
    # belly screw pilot holes (M2 self-tapping)
    for x in (-55.0, 55.0):
        for y in (-DUCT_Y, DUCT_Y):
            cut.append(cyl_z(x, y, HULL_BOTTOM - 1, HULL_BOTTOM + 9, 0.8, seg=16))
    # wheel screw pilot holes (M3 self-tapping)
    for x in (-AXLE_X, AXLE_X):
        cut.append(cyl_y(-40, -21.0, x, AXLE_Z, 1.25, seg=20))
        cut.append(cyl_y(21.0, 40, x, AXLE_Z, 1.25, seg=20))
    # winch fairlead slot
    cut.append(box(75.0, 95.0, -4.5, 4.5, 42.0, 47.0))
    # headlights: shallow bezel + LED pocket + wire hole into the bay
    for y in (-19.0, 19.0):
        z = 55.0
        cut.append(box(89.3, 95.0, y - 3.5, y + 3.5, z - 2.0, z + 2.0))     # 0.5 bezel
        cut.append(box(88.6, 95.0, y - 1.2, y + 1.2, z - 0.8, z + 0.8))     # LED pocket
        cut.append(cyl_x(80.0, 92.0, y, z, WIRE_R, 16))
    # spotlights: lens recess + LED pocket + wire hole
    for y in (-16.0, 16.0):
        cut.append(cyl_x(92.0, 94.0, y, 42.5, 2.6))
        cut.append(box(92.0 - LED_POCKET[2], 94.0, y - 1.2, y + 1.2, 41.7, 43.3))
        cut.append(cyl_x(80.0, 93.0, y, 42.5, WIRE_R, 16))
    # tail lights (rear face)
    for y in (-21.0, 21.0):
        z = 52.0
        cut.append(box(-95.0, -90.4, y - 2.0, y + 2.0, z - 3.5, z + 3.5))
        cut.append(box(-95.0, -89.6, y - 0.8, y + 0.8, z - 1.2, z + 1.2))
        cut.append(cyl_x(-93.0, -84.0, y, z, WIRE_R, 16))
    # USB-C charge port (rear face, centre)
    cut.append(box(-95.0, -80.0, -4.8, 4.8, 44.1, 47.9))
    # duct pass-through holes in the deck
    for x in (FRONT_DUCT_DECK_X, REAR_DUCT_X):
        for y in (-DUCT_Y, DUCT_Y):
            cut.append(cyl_z(x, y, DECK_Z - 2, DECK_TOP + 2, DUCT_IR + 0.2, seg=20))
    # fridge wire hole in the deck
    cut.append(cyl_z(FRIDGE_LED[0], FRIDGE_LED[1], DECK_Z - 2, DECK_TOP + 2, 1.2, seg=20))
    # spare 5 mm pass-through in the cab floor (dash Hub / future accessories)
    cut.append(cyl_z(48.0, 0.0, DECK_Z - 2, DECK_TOP + 2, 2.5, seg=24))

    return hull - union(cut)


# --------------------------------------------------------------------------
# CABIN SHELL  (z 61.6 .. 126, printed roof-down)
# --------------------------------------------------------------------------
WINDOWS_SIDE = [
    [(36.0, 82.0), (front_face_x(82.0) - 9.0, 82.0),
     (front_face_x(114.0) - 9.0, 114.0), (36.0, 114.0)],      # cab door window
    [(2.0, 82.0), (28.0, 82.0), (28.0, 114.0), (2.0, 114.0)],
    [(-30.0, 82.0), (-4.0, 82.0), (-4.0, 114.0), (-30.0, 114.0)],
    [(-62.0, 82.0), (-36.0, 82.0), (-36.0, 114.0), (-62.0, 114.0)],
]


def make_cabin():
    clip = box(-120, 120, -60, 60, DECK_TOP, ROOF_Z + 1)
    shell = (OUTER ^ clip) - (INNER ^ box(-120, 120, -60, 60, DECK_TOP - 1, ROOF_Z - WALL))

    add = []
    # front A-pillar ducts (follow the windscreen rake)
    for y in (-DUCT_Y, DUCT_Y):
        add.append(slanted_cyl(DECK_TOP, ROOF_Z - 0.5, y, DUCT_OR) ^ OUTER)
    # rear corner ducts
    for y in (-DUCT_Y, DUCT_Y):
        add.append(cyl_z(REAR_DUCT_X, y, DECK_TOP, ROOF_Z - 0.5, DUCT_OR, seg=32))
    cabin = shell + union(add)

    cut = []
    for w in WINDOWS_SIDE:
        cut.append(side_prism(w, -40, 40))
    # windscreen in the front door
    z0, z1 = 70.0, 112.0
    ws = [(front_face_x(z0) - 6, z0), (front_face_x(z0) + 6, z0),
          (front_face_x(z1) + 6, z1), (front_face_x(z1) - 6, z1)]
    cut.append(side_prism(ws, -20.5, 20.5))
    # rear door window
    cut.append(box(-96.0, -86.0, -15.0, 15.0, 80.0, 112.0))
    # duct bores (run out through the roof)
    for y in (-DUCT_Y, DUCT_Y):
        cut.append(slanted_cyl(DECK_TOP - 2, ROOF_Z + 3, y, DUCT_IR))
        cut.append(cyl_z(REAR_DUCT_X, y, DECK_TOP - 2, ROOF_Z + 3, DUCT_IR, seg=24))
    # roof escape hatch outline (0.6 mm groove)
    hx, hy, hs = 10.0, 0.0, 24.0
    groove = box(hx - hs / 2, hx + hs / 2, hy - hs / 2, hy + hs / 2, ROOF_Z - 0.6, ROOF_Z + 1) - \
        box(hx - hs / 2 + 0.8, hx + hs / 2 - 0.8, hy - hs / 2 + 0.8, hy + hs / 2 - 0.8, ROOF_Z - 1, ROOF_Z + 2)
    cut.append(groove)
    # front-door outline groove around the windscreen (on the raked face)
    door_o = [(front_face_x(64.0) - 3, 64.0), (front_face_x(64.0) + 3, 64.0),
              (front_face_x(116.0) + 3, 116.0), (front_face_x(116.0) - 3, 116.0)]
    door_i = [(front_face_x(64.8) - 4, 64.8), (front_face_x(64.8) + 4, 64.8),
              (front_face_x(115.2) + 4, 115.2), (front_face_x(115.2) - 4, 115.2)]
    door = side_prism(door_o, -24.0, 24.0) - side_prism(door_i, -23.2, 23.2)
    # only the outer skin, 0.6 mm deep
    door = door - side_prism_cs(OUTER_CS.offset(-0.6, mf.JoinType.Miter), -60, 60)
    cut.append(door)
    # rear door outline
    rd = box(-96.0, -90.4, -20.0, 20.0, DECK_TOP + 1, 121.0) - \
        box(-97.0, -89.0, -19.2, 19.2, DECK_TOP, 120.2)
    cut.append(rd)

    return cabin - union(cut)


# --------------------------------------------------------------------------
# BELLY PLATE (removable, carries the electronics)
# --------------------------------------------------------------------------
BATT = (52.0, 32.0)          # inner pocket for a 1S LiPo up to ~50 x 30 x 8
BATT_CX = -20.0
ESP = (23.2, 18.6)           # ESP32-C3 SuperMini 22.5 x 18
ESP_CX = 25.0
SERVO_SLOT = (40.0, 65.0)    # rails either side of the winch servo (23 mm long)


def make_belly():
    z0 = HULL_BOTTOM - WALL
    plate = box(-74.0, 70.0, -26.0, 26.0, z0, HULL_BOTTOM)
    add = []
    # battery cradle
    bx0, bx1 = BATT_CX - BATT[0] / 2, BATT_CX + BATT[0] / 2
    by = BATT[1] / 2
    add.append(box(bx0 - 1.2, bx1 + 1.2, -by - 1.2, by + 1.2, HULL_BOTTOM, HULL_BOTTOM + 6) -
               box(bx0, bx1, -by, by, HULL_BOTTOM - 1, HULL_BOTTOM + 7) -
               box(bx0 + 10, bx1 - 10, -by - 3, by + 3, HULL_BOTTOM + 2, HULL_BOTTOM + 7))
    # ESP32 cradle (open at the -X end for its USB-C)
    ex0, ex1 = ESP_CX - ESP[0] / 2, ESP_CX + ESP[0] / 2
    ey = ESP[1] / 2
    add.append(box(ex0, ex1 + 1.2, -ey - 1.2, ey + 1.2, HULL_BOTTOM, HULL_BOTTOM + 3) -
               box(ex0 - 1, ex1, -ey, ey, HULL_BOTTOM - 1, HULL_BOTTOM + 4))
    # winch servo rails
    for x in SERVO_SLOT:
        add.append(box(x - 1.2, x, -12.0, 12.0, HULL_BOTTOM, HULL_BOTTOM + 3))
    # power switch frame (SS12D00 slide switch)
    add.append(box(-65.0, -54.4, -2.6, 2.6, HULL_BOTTOM, HULL_BOTTOM + 3.5) -
               box(-64.0, -55.4, -1.85, 1.85, HULL_BOTTOM - 1, HULL_BOTTOM + 4))
    belly = plate + union(add)

    cut = []
    for x in (-55.0, 55.0):
        for y in (-DUCT_Y, DUCT_Y):
            cut.append(cyl_z(x, y, z0 - 1, HULL_BOTTOM + 1, 1.15, seg=16))
            cut.append(cyl_z(x, y, z0 - 0.01, z0 + 1.2, 2.2, 1.0, seg=24))  # countersink
    # switch actuator slot
    cut.append(box(-62.5, -56.9, -0.9, 0.9, z0 - 1, HULL_BOTTOM + 1))
    # cable-tie slots for the battery and the servo
    for y in (-15.5, 15.5):
        cut.append(box(BATT_CX - 2, BATT_CX + 2, y - 0.9, y + 0.9, z0 - 1, HULL_BOTTOM + 1))
        cut.append(box(50.5, 54.5, y - 0.9, y + 0.9, z0 - 1, HULL_BOTTOM + 1))
    # finger notch to lift the plate off
    cut.append(cyl_z(-74.0, 0, z0 - 1, HULL_BOTTOM + 1, 4.0))
    return belly - union(cut)


# --------------------------------------------------------------------------
# WHEELS
# --------------------------------------------------------------------------
def make_wheel():
    """One wheel, axis along Z, inner face on the bed (z=0)."""
    r, w, ch = TYRE_R - 1.0, TYRE_W, 5.0
    seg = 96
    body = M.cylinder(ch, r - ch, r, seg) + \
        M.cylinder(w - 2 * ch, r, r, seg).translate([0, 0, ch]) + \
        M.cylinder(ch, r, r - ch, seg).translate([0, 0, w - ch])
    lugs = []
    n = 18
    for row, (za, zb) in enumerate(((ch + 0.3, w / 2 - 0.4), (w / 2 + 0.4, w - ch - 0.3))):
        for i in range(n):
            a = 360.0 / n * (i + 0.5 * row)
            lug = box(r - 0.6, r + 1.4, -3.0, 3.0, za, zb).rotate([0, 0, a])
            lugs.append(lug)
    wheel = body + union(lugs)
    # outer face: rim dish, hub, bolt bumps
    top = w
    wheel = wheel - cyl_z(0, 0, top - 3.0, top + 1, 25.0, seg=seg)
    wheel = wheel + cyl_z(0, 0, top - 3.1, top - 0.6, 7.0, seg=48)
    for i in range(8):
        a = math.radians(45 * i)
        wheel = wheel + cyl_z(15 * math.cos(a), 15 * math.sin(a), top - 3.1, top - 2.0, 1.4, seg=16)
    # screw hole + counterbore
    wheel = wheel - cyl_z(0, 0, -1, top + 1, 1.7, seg=24)
    wheel = wheel - cyl_z(0, 0, top - 3.5, top + 1, 3.1, seg=32)
    return wheel


# --------------------------------------------------------------------------
# Roof accessories and interior
# --------------------------------------------------------------------------
BAR_LEN = 2 * (DUCT_Y + 2.5)   # spans both front duct exits


def make_lightbar():
    """Light bar, printed groove-up. Local: X depth, Y length, Z height; base on z=0."""
    L, D, H = BAR_LEN, 5.0, 4.6
    bar = box(-D / 2, D / 2, -L / 2, L / 2, 0, H)
    cut = [box(-1.2, 1.2, -L / 2 - 1, L / 2 + 1, -1, 2.4)]      # wire groove underneath
    for y in (-14.0, 0.0, 14.0):                                 # 3 LED pockets
        cut.append(box(D / 2 - LED_POCKET[2], D / 2 + 1, y - 1.2, y + 1.2, 2.5, 4.1))
        cut.append(cyl_x(0, D / 2, y, 3.3, 0.7, 12))
        cut.append(cyl_z(0, y, 0, 3.5, 0.7, seg=12))
    return bar - union(cut)


def make_beacon():
    """Amber beacon base. LED sits in the top pocket under a dob of clear glue."""
    b = cyl_z(0, 0, 0, 2.0, 3.6, seg=40) + cyl_z(0, 0, 2.0, 5.5, 2.8, seg=40)
    b = b - box(-1.2, 1.2, -0.8, 0.8, 5.5 - LED_POCKET[2], 6)
    b = b - cyl_z(0, 0, -1, 6, 0.8, seg=12)
    return b


FRIDGE = (32.0, 18.0, 20.0)       # ~700 x 400 x 440 mm real (60 L class)
FRIDGE_POS = (-67.0, 13.5)        # centre on the deck, rear module, left side
FRIDGE_LED = (FRIDGE_POS[0] - 11.0, FRIDGE_POS[1])


def make_fridge():
    L, W, H = FRIDGE
    f = box(-L / 2, L / 2, -W / 2, W / 2, 0, H)
    # lid seam and handle
    f = f - (box(-L / 2 - 1, L / 2 + 1, -W / 2 - 1, W / 2 + 1, H - 5.4, H - 4.8) -
             box(-L / 2 + 0.5, L / 2 - 0.5, -W / 2 + 0.5, W / 2 - 0.5, 0, H))
    f = f + box(-L / 2 - 1.0, -L / 2, -5.0, 5.0, H - 4.0, H - 2.5)
    # control panel recess with LED pocket on the top (rear-door end)
    lx = FRIDGE_LED[0] - FRIDGE_POS[0]
    f = f - box(lx - 1.2, lx + 1.2, -0.8, 0.8, H - LED_POCKET[2], H + 1)
    f = f - cyl_z(lx, 0, -1, H + 1, 0.8, seg=12)
    # vent slots on the side
    for i in range(4):
        x = 4 + i * 2.5
        f = f - box(x, x + 1.2, W / 2 - 0.5, W / 2 + 1, 3, 9)
    return f


def make_seat():
    s = box(0, 10, -5, 5, 0, 5)                   # cushion
    s = s + box(-2.5, 0.5, -5, 5, 0, 14)          # backrest
    s = s + box(-2.0, 0, -3.5, 3.5, 14, 16.5)     # headrest
    return s


# --------------------------------------------------------------------------
# Build, export, render
# --------------------------------------------------------------------------
def to_trimesh(man):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def lay_flat(man, flip=False):
    """Put the part on z=0, centred in XY. flip=True turns it upside down."""
    if flip:
        man = man.rotate([180, 0, 0])
    bb = man.bounding_box()
    cx, cy = (bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2
    return man.translate([-cx, -cy, -bb[2]])


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_stl = os.path.join(here, "stl")
    out_png = os.path.join(here, "renders")
    os.makedirs(out_stl, exist_ok=True)
    os.makedirs(out_png, exist_ok=True)

    hull = make_hull()
    cabin = make_cabin()
    belly = make_belly()
    wheel = make_wheel()
    bar = make_lightbar()
    beacon = make_beacon()
    fridge = make_fridge()
    seat = make_seat()

    # --- assembled positions -------------------------------------------
    asm = {
        "hull": hull,
        "cabin": cabin,
        "belly": belly,
        "lightbar": bar.translate([FRONT_DUCT_TOP_X, 0, ROOF_Z]),
        "beacon": beacon.translate([REAR_DUCT_X, DUCT_Y, ROOF_Z]),
        "fridge": fridge.translate([FRIDGE_POS[0], FRIDGE_POS[1], DECK_TOP]),
        "seat_l": seat.translate([44.0, 12.0, DECK_TOP]),
        "seat_r": seat.translate([44.0, -12.0, DECK_TOP]),
    }
    for i, x in enumerate((-AXLE_X, AXLE_X)):
        # left wheel: inner face at y = +30, extends outwards to +Y
        asm[f"wheel_l{i}"] = wheel.rotate([-90, 0, 0]).translate([x, BODY_HALF + BOSS_OUT, AXLE_Z])
        asm[f"wheel_r{i}"] = wheel.rotate([90, 0, 0]).translate([x, -BODY_HALF - BOSS_OUT, AXLE_Z])

    assembled = union(asm.values())
    bb = assembled.bounding_box()
    print("Assembled model size (mm): L %.1f  W %.1f  H %.1f" %
          (bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]))

    # --- printable parts, in print orientation -------------------------
    parts = {
        "01_hull_print_deck_down": lay_flat(hull, flip=True),
        "02_cabin_print_roof_down": lay_flat(cabin, flip=True),
        "03_belly_plate": lay_flat(belly),
        "04_wheel_x4": lay_flat(wheel),
        "05_lightbar_print_groove_up": lay_flat(bar, flip=True),
        "06_beacon": lay_flat(beacon),
        "07_fridge": lay_flat(fridge),
        "08_seat_x2": lay_flat(seat),
    }
    for name, p in parts.items():
        st = p.status()
        tm = to_trimesh(p)
        b = p.bounding_box()
        path = os.path.join(out_stl, name + ".stl")
        tm.export(path)
        print("%-32s %6.1f x %6.1f x %6.1f mm  vol %7.1f cm3  watertight=%s  %s" %
              (name, b[3] - b[0], b[4] - b[1], b[5] - b[2], p.volume() / 1000,
               tm.is_watertight, st))
    to_trimesh(assembled).export(os.path.join(out_stl, "00_assembled_preview_DO_NOT_PRINT.stl"))

    render(asm, out_png)


def render(asm, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    colours = {
        "hull": "#5b6b4a", "cabin": "#7d8c63", "belly": "#333333",
        "lightbar": "#222222", "beacon": "#e08a00", "fridge": "#dddddd",
        "seat_l": "#444444", "seat_r": "#444444",
    }
    views = [("iso_front", 22, 35), ("iso_rear", 22, 215), ("side", 0, 90),
             ("front", 0, 0), ("top", 89, 90)]
    meshes = []
    for k, man in asm.items():
        tm = to_trimesh(man)
        meshes.append((tm, colours.get(k, "#1c1c1c")))
    light = np.array([0.4, 0.3, 0.85]); light /= np.linalg.norm(light)
    # cutaway: keep the right-hand half (y <= 0) so ducts, deck and bay show
    cut_half = box(-120, 120, -80, 0.0, -10, 200)
    cut_meshes = []
    for k, man in asm.items():
        if k.startswith("wheel"):
            continue
        cut_meshes.append((to_trimesh(man ^ cut_half), colours.get(k, "#1c1c1c")))
    jobs = [(n, e, a, meshes) for n, e, a in views] + [("cutaway_from_left", 8, 70, cut_meshes)]
    for name, elev, azim, meshes in jobs:
        fig = plt.figure(figsize=(10, 7), dpi=110)
        ax = fig.add_subplot(111, projection="3d")
        for tm, col in meshes:
            tris = tm.vertices[tm.faces]
            nrm = tm.face_normals
            shade = 0.45 + 0.55 * np.clip(nrm @ light, 0, 1)
            base = np.array(matplotlib.colors.to_rgb(col))
            fc = np.clip(base[None, :] * shade[:, None], 0, 1)
            pc = Poly3DCollection(tris, facecolors=fc, edgecolors="none", linewidths=0)
            ax.add_collection3d(pc)
        ax.set_xlim(-78, 78); ax.set_ylim(-78, 78); ax.set_zlim(-12, 144)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title("SHERP N 1200 - 1:%d AirLoom demo model v0.1 (%s)" % (SCALE, name))
        fig.tight_layout()
        fig.savefig(os.path.join(out_png, name + ".png"))
        plt.close(fig)


if __name__ == "__main__":
    main()
