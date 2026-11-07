#!/usr/bin/env python3
"""
SHERP N 1200 - 1:22 scale AirLoom model, v0.3
One hull for both a static display and an outdoor, floating RC version.

  * Sealed lower hull (tub) holds 4 x N20 drive motors (skid steer like the
    real thing), battery and electronics. Waterproof winch servo sits in a
    flooded pocket in the bow.
  * Removable cab: upper body glued to a deck lid; the lid seals on the tub
    rim with a printed TPU gasket and 4 x M3 screws from underneath.
  * Body shape from the SHERP N 1200 spec sheet + photos: wide cab sitting
    over the tyres, narrow hull between them, near-vertical front door,
    roof A/C, light bar + spotlights, rear work lights, beacon.
  * Wheels: PLA hub (D-bore onto the N20 shaft) + TPU paddle tyre + PLA cap.

Run:  pip install manifold3d trimesh numpy matplotlib
      python3 generate.py
Writes ./stl, ./3mf (Bambu A1 project) and ./renders.

Coordinates (assembled): +X forward, +Y left, +Z up, origin on the ground
under the vehicle centre. All mm.
"""
import math
import os

import manifold3d as mf
import numpy as np
import trimesh

import bambu_3mf

M = mf.Manifold
CS = mf.CrossSection
VERSION = "v0.3"
SEG = 48

# --------------------------------------------------------------------------
# Real vehicle (SHERP N 1200 spec sheet) and scale
# --------------------------------------------------------------------------
SCALE = 22.0
REAL = dict(length=4002, width=2520, height=2825.5, wheelbase=2118,
            track=1900, tyre_d=1800, tyre_w=600, clearance=600)
TYRE_R = REAL["tyre_d"] / SCALE / 2            # 40.9
TYRE_W = REAL["tyre_w"] / SCALE                # 27.3
AXLE_X = REAL["wheelbase"] / SCALE / 2         # 48.1
AXLE_Z = TYRE_R

# --------------------------------------------------------------------------
# Hull (sealed tub)
# --------------------------------------------------------------------------
HULL_HALF = 28.5
HULL_WALL = 2.0             # 5 perimeters - helps keep water out
HULL_BOTTOM = REAL["clearance"] / SCALE        # 27.3
RIM_Z = 64.0
LIP = 4.5                   # rim width for the gasket
GASKET_T = 1.0
LID_Z0 = RIM_Z + GASKET_T   # 65
LID_T = 2.0
LID_Z1 = LID_Z0 + LID_T     # 67  (upper body starts here)
HULL_PROFILE = [(-80.0, HULL_BOTTOM), (72.0, HULL_BOTTOM), (89.0, 40.0),
                (91.0, 50.0), (91.0, RIM_Z), (-91.0, RIM_Z), (-91.0, 40.0)]

BOLT_X, BOLT_Y = 60.0, 23.0          # 4 x M3 x 40 from underneath
WHEEL_Y0 = HULL_HALF + 2.0           # inner face of the wheel (after spacer boss)

# N20 gear motor: 12 x 10 body, 24 long (gearbox + motor), 3 mm D shaft
N20_W, N20_H, N20_L = 12.0, 10.0, 24.0
SHAFT_R = 1.5
ORING_R, ORING_D = 3.1, 1.4          # 3 x 6 x 1.5 O-ring pocket in the wall

# winch: INJORA INJS2065-360 (23 x 13 x 25.8) in a flooded bow pocket
WINCH_POCKET = (66.0, 100.0, -15.0, 15.0, 29.5, 43.0)

# --------------------------------------------------------------------------
# Upper body (cab) - glued to the deck lid, lifts off as one unit
# --------------------------------------------------------------------------
CAB_HALF = 51.0             # wide cab over the tyres (~2250 mm real)
CAB_WALL = 1.6
SHELF_Z = 85.0              # underside of the cab over the tyres
ROOF_Z = REAL["height"] / SCALE   # 128.4
SPINE_TOP = 88.0


def cab_front_x(z):
    """Front face of the wide cab: slight rake, 86 at the shelf to 80 at z=122."""
    return 86.0 - (z - SHELF_Z) * 6.0 / 37.0


CAB_PROFILE = [(-91.0, SHELF_Z), (cab_front_x(SHELF_Z), SHELF_Z), (cab_front_x(122.0), 122.0),
               (76.0, ROOF_Z), (-87.0, ROOF_Z), (-91.0, ROOF_Z - 4.4)]
SPINE_PROFILE = [(-91.0, LID_Z1), (90.5, LID_Z1), (89.0, SPINE_TOP + 1), (-91.0, SPINE_TOP + 1)]

LED_POCKET = (2.4, 1.6, 1.0)
WIRE_R = 0.9
DUCT_OR, DUCT_IR = 3.0, 1.7          # 3.4 mm bore conduits
FRONT_DUCT_X, REAR_DUCT_X = 73.0, -84.0
DUCT_POD_Y = 40.0                    # conduits sit behind the headlights/tail lights
DUCT_DROP_Y = 24.6                   # drop down inside the spine to the lid
LIGHT_Z = 97.0                       # headlights / tail lights height
WORKLIGHT_Z = 121.0

FRIDGE = (32.0, 18.0, 20.0)
FRIDGE_POS = (-66.0, 0.0)
FRIDGE_LED = (FRIDGE_POS[0] - 11.0, FRIDGE_POS[1])
SPARE_HOLE = (48.0, 0.0)


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
    return M.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])


def side_prism_cs(cs, y0, y1):
    return cs.extrude(y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])


def side_prism(pts, y0, y1):
    return side_prism_cs(CS([pts]), y0, y1)


def union(parts):
    parts = [p for p in parts if p is not None]
    return M.batch_boolean(parts, mf.OpType.Add) if parts else None


def capsule_chain(pts, r, seg=20):
    sph = [M.sphere(r, seg).translate(list(p)) for p in pts]
    return union([M.batch_hull([a, b]) for a, b in zip(sph[:-1], sph[1:])])


def arc(c, r, a0, a1, n=8):
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def conduit_path(x, s):
    """Roof -> down the corner -> across the shelf -> down the spine -> lid."""
    r = 6.0
    zc = SHELF_Z + CAB_WALL + DUCT_OR            # 89.6, sits on the shelf
    pts = [(x, s * DUCT_POD_Y, ROOF_Z + 3.0), (x, s * DUCT_POD_Y, zc + r)]
    for (u, v) in arc((DUCT_POD_Y - r, zc + r), r, 0, -90)[1:]:
        pts.append((x, s * u, v))
    pts.append((x, s * (DUCT_DROP_Y + r), zc))
    for (u, v) in arc((DUCT_DROP_Y + r, zc - r), r, 90, 180)[1:]:
        pts.append((x, s * u, v))
    pts.append((x, s * DUCT_DROP_Y, LID_Z1 - 3.0))
    return pts


# --------------------------------------------------------------------------
# HULL - sealed tub
# --------------------------------------------------------------------------
HULL_CS = CS([HULL_PROFILE])
_open = [(x, (z if z < RIM_Z else 90.0)) for x, z in HULL_PROFILE]
HULL_INNER_CS = CS([_open]).offset(-HULL_WALL, mf.JoinType.Miter)
HULL_OUTER = side_prism_cs(HULL_CS, -HULL_HALF, HULL_HALF)


def motor_positions():
    for x in (-AXLE_X, AXLE_X):
        for s in (-1, 1):
            yield x, s


def make_hull():
    inner = side_prism_cs(HULL_INNER_CS, -HULL_HALF + HULL_WALL, HULL_HALF - HULL_WALL)
    tub = HULL_OUTER - inner
    add = []
    # gasket lip around the rim (inward ledge)
    lip_ring = (side_prism_cs(HULL_INNER_CS, -HULL_HALF + HULL_WALL + 0.01, HULL_HALF - HULL_WALL - 0.01) -
                side_prism_cs(HULL_INNER_CS.offset(-(LIP - HULL_WALL), mf.JoinType.Miter),
                              -HULL_HALF + LIP, HULL_HALF - LIP))
    add.append(lip_ring ^ box(-120, 120, -60, 60, RIM_Z - 2.5, RIM_Z))
    # lid bolt bosses (merged into the walls)
    for x in (-BOLT_X, BOLT_X):
        for y in (-BOLT_Y, BOLT_Y):
            add.append(cyl_z(x, y, HULL_BOTTOM + 0.5, RIM_Z, 3.5))
    # motor mounts: outside spacer boss, inside cradle
    for x, s in motor_positions():
        y_out0, y_out1 = sorted((s * (HULL_HALF - 0.2), s * WHEEL_Y0))
        add.append(cyl_y(y_out0, y_out1, x, AXLE_Z, 6.0))
        yi0, yi1 = sorted((s * 3.0, s * (HULL_HALF - HULL_WALL + 0.1)))
        cradle = box(x - N20_W / 2 - 1.4, x + N20_W / 2 + 1.4, yi0, yi1, HULL_BOTTOM + 1, AXLE_Z + 2.0)
        cradle = cradle - box(x - N20_W / 2 - 0.1, x + N20_W / 2 + 0.1, yi0 - 1, yi1 + 1,
                              AXLE_Z - N20_H / 2 - 0.1, AXLE_Z + 10)
        add.append(cradle)
    # winch pocket walls
    px0, px1, py0, py1, pz0, pz1 = WINCH_POCKET
    add.append(box(px0 - 2.0, 91.0, py0 - 2.0, py1 + 2.0, HULL_BOTTOM + 0.5, pz1 + 2.0) ^ HULL_OUTER)
    # front step (orange step-plate on the real one) and rear bumper / tow hitch
    add.append(box(86.0, 94.0, -24.0, 24.0, 44.0, 47.0))
    add.append(box(-93.0, -88.0, -24.0, 24.0, 33.0, 38.0))
    # ledges that carry the electronics tray above the battery
    for s in (-1, 1):
        y0, y1 = sorted((s * (HULL_HALF - HULL_WALL + 0.1), s * (HULL_HALF - HULL_WALL - 1.6)))
        add.append(box(-36.0, 36.0, y0, y1, 47.0, 48.6))
    # battery corral
    add.append(box(-34.0, 30.0, -18.6, 18.6, HULL_BOTTOM + 1, HULL_BOTTOM + 7) -
               box(-32.6, 28.6, -17.2, 17.2, HULL_BOTTOM, HULL_BOTTOM + 8))
    # boss for the power button nut
    add.append(cyl_z(-66.0, 0.0, HULL_BOTTOM + 0.5, HULL_BOTTOM + 4.0, 10.0))
    hull = tub + union(add)

    cut = []
    for x in (-BOLT_X, BOLT_X):
        for y in (-BOLT_Y, BOLT_Y):
            cut.append(cyl_z(x, y, HULL_BOTTOM - 1, RIM_Z + 1, 1.7, seg=20))
            cut.append(cyl_z(x, y, HULL_BOTTOM - 1, HULL_BOTTOM + 4.0, 3.2, seg=24))
    for x, s in motor_positions():
        y0, y1 = sorted((s * 20.0, s * (WHEEL_Y0 + 1)))
        cut.append(cyl_y(y0, y1, x, AXLE_Z, SHAFT_R + 0.2, 20))
        y0, y1 = sorted((s * (HULL_HALF - HULL_WALL - 1), s * (HULL_HALF - HULL_WALL + ORING_D)))
        cut.append(cyl_y(y0, y1, x, AXLE_Z, ORING_R, 32))
    cut.append(box(*WINCH_POCKET))
    cut.append(cyl_x(60.0, WINCH_POCKET[0] + 1, 0.0, 40.0, 1.5, 16))      # winch lead
    # power button hole through the floor (12 mm IP67 latching button)
    cut.append(cyl_z(-66.0, 0.0, HULL_BOTTOM - 1, HULL_BOTTOM + 6, 6.1, seg=48))
    return hull - union(cut)


# --------------------------------------------------------------------------
# GASKET (TPU) and DECK LID (PLA)
# --------------------------------------------------------------------------
def make_gasket():
    g = box(-90.7, 90.7, -HULL_HALF + 0.3, HULL_HALF - 0.3, 0, GASKET_T) - \
        box(-91 + LIP + 0.2, 91 - LIP - 0.2, -HULL_HALF + LIP + 0.2, HULL_HALF - LIP - 0.2, -1, 2)
    for x in (-BOLT_X, BOLT_X):
        for y in (-BOLT_Y, BOLT_Y):
            g = g + cyl_z(x, y, 0, GASKET_T, 4.2)
            g = g - cyl_z(x, y, -1, 2, 1.7, seg=20)
    return g


def make_lid():
    """Deck lid / cab floor, modelled in place (z 65..67)."""
    lid = box(-90.7, 90.7, -HULL_HALF + 0.3, HULL_HALF - 0.3, LID_Z0, LID_Z1)
    for x in (-BOLT_X, BOLT_X):
        for y in (-BOLT_Y, BOLT_Y):
            lid = lid + cyl_z(x, y, LID_Z1 - 0.1, LID_Z1 + 6.0, 3.0)
    cut = []
    for x in (-BOLT_X, BOLT_X):
        for y in (-BOLT_Y, BOLT_Y):
            cut.append(cyl_z(x, y, LID_Z0 - 1, LID_Z1 + 7, 1.25, seg=20))   # M3 self-tap
    for x in (FRONT_DUCT_X, REAR_DUCT_X):
        for s in (-1, 1):
            cut.append(cyl_z(x, s * DUCT_DROP_Y, LID_Z0 - 1, LID_Z1 + 1, 1.9, seg=20))
    cut.append(cyl_z(FRIDGE_LED[0], FRIDGE_LED[1], LID_Z0 - 1, LID_Z1 + 1, 1.2, seg=16))
    cut.append(cyl_z(SPARE_HOLE[0], SPARE_HOLE[1], LID_Z0 - 1, LID_Z1 + 1, 2.5, seg=24))
    return lid - union(cut)


# --------------------------------------------------------------------------
# UPPER BODY (cab)
# --------------------------------------------------------------------------
CAB_CS = CS([CAB_PROFILE])
SPINE_CS = CS([SPINE_PROFILE])


def make_cab():
    outer = side_prism_cs(CAB_CS, -CAB_HALF, CAB_HALF) + \
        side_prism_cs(SPINE_CS, -HULL_HALF, HULL_HALF)
    cab_in = side_prism_cs(CAB_CS.offset(-CAB_WALL, mf.JoinType.Miter),
                           -CAB_HALF + CAB_WALL, CAB_HALF - CAB_WALL)
    spine_in_pts = [(-91 + HULL_WALL, LID_Z1 - 1), (90.5 - HULL_WALL, LID_Z1 - 1),
                    (89.0 - HULL_WALL, SPINE_TOP + 3), (-91 + HULL_WALL, SPINE_TOP + 3)]
    spine_in = side_prism(spine_in_pts, -HULL_HALF + HULL_WALL, HULL_HALF - HULL_WALL)
    shell = outer - cab_in - spine_in

    add = []
    for x in (FRONT_DUCT_X, REAR_DUCT_X):
        for s in (-1, 1):
            add.append(capsule_chain(conduit_path(x, s), DUCT_OR) ^
                       box(-120, 120, -60, 60, LID_Z1, ROOF_Z))
    cab = shell + union(add)

    cut = []
    for x in (FRONT_DUCT_X, REAR_DUCT_X):
        for s in (-1, 1):
            cut.append(capsule_chain(conduit_path(x, s), DUCT_IR, 16))
    # side windows (only through the side walls, conduits are safe)
    wins = [
        [(52.0, 96.0), (cab_front_x(96.0) - 6.0, 96.0), (cab_front_x(118.0) - 6.0, 118.0), (52.0, 118.0)],
        [(8.0, 96.0), (34.0, 96.0), (34.0, 118.0), (8.0, 118.0)],
        [(-30.0, 96.0), (-4.0, 96.0), (-4.0, 118.0), (-30.0, 118.0)],
    ]
    for w in wins:
        cut.append(side_prism(w, CAB_HALF - 3.0, CAB_HALF + 3.0))
        cut.append(side_prism(w, -CAB_HALF - 3.0, -CAB_HALF + 3.0))
    # windscreen in the front door
    z0, z1 = 94.0, 119.0
    ws = [(cab_front_x(z0) - 5, z0), (cab_front_x(z0) + 5, z0),
          (cab_front_x(z1) + 5, z1), (cab_front_x(z1) - 5, z1)]
    cut.append(side_prism(ws, -19.0, 19.0))
    # small windows in the front corner pods (above the headlights)
    for s in (-1, 1):
        y0, y1 = sorted((s * 30.0, s * 47.0))
        zz0, zz1 = 104.0, 118.0
        p = [(cab_front_x(zz0) - 5, zz0), (cab_front_x(zz0) + 5, zz0),
             (cab_front_x(zz1) + 5, zz1), (cab_front_x(zz1) - 5, zz1)]
        cut.append(side_prism(p, y0, y1))
    # rear door window
    cut.append(box(-96.0, -88.0, -14.0, 14.0, 100.0, 120.0))
    # headlights (front of the pods), tail lights and work lights (rear)
    for s in (-1, 1):
        y = s * DUCT_POD_Y
        fx = cab_front_x(LIGHT_Z)
        cut.append(box(fx - 0.5, fx + 5, y - 3.0, y + 3.0, LIGHT_Z - 2.2, LIGHT_Z + 2.2))
        cut.append(box(fx - 0.5 - LED_POCKET[2], fx + 5, y - 1.2, y + 1.2, LIGHT_Z - 0.8, LIGHT_Z + 0.8))
        cut.append(cyl_x(FRONT_DUCT_X, fx, y, LIGHT_Z, WIRE_R, 16))
        cut.append(box(-96.0, -90.5, y - 2.5, y + 2.5, LIGHT_Z - 3.5, LIGHT_Z + 3.5))
        cut.append(box(-96.0, -90.5 + LED_POCKET[2], y - 0.8, y + 0.8, LIGHT_Z - 1.2, LIGHT_Z + 1.2))
        cut.append(cyl_x(-92.0, REAR_DUCT_X, y, LIGHT_Z, WIRE_R, 16))
        cut.append(box(-96.0, -90.5, y - 3.5, y + 3.5, WORKLIGHT_Z - 2.5, WORKLIGHT_Z + 2.5))
        cut.append(box(-96.0, -90.5 + LED_POCKET[2], y - 1.2, y + 1.2, WORKLIGHT_Z - 0.8, WORKLIGHT_Z + 0.8))
        cut.append(cyl_x(-92.0, REAR_DUCT_X, y, WORKLIGHT_Z, WIRE_R, 16))
    # front door outline (0.6 mm groove on the cab front)
    for z in (93.0, 121.0):
        cut.append(side_prism([(cab_front_x(z) - 0.6, z - 0.4), (cab_front_x(z) + 3, z - 0.4),
                               (cab_front_x(z) + 3, z + 0.4), (cab_front_x(z) - 0.6, z + 0.4)], -22.0, 22.0))
    for y in (-22.0, 22.0):
        cut.append(side_prism([(cab_front_x(SHELF_Z) - 0.6, SHELF_Z), (cab_front_x(SHELF_Z) + 3, SHELF_Z),
                               (cab_front_x(121.4) + 3, 121.4), (cab_front_x(121.4) - 0.6, 121.4)],
                              y - 0.4, y + 0.4))
    # roof hatches (front and rear) as grooves
    for hx in (40.0, -50.0):
        cut.append(box(hx - 11, hx + 11, -11, 11, ROOF_Z - 0.6, ROOF_Z + 1) -
                   box(hx - 10.2, hx + 10.2, -10.2, 10.2, ROOF_Z - 1, ROOF_Z + 2))
    # side vent grille on the rear module (orange panel on the real one)
    for s in (-1, 1):
        for i in range(5):
            z = 88.5 + i * 1.6
            y0, y1 = sorted((s * (CAB_HALF - 0.6), s * (CAB_HALF + 1)))
            cut.append(box(-50.0, -38.0, y0, y1, z, z + 0.7))
    return cab - union(cut)


# --------------------------------------------------------------------------
# WHEELS: PLA hub + TPU tyre + PLA cap
# --------------------------------------------------------------------------
HUB_R = 17.0
HUB_FLANGE_R, HUB_FLANGE_T = 19.5, 1.6
CAP_T = 1.6
KEYS, KEY_H, KEY_W = 8, 1.0, 2.4
TYRE_FIT = 0.05
SPIGOT_R, SPIGOT_H = 12.0, 2.0
TYRE_OD_R = TYRE_R - 0.3
TYRE_BASE_R = TYRE_OD_R - 2.8        # deep paddle lugs (they propel it in water)
TYRE_LEN = TYRE_W - HUB_FLANGE_T


def make_hub():
    w = TYRE_W
    hub = cyl_z(0, 0, 0, HUB_FLANGE_T, HUB_FLANGE_R, seg=96) + cyl_z(0, 0, 0, w, HUB_R, seg=96)
    for i in range(KEYS):
        hub = hub + box(HUB_R - 0.5, HUB_R + KEY_H, -KEY_W / 2, KEY_W / 2,
                        HUB_FLANGE_T, w - 0.6).rotate([0, 0, 360.0 / KEYS * i])
    hub = hub - cyl_z(0, 0, w - SPIGOT_H, w + 1, SPIGOT_R, seg=64)
    for i in range(6):
        a = math.radians(60 * i + 30)
        hub = hub - cyl_z(8.0 * math.cos(a), 8.0 * math.sin(a), 3.0, w + 1, 2.6, seg=24)
    # N20 D-shaft bore (3 mm, flat at 2.5): press fit, a drop of CA to finish
    bore = cyl_z(0, 0, -1, 12.0, 1.55, seg=32) ^ box(-2, 1.05, -2, 2, -2, 13)
    hub = hub - bore
    hub = hub - cyl_z(0, 0, 12.0, w + 1, 1.6, seg=16)
    return hub


def make_cap():
    t = CAP_T
    cap = cyl_z(0, 0, 0, t, HUB_FLANGE_R, seg=96)
    cap = cap + cyl_z(0, 0, t, t + SPIGOT_H - 0.2, SPIGOT_R - 0.12, seg=64)
    cap = cap - (cyl_z(0, 0, -1, 0.6, 16.5, seg=96) - cyl_z(0, 0, -2, 1, 8.0, seg=64))
    for i in range(8):
        a = math.radians(45 * i)
        cap = cap + cyl_z(12.2 * math.cos(a), 12.2 * math.sin(a), 0, 0.7, 1.3, seg=16)
    cap = cap - cyl_z(0, 0, -1, 0.5, 3.0, seg=32)
    return cap


def make_tyre():
    L, ch = TYRE_LEN, 5.0
    rb, ro = TYRE_BASE_R, TYRE_OD_R
    body = M.cylinder(ch, rb - ch, rb, 128) + \
        M.cylinder(L - 2 * ch, rb, rb, 128).translate([0, 0, ch]) + \
        M.cylinder(ch, rb, rb - ch, 128).translate([0, 0, L - ch])
    lugs = []
    n = 18
    half = L / 2 - 0.6
    for i in range(n):
        for zc, ang in ((L / 4 + 0.2, 24.0), (3 * L / 4 - 0.2, -24.0)):   # chevron "V"
            lug = box(-0.6, ro - rb, -1.8, 1.8, -half / 2 - 1.2, half / 2 + 1.2)
            lug = lug.rotate([ang, 0, 0]).translate([rb, 0, zc])
            lugs.append(lug.rotate([0, 0, 360.0 / n * i]))
    envelope = cyl_z(0, 0, 0, L, ro + 0.01, seg=128) ^ (
        cyl_z(0, 0, 0, 2.0, ro - 2.0, ro + 0.01, seg=128) +
        cyl_z(0, 0, 2.0, L - 2.0, ro + 0.01, seg=128) +
        cyl_z(0, 0, L - 2.0, L, ro + 0.01, ro - 2.0, seg=128))
    tyre = body + (union(lugs) ^ envelope)
    bore = cyl_z(0, 0, -1, L + 1, HUB_R - TYRE_FIT, seg=96)
    for i in range(KEYS):
        bore = bore + box(HUB_R - 1, HUB_R + KEY_H + 0.25, -KEY_W / 2 - 0.2, KEY_W / 2 + 0.2,
                          -1, L + 1).rotate([0, 0, 360.0 / KEYS * i])
    return tyre - bore


# --------------------------------------------------------------------------
# Roof and interior details
# --------------------------------------------------------------------------
RACK_LEN = 2 * (DUCT_POD_Y + 4.0)


def make_lightbar():
    """Front roof light bar (3 LEDs) with a spotlight on each end."""
    L, D, H = RACK_LEN, 6.0, 4.6
    rack = box(-D / 2, D / 2, -L / 2, L / 2, 0, H)
    spot_r, spot_z = 3.2, H + 3.0
    for y in (-DUCT_POD_Y, DUCT_POD_Y):
        rack = rack + cyl_x(-3.0, D / 2 + 1.0, y, spot_z, spot_r, 40) + \
            box(-2.0, 2.0, y - 2.0, y + 2.0, H - 0.1, spot_z)
    cut = [box(-1.3, 1.3, -L / 2 - 1, L / 2 + 1, -1, 2.4)]
    for y in (-16.0, 0.0, 16.0):
        cut.append(box(D / 2 - LED_POCKET[2], D / 2 + 1, y - 1.2, y + 1.2, 2.6, 4.2))
        cut.append(cyl_x(0, D / 2, y, 3.4, 0.7, 12))
        cut.append(cyl_z(0, y, 0, 3.8, 0.7, seg=12))
    for y in (-DUCT_POD_Y, DUCT_POD_Y):
        cut.append(cyl_x(D / 2 + 0.2, D / 2 + 2, y, spot_z, 2.4, 32))
        cut.append(box(D / 2 + 0.2 - LED_POCKET[2], D / 2 + 2, y - 1.2, y + 1.2, spot_z - 0.8, spot_z + 0.8))
        cut.append(cyl_x(-0.5, D / 2 + 0.5, y, spot_z, 0.8, 12))
        cut.append(cyl_z(-0.5, y, 0, spot_z + 0.5, 0.8, seg=12))
    return rack - union(cut)


def make_aircon():
    """Roof A/C unit (factory option)."""
    a = box(-15, 15, -20, 20, 0, 6.0) + box(-13, 13, -18, 18, 6.0, 7.0)
    for i in range(7):
        y = -15 + i * 5
        a = a - box(-10, 10, y - 0.5, y + 0.5, 6.4, 8)
    return a


def make_beacon():
    b = cyl_z(0, 0, 0, 2.0, 3.6, seg=40) + cyl_z(0, 0, 2.0, 5.5, 2.8, seg=40)
    b = b - box(-1.2, 1.2, -0.8, 0.8, 5.5 - LED_POCKET[2], 6)
    return b - cyl_z(0, 0, -1, 6, 0.8, seg=12)


def make_fridge():
    L, W, H = FRIDGE
    f = box(-L / 2, L / 2, -W / 2, W / 2, 0, H)
    f = f - (box(-L / 2 - 1, L / 2 + 1, -W / 2 - 1, W / 2 + 1, H - 5.4, H - 4.8) -
             box(-L / 2 + 0.5, L / 2 - 0.5, -W / 2 + 0.5, W / 2 - 0.5, 0, H))
    f = f + box(-L / 2 - 1.0, -L / 2, -5.0, 5.0, H - 4.0, H - 2.5)
    lx = FRIDGE_LED[0] - FRIDGE_POS[0]
    f = f - box(lx - 1.2, lx + 1.2, -0.8, 0.8, H - LED_POCKET[2], H + 1)
    f = f - cyl_z(lx, 0, -1, H + 1, 0.8, seg=12)
    for i in range(4):
        x = 4 + i * 2.5
        f = f - box(x, x + 1.2, W / 2 - 0.5, W / 2 + 1, 3, 9)
    return f


def make_seat():
    return box(0, 10, -5, 5, 0, 5) + box(-2.5, 0.5, -5, 5, 0, 14) + box(-2.0, 0, -3.5, 3.5, 14, 16.5)


def make_tray():
    """Electronics tray: sits on the ledges above the battery (z 48.6)."""
    w = 2 * (HULL_HALF - HULL_WALL) - 0.6
    t = box(-35.5, 35.5, -w / 2, w / 2, 0, 1.6)
    for x in range(-30, 31, 10):           # zip-tie / wire slots
        t = t - box(x - 1.5, x + 1.5, -w / 2 + 3, -w / 2 + 5, -1, 3)
        t = t - box(x - 1.5, x + 1.5, w / 2 - 5, w / 2 - 3, -1, 3)
    return t - box(-8, 8, -6, 6, -1, 3)


# --------------------------------------------------------------------------
# Float check
# --------------------------------------------------------------------------
PLA, TPU = 1.24e-3, 1.21e-3            # g/mm3


def printed_mass(man, density, wall=1.6, infill=0.10):
    v, a = man.volume(), man.surface_area()
    shell = min(v, a * wall / 2)
    return density * (shell + infill * max(v - shell, 0))


HARDWARE_G = {
    "4 x N20 gear motor": 40, "2S 1000 mAh LiPo": 55, "winch servo": 20,
    "ESP32-S3 SuperMini": 3, "2 x TB6612 driver": 6, "5 V buck": 3,
    "USB-C 2S charger": 5, "IP67 button": 8, "4 x M3x40 screws": 8,
    "wire, LEDs, glue": 15,
}


def float_check(mass_g, buoy_solid):
    def disp(zw):
        return (buoy_solid ^ box(-200, 200, -200, 200, -1, zw)).volume() * 1e-3
    lo, hi = 0.0, RIM_Z + 40
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if disp(mid) < mass_g else (lo, mid)
    return (lo + hi) / 2


# --------------------------------------------------------------------------
# Export helpers
# --------------------------------------------------------------------------
def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=False)


def lay_flat(man, flip=False):
    if flip:
        man = man.rotate([180, 0, 0])
    bb = man.bounding_box()
    return man.translate([-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]])


BED, BED_MARGIN = 256.0, 5.0


def place(parts_xy):
    out, boxes = [], []
    for name, m, x, y, ext in parts_xy:
        bb = m.bounding_box()
        mm = m.translate([x - bb[0], y - bb[1], 0])
        b = mm.bounding_box()
        assert b[0] >= BED_MARGIN - 1e-6 and b[1] >= BED_MARGIN - 1e-6 and \
            b[3] <= BED - BED_MARGIN + 1e-6 and b[4] <= BED - BED_MARGIN + 1e-6, \
            "%s off the bed: %s" % (name, [round(v, 1) for v in b])
        for n2, b2 in boxes:
            if b[0] < b2[3] + 2 and b2[0] < b[3] + 2 and b[1] < b2[4] + 2 and b2[1] < b[4] + 2:
                raise AssertionError("%s overlaps %s" % (name, n2))
        boxes.append((name, b))
        out.append((name, mm, ext))
    return out


def build():
    parts = dict(hull=make_hull(), cab=make_cab(), lid=make_lid(), gasket=make_gasket(),
                 hub=make_hub(), tyre=make_tyre(), cap=make_cap(), rack=make_lightbar(),
                 aircon=make_aircon(), beacon=make_beacon(), fridge=make_fridge(),
                 seat=make_seat(), tray=make_tray())
    p = parts
    asm = {
        "hull": p["hull"], "cab": p["cab"], "lid": p["lid"],
        "gasket": p["gasket"].translate([0, 0, RIM_Z]),
        "lightbar": p["rack"].translate([FRONT_DUCT_X, 0, ROOF_Z]),
        "aircon": p["aircon"].translate([46.0, 0, ROOF_Z]),
        "beacon": p["beacon"].translate([REAR_DUCT_X, DUCT_POD_Y, ROOF_Z]),
        "fridge": p["fridge"].translate([FRIDGE_POS[0], FRIDGE_POS[1], LID_Z1]),
        "seat_l": p["seat"].translate([44.0, 12.0, LID_Z1]),
        "seat_r": p["seat"].translate([44.0, -12.0, LID_Z1]),
        "tray": p["tray"].translate([0, 0, 48.6]),
    }
    cap_out = p["cap"].rotate([180, 0, 0]).translate([0, 0, CAP_T])
    for i, x in enumerate((-AXLE_X, AXLE_X)):
        for side, sgn, rot in (("l", 1, -90), ("r", -1, 90)):
            def put(m, z_off=0.0):
                return m.translate([0, 0, z_off]).rotate([rot, 0, 0]).translate(
                    [x, sgn * (WHEEL_Y0 + 0.5), AXLE_Z])
            asm["hub_%s%d" % (side, i)] = put(p["hub"])
            asm["tyre_%s%d" % (side, i)] = put(p["tyre"], HUB_FLANGE_T)
            asm["cap_%s%d" % (side, i)] = put(cap_out, TYRE_W)
    return parts, asm


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    dirs = {k: os.path.join(here, k) for k in ("stl", "3mf", "renders")}
    for d in dirs.values():
        os.makedirs(d, exist_ok=True)
        for f in os.listdir(d):
            os.remove(os.path.join(d, f))

    p, asm = build()
    assembled = union(list(asm.values()))
    bb = assembled.bounding_box()
    print("Assembled size (mm): L %.1f  W %.1f  H %.1f" % (bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]))

    # --- float check ----------------------------------------------------
    pm = {
        "hull": printed_mass(p["hull"], PLA, 2.0), "cab": printed_mass(p["cab"], PLA),
        "lid": printed_mass(p["lid"], PLA), "gasket": printed_mass(p["gasket"], TPU),
        "4 hubs": 4 * printed_mass(p["hub"], PLA), "4 caps": 4 * printed_mass(p["cap"], PLA),
        "4 tyres": 4 * printed_mass(p["tyre"], TPU, 1.2),
        "roof + interior": sum(printed_mass(p[k], PLA) for k in ("rack", "aircon", "beacon", "fridge", "tray"))
        + 2 * printed_mass(p["seat"], PLA),
    }
    total = sum(pm.values()) + sum(HARDWARE_G.values())
    buoy = union([HULL_OUTER ^ box(-200, 200, -200, 200, -10, RIM_Z)] +
                 [asm[k] for k in asm if k.startswith(("hub_", "tyre_", "cap_"))]) - box(*WINCH_POCKET)
    wl = float_check(total, buoy)
    print("Mass: printed %.0f g + hardware %d g = %.0f g" % (sum(pm.values()), sum(HARDWARE_G.values()), total))
    for k, v in pm.items():
        print("   %-16s %5.0f g" % (k, v))
    print("Waterline z = %.1f mm -> freeboard to the gasket %.1f mm (%.0f mm real)" %
          (wl, RIM_Z - wl, (RIM_Z - wl) * SCALE))

    # --- printable parts ------------------------------------------------
    parts = {
        "01_hull_tub_print_floor_down": lay_flat(p["hull"]),
        "02_cab_print_roof_down": lay_flat(p["cab"], flip=True),
        "03_deck_lid": lay_flat(p["lid"]),
        "04_gasket_TPU": lay_flat(p["gasket"]),
        "05_wheel_hub_x4": lay_flat(p["hub"]),
        "06_tyre_x4_TPU": lay_flat(p["tyre"]),
        "07_wheel_cap_x4": lay_flat(p["cap"]),
        "08_roof_light_bar": lay_flat(p["rack"]),
        "09_roof_aircon": lay_flat(p["aircon"]),
        "10_beacon": lay_flat(p["beacon"]),
        "11_fridge": lay_flat(p["fridge"]),
        "12_seat_x2": lay_flat(p["seat"]),
        "13_electronics_tray": lay_flat(p["tray"]),
    }
    for name, m in parts.items():
        tm = to_trimesh(m)
        b = m.bounding_box()
        tm.export(os.path.join(dirs["stl"], name + ".stl"))
        print("%-32s %6.1f x %6.1f x %6.1f  ok=%s" % (name, b[3] - b[0], b[4] - b[1], b[5] - b[2],
                                                    tm.is_watertight and m.status() == mf.Error.NoError))
    to_trimesh(assembled).export(os.path.join(dirs["stl"], "00_assembled_preview_DO_NOT_PRINT.stl"))

    P = parts
    plates = [
        ("Body - PLA", place([
            ("Hull tub", P["01_hull_tub_print_floor_down"], 10, 8, 1),
            ("Cab (roof down)", P["02_cab_print_roof_down"], 10, 76, 1)])),
        ("Details - PLA", place(
            [("Deck lid", P["03_deck_lid"], 8, 6, 1)] +
            [("Wheel hub %d" % (i + 1), P["05_wheel_hub_x4"], 8 + i * 42, 72, 1) for i in range(4)] +
            [("Wheel cap %d" % (i + 1), P["07_wheel_cap_x4"], 8 + i * 42, 116, 1) for i in range(4)] +
            [("Electronics tray", P["13_electronics_tray"], 8, 162, 1),
             ("Roof light bar", P["08_roof_light_bar"].rotate([0, 0, 90]), 88, 162, 1),
             ("Roof aircon", P["09_roof_aircon"].rotate([0, 0, 90]), 182, 72, 1),
             ("Fridge", P["11_fridge"], 182, 110, 1),
             ("Beacon", P["10_beacon"], 222, 110, 1),
             ("Seat 1", P["12_seat_x2"], 182, 134, 1),
             ("Seat 2", P["12_seat_x2"], 200, 134, 1)])),
        ("Tyres + gasket - TPU 95A", place(
            [("Tyre %d" % (i + 1), P["06_tyre_x4_TPU"], 30 + (i % 2) * 100, 6 + (i // 2) * 86, 2)
             for i in range(4)] +
            [("Gasket", P["04_gasket_TPU"], 8, 186, 2)])),
    ]
    proj = "SHERP N1200 %s - A1 project (3 plates, PLA + TPU).3mf" % VERSION
    bambu_3mf.write_project(os.path.join(dirs["3mf"], proj), plates, "SHERP N1200 %s" % VERSION)
    print("3MF project:", proj, "(Bambu profiles included: %s)" % bambu_3mf.profiles_available())
    view = assembled.translate([128 - (bb[0] + bb[3]) / 2, 128 - (bb[1] + bb[4]) / 2, -bb[2]])
    bambu_3mf.write_project(os.path.join(dirs["3mf"], "SHERP N1200 %s - ASSEMBLED view only.3mf" % VERSION),
                            [("Assembled - view only", [("SHERP N1200 assembled", view, 1)])],
                            "SHERP N1200 %s assembled" % VERSION)
    render(asm, dirs["renders"])


def render(asm, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    colours = {"hull": "#3a3d40", "cab": "#e9e9e6", "lid": "#777777", "gasket": "#111111",
               "lightbar": "#222222", "aircon": "#2b2b2b", "beacon": "#e08a00",
               "fridge": "#cccccc", "seat_l": "#444444", "seat_r": "#444444", "tray": "#2d6a4f"}
    for k in asm:
        if k.startswith(("hub_", "cap_")):
            colours[k] = "#55595e"
        if k.startswith("tyre_"):
            colours[k] = "#1a1a1a"
    light = np.array([0.4, 0.3, 0.85])
    light /= np.linalg.norm(light)

    def meshes_of(items):
        return [(to_trimesh(m), colours.get(k, "#1c1c1c")) for k, m in items if not m.is_empty()]

    allk = list(asm.items())
    clip = box(-120, 120, -80, 0, -10, 200)
    body_cut = [(k, m ^ clip) for k, m in allk if not k.startswith(("hub_", "tyre_", "cap_"))]
    lifted = []
    for k, m in allk:
        if k in ("cab", "lid", "lightbar", "aircon", "beacon", "fridge", "seat_l", "seat_r"):
            m = m.translate([0, 0, 45])
        elif k == "gasket":
            m = m.translate([0, 0, 20])
        lifted.append((k, m))
    jobs = [("iso_front", 20, 35, allk, 150), ("iso_rear", 20, 215, allk, 150),
            ("side", 0, 90, allk, 150), ("front", 0, 0, allk, 150),
            ("cutaway", 8, 70, body_cut, 150), ("exploded_cab_off", 22, 35, lifted, 190)]
    for name, elev, azim, items, zmax in jobs:
        fig = plt.figure(figsize=(10, 7.5), dpi=110)
        ax = fig.add_subplot(111, projection="3d")
        for tm, col in meshes_of(items):
            tris = tm.vertices[tm.faces]
            shade = 0.45 + 0.55 * np.clip(tm.face_normals @ light, 0, 1)
            base = np.array(matplotlib.colors.to_rgb(col))
            ax.add_collection3d(Poly3DCollection(tris, facecolors=np.clip(base * shade[:, None], 0, 1),
                                                 edgecolors="none", linewidths=0))
        ax.set_xlim(-80, 80); ax.set_ylim(-80, 80); ax.set_zlim(zmax - 160, zmax)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title("SHERP N 1200 - 1:%d AirLoom model %s (%s)" % (SCALE, VERSION, name))
        fig.tight_layout()
        fig.savefig(os.path.join(out_png, name + ".png"))
        plt.close(fig)


if __name__ == "__main__":
    main()
