# SHERP N 1200 – 1:22 AirLoom demo model (v0.1, quick draft)

A 3D-printable scale SHERP N 1200 set up as an off-road camper (no awning) to
demonstrate the AirLoom wireless switching system. It has wired spots for
headlights, tail lights, a light bar, spotlights, an amber beacon and a fridge,
plus an RC winch. The ESP32, battery and wiring are hidden in the hull, behind
a removable belly plate.

**This is a first-fit draft.** Print it fast and light, check the fit, and
we'll refine the shape and details in v0.2.

![front](renders/iso_front.png)

| Renders | |
|---|---|
| ![side](renders/side.png) | ![rear](renders/iso_rear.png) |
| ![cutaway](renders/cutaway_from_left.png) | ![top](renders/top.png) |

## Size and sources

| | Real vehicle | Model (1:22) |
|---|---|---|
| Length | 4002 mm | 182 mm body (185.5 mm incl. spotlights) |
| Width | 2520 mm | 114.5 mm |
| Height | 2852 mm | 131.9 mm incl. light bar / beacon |
| Wheelbase | 2118 mm | 96.3 mm |
| Tyres | 1800 × 600 mm | 82 × 27 mm |
| Ground clearance | – | 27 mm |

Sources:
- SHERP N 1200 manufacturer spec page (length, width, height, wheelbase):
  https://sherpglobal.com/en/sherp-n-1200-2/
- Manufacturer PDF spec sheet: https://sherpglobal.com/wp-content/uploads/2020/11/SHERP-N-1200.pdf
  (older figures: 3984 × 2520 × 2846 mm)
- *SHERP-N_catalogue.pdf* in your Google Drive: tyre size 1800×600×25, the
  hardtop aluminium cabin, front door with 3 support points, back door, roof
  hatch, 2 or 4 side windows, front and rear tow bumpers, and the factory
  winch, LED headlamps, rear lights and light bar options.
- I couldn't reach www.sherpaustralia.com or sherp.global from this session
  because the network blocked them. The shape is my reading of the
  manufacturer data. It isn't traced from photos, so expect to correct some
  proportions in v0.2.

## Parts to print (`stl/`)

All parts print **without supports** in the orientation of the STL file.

| File | Qty | Notes |
|---|---|---|
| `01_hull_print_deck_down.stl` | 1 | Lower body, floor (deck) on the bed. Holds the lights, winch fairlead, USB-C port and the electronics bay |
| `02_cabin_print_roof_down.stl` | 1 | Cabin shell, roof on the bed. Window openings, plus 4 built-in wiring ducts |
| `03_belly_plate.stl` | 1 | Removable underside with cradles for the battery, ESP32, winch servo and switch |
| `04_wheel_x4.stl` | 4 | Inner face on the bed |
| `05_lightbar_print_groove_up.stl` | 1 | |
| `06_beacon.stl` | 1 | Amber beacon base |
| `07_fridge.stl` | 1 | ~60 L-class camp fridge at scale |
| `08_seat_x2.stl` | 2 | Optional cab seats |
| `00_assembled_preview_DO_NOT_PRINT.stl` | – | For viewing only |

Everything fits a Bambu A1 / 256 mm bed. The hull and cabin are 185 × 60 mm
and 179 × 57 mm, so both fit on one plate with the small parts.

### Draft print settings (PLA)
- Layer 0.2 mm (0.28 is fine for the wheels)
- **Wall loops 4.** The model walls are 1.6 mm, so they come out solid and the
  infill only affects the wheels, fridge and seats.
- **Sparse infill 5 %** (gyroid or lightning), top/bottom shells 3
- Supports off. If the bumper spotlight pods droop, turn on tree supports
  ("touching build plate only") for the hull.
- The wheels are solid models that the slicer fills sparsely. Hollow wheels
  are on the v0.2 list.

## Electronics: parts list

Small, cheap and enough for 6 light channels (1 blinking amber) plus a
winch that runs in/out/stop. The links came from web searches. They're
examples, not tested suppliers, so any listing of the same part works
(AliExpress, eBay, Core Electronics, etc.).

| # | Part | Qty | Why / notes | Example link |
|---|---|---|---|---|
| 1 | **ESP32-C3 SuperMini** (22.5 × 18 mm) | 1 | Tiny, Bluetooth LE + Wi-Fi, same chip family as the AirLoom Node C3 firmware | Pinout/spec: https://www.espboards.dev/esp32/esp32-c3-super-mini/ (search "ESP32-C3 SuperMini") |
| 2 | **Injora INJS2065-360 winch servo** (23 × 13 × 25.8 mm, 4.8–8.4 V, 360° continuous) | 1 | Made for 1:24 crawlers. One signal wire gives IN / OUT / STOP, so no motor driver board is needed | Servo only: https://racechoice.com/injora-tow-truck-crane-kit-injs2065-360-winch-servo-only-trx4m-injs2065-360/ · in a crane kit: https://nankinhobby.com/products/tow-truck-crane-kit-injs2065-360-winch-servo-for-124-scx24-injs2065-360-194bk |
| 3 | **USB-C 1S Li-ion charge + 5 V boost module** (e.g. LX-LBC3, 16 × 12 mm) | 1 | Charges the battery from USB-C, gives 5 V out, and can charge while running | https://scccltd.com/products/lx-lbc3-type-c-2-in-1-charging-discharging-module · alt: https://thinkrobotics.com/products/type-c-usb-5v-2a-battery-charging-discharging-boost-module |
| 4 | **1S 3.7 V LiPo, 500–1000 mAh**, max 50 × 30 × 8 mm (e.g. 503035 / 603040) with a protection PCB | 1 | Cradle is 52 × 32 mm | search "503035 LiPo JST" |
| 5 | **Pre-wired 0603 micro-LEDs (litz wire)**, white | ~10 | 2 head + 3 bar + 2 spot + 1 fridge (+ spares) | https://eve-modeler.com/products/l0603-20pcs-pre-wired-smd-0603-led-pre-soldered-micro-litz-wired |
| 6 | Pre-wired 0603 LEDs, **red** | 2 | Tail lights | https://www.desertcart.ie/products/158517003-25pcs-pre-soldered-micro-litz-wired-leads-red-smd-led |
| 7 | Pre-wired 0603/0805 LED, **amber/orange** | 1 | Beacon. I couldn't confirm an amber 0603 listing; this seller lists several sizes and colours | https://ebay.com/itm/155149687588 |
| 8 | SS12D00 mini slide switch (8.5 × 3.5 mm) | 1 | Main power switch, operated from underneath | search "SS12D00G3" |
| 9 | Resistors 1/8 W: **47 Ω** ×8 (white), **100 Ω** ×3 (red/amber) | – | One per LED, at the ESP32 end | |
| 10 | M2 × 6 self-tapping screws | 4 | Belly plate | |
| 11 | M3 × 30 pan-head screws + washers | 4 | Wheels (they self-tap into the hull bosses; don't overtighten or they won't spin) | |
| 12 | 0.5 mm braid or thread + a tiny hook | – | Winch rope | |
| 13 | Thin silicone hook-up wire (28–30 AWG), heat-shrink, double-sided foam tape, cable ties ≤3 mm, CA or UV glue | – | | |

## Wiring

```
USB-C ──► charge/boost module ──5V──┬──► ESP32-C3 "5V" pin
            │  ▲                    └──► winch servo + (red)
            │  └─ battery via SS12D00 switch
            GND ─────────────────────── ESP32 GND, servo GND (brown), all LED (−)

ESP32-C3 GPIO ─ resistor ─ LED(+)      (LEDs on the same channel are wired in parallel, each with its own resistor)
  GPIO0  headlights   2 × white  47 Ω each
  GPIO1  tail lights  2 × red   100 Ω each
  GPIO3  light bar    3 × white  47 Ω each
  GPIO4  spotlights   2 × white  47 Ω each
  GPIO5  fridge       1 × white  47 Ω
  GPIO6  amber beacon 1 × amber 100 Ω   (blinks in firmware)
  GPIO7  winch servo signal (orange/white wire)
  GPIO10 spare
  (avoid GPIO2/8/9 – boot strapping pins / on-board LED)
```

Notes:
- Each GPIO sources about 8–13 mA per LED at 3.3 V, which is within the
  ESP32-C3's limits. If a white LED is too dim, drop it to 33 Ω.
- Some charge/boost modules switch off when the load is light (under about
  50 mA). The ESP32 running Bluetooth normally draws more than that. If the
  module does cut out, choose an "always-on" module or leave the fridge LED on.

## Where everything goes / hidden wiring

All wiring ends up in the **hull bay** (z 27–60 mm, about 140 × 54 × 31 mm),
under the cabin floor where you can't see it through the windows.

| Light | Location | Wire route |
|---|---|---|
| Headlights | Bow panel either side of the front door | 1.8 mm hole straight into the bay |
| Spotlights | Two pods on the front tow bumper | Straight into the bay |
| Tail lights | Rear face, lower corners | Straight into the bay |
| Light bar | Roof front edge | Bar's underside groove → two roof holes → **A-pillar ducts** (3 mm bore, inside the windscreen pillars) → floor holes → bay |
| Amber beacon | Roof, rear left corner | Rear-left corner duct → floor → bay |
| *Spare* | Roof, rear right corner | Rear-right corner duct, free for a camp/rear work light |
| Fridge | Rear module, near the back door | LED pocket on the fridge lid → hole down through the fridge → floor hole → bay |
| Winch | Servo on the belly plate, front | Rope runs forward through the fairlead in the bow |
| USB-C | Rear face, centre, below the back door | Charge module receptacle is hot-glued into the collar behind the slot |
| Spare | 5 mm hole in the cab floor between the seats | e.g. for a dash Hub display later |

**Fitting an LED:** if a lead is too short to reach the ESP32, solder an extension wire on inside the bay. Feed the two wire tails in from the outside through the
pocket's hole, pull them through, then set the LED into its pocket with a dot
of CA or UV glue. A dome of clear UV resin over the beacon LED makes a nice
lens.

## Assembly order
1. Print everything. Test-fit the cabin over the rim on the hull deck (it
   should press on; leave it unglued for now).
2. Fit the hull lights (head, spot, tail) and pull their wires into the bay.
3. Fit the light bar LEDs, lay their wires in the bar's groove and run them
   out of the ends. Hold the cabin just above the hull and push the wires
   down the two A-pillar ducts and through the floor holes. Do the same for
   the beacon (rear-left duct). Glue the bar and the beacon to the roof.
4. Glue the fridge (LED wire through the floor) and the seats onto the deck.
   Then press the cabin down onto the hull.
5. On the belly plate: battery in its cradle (cable tie), ESP32 in its
   cradle (foam tape; its USB-C faces the rear so you can reach it),
   winch servo between the rails (cable tie through the slots, spool
   forward), and the switch in its frame with the slider through the slot.
6. Hot-glue the charge module with its USB-C in the rear collar. Wire it up,
   flash `test_sketch/` and check every channel and the winch.
7. Thread the winch rope out through the fairlead. Screw on the belly plate
   (4 × M2) and the wheels (4 × M3 × 30).

## Regenerating / changing the model
Everything is parametric in `generate.py` (Python, `manifold3d`):

```
pip install manifold3d trimesh numpy matplotlib
python3 generate.py
```

The scale, wall thickness, LED pocket size, duct size, battery size and so
on are constants at the top of the file.

## Known limits of v0.1 / ideas for v0.2
- The body is a simple extruded side profile: flat sides, sharp edges, no
  wheel fenders. Next steps are rounding it off and adding the door/hinge
  detail, front-door support struts, mirrors, a roof rack or the A/C unit.
- The wheels are solid. Next version: hollow shells and a real SHERP
  tread pattern.
- The winch bay is generic because I don't have the servo's exact spool
  geometry. I'll refine it once you've got the part in hand.
- SHERP the Ark comes later. The script is set up so a new profile and
  dimensions can be dropped in.
