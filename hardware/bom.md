# Bill of Materials - proposal vs as-built

This file has two lists. The two lists do **not** match. Read both lists.

- The **Proposal BOM** is the funded and approved plan. The source is
  `TrolleyX_Revised_BOM.xlsx` and the revised proposal. It is the academic
  authority for scope and cost.
- The **As-built** list is the hardware on the cart today. The team changed the
  drivetrain electronics during fabrication.

## Divergence summary (important)

| Item | Proposal / BOM | As-built | Why it matters |
| --- | --- | --- | --- |
| Motor driver | 1x SmartDrive Duo MDDS30 | **2x L298N** | Different command interface, current limits, and wiring. |
| Motors | 2x 30 W gear motor | **4x 6 V gear motor** (skid) | 4WD skid-steer. Cap the PWM for the 6 V motors. |
| Controller | "Arduino" (unspecified) | **2x Arduino Uno R3** (one on Prototype Shield v5) | The Uno fixes the pin map and interrupt budget. |
| Follow | Webcam vision (BOM) | **UWB** (RYUW122_Lite x3) | UWB replaces vision. |
| Compute | Pi 5 or tether | Pi 5 (16 GB) | As planned. |

To cost or report the project, cite the proposal BOM. To wire or code the cart,
follow **[wiring.md](wiring.md)**.

## Proposal BOM (funded plan)

Source: `TrolleyX_Revised_BOM.xlsx`, sheet `Revised BOM`. The project is
self-funded, with no grant. It uses vision-follow on an owned 720p webcam. It
removes the Intel NUC and RealSense from the original plan.

| Section | Item | Qty | Status | Cost (AED) |
| --- | --- | ---: | --- | ---: |
| Fabrication | Aluminium chassis fabrication | 1 | Buy | 1500 |
| Fabrication | Plastic basket | 1 | Buy | 200 |
| Fabrication | Sensor mounts & brackets (3D print) | Multi | Make | 250 |
| Fabrication | Wiring & connectors | Multi | Buy | 300 |
| Power | LiFePO4 12 V battery pack | 1 | Buy | 480 |
| Power | DC-DC converter | 2 | Buy | 150 |
| Power | Industrial E-stop | 1 | Buy | 60 |
| Power | Power distribution & fuse set | 1 | Buy | 150 |
| Motor | 30 W gear motor w/ encoder | 2 | Buy | 500 |
| Motor | SmartDrive Duo MDDS30 | 1 | Buy | 700 |
| Motor | Caster wheel | 2 | Buy | 150 |
| Motor | Rubber drive wheel | 2 | Buy | 250 |
| Nav/compute | RPLIDAR A1 | 1 | Buy | 450 |
| Nav/compute | Arduino (owned) | 1 | Have | 0 |
| Nav/compute | Raspberry Pi 5 16 GB | 1 | Buy | 700 |
| Nav/compute | 720p webcam (owned) | 1 | Have | 0 |

Build subtotal with the Pi 5: **AED 5840**. Laptop-tethered: **AED 5140**.
Optional UWB kit (now adopted): ~AED 400-500. Removed from the original (NUC,
RealSense, owned Arduino): AED 3300. Original BOM total: AED 9140.

## As-built additions / substitutions (actual spend to reconcile)

The physical build replaced the MDDS30 and 2-motor plan with:

- 2x L298N dual H-bridge drivers.
- 4x 6 V DC gear motors with quadrature encoders (65 mm wheels).
- 2x Arduino Uno R3 (one with Prototype Shield v5 screw-terminal shield).
- REYAX RYUW122_Lite UWB x3 (ordered).
- LM2596 buck (Pi 5 V), WAGO lever connectors for distribution.
- MPU accelerometer (owned, to integrate).

> TODO: record the actual purchase prices of the L298Ns, the 4 motors, the second
> Uno and shield, the UWB modules, and the wiring consumables (below). Then produce
> a final as-spent total for the cost section of the report.

## Still to order (before further testing)

| Priority | Item | Purpose |
| --- | --- | --- |
| Now | Dupont jumpers, F-M + M-M assorted | ENA/ENB and IN pins to the Arduino; grounds |
| Now | Heat-shrink assortment | Re-sheath the weak encoder splices |
| Now | Silicone hookup wire 22 AWG (red/black) | A proper common-ground bus |
| Now | Ferrules / bootlace crimps + spade terminals | Clamp bare wire into shield/WAGO screw terminals |
| Soon | Small 5 V buck (MP1584/mini LM2596) if LM2596 headroom is tight | Dedicated 5 V for the LiDAR |
| Check | Industrial E-stop + fuse/PDU | The BOM lists both as "Buy". Confirm they are in hand. |
