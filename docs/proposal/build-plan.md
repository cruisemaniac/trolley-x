# Trolley-X - Sim-to-Real Build & Dissertation Plan

**Module:** PDE4445 Robotics Dissertation Project (MDX Dubai)
**Team:** Ashwin Murali Thanalapati (M01037932) - Mohammed Shalaby (M01035318) - Vignesh Lakshmanasamy (M01026685)
**Scope:** Convert the validated CW2 ROS 2 simulation into a working, load-bearing physical cart that demonstrates autonomous following + three-zone LiDAR safety. Self-funded, no university grant.

---

## 1. What we are building (and why this is the right deliverable)

The CW2 simulation was already submitted and demonstrated, so it cannot be the dissertation deliverable - reusing it is self-plagiarism. The dissertation is the **physical realization** of that validated design, which satisfies Learning Outcome 3 ("a development of an existing piece of software... or hardware") and gives a fresh demo for the presentation.

The cart keeps the full load-bearing drivetrain (aluminium chassis, 30 W encoded gear motors, MDDS30 drive, rubber wheels, LiFePO4 power, industrial E-stop). The perception/compute side is cut to what we already own: the **Arduino** for low-level motor control and the **720p webcam** for vision - replacing the Intel NUC and RealSense. Revised build cost: **~AED 5,840** (with onboard Pi) or **~AED 5,140** (laptop tethered), vs AED 9,140 original.

**Following approach:** vision-follow on the existing webcam (detect person -> centre -> drive, distance held via bounding-box size or LiDAR). Lower risk and zero extra hardware. UWB realization listed as future work (optional ~AED 400 kit if time allows).

---

## 2. Team split (parallel tracks)

| Owner | Track | Responsibilities |
|---|---|---|
| **Vignesh** | Mechanical & power | Chassis fabrication, motor/wheel/caster mounting, LiFePO4 + DC-DC, E-stop, fused power distribution |
| **Ashwin** | Sensor fusion & control | RPLIDAR three-zone safety (port from CW2), follow controller, sensor integration, test/evaluation |
| **Shalaby** | ROS 2 & vision | ROS 2 port to Pi, Arduino serial motor interface, webcam vision pipeline/model, system integration, supervisor liaison |

---

## 3. Build timeline (~7 weeks, then write-up to deadlines)

**Week 0 - now.** Order long-lead items first: chassis fabrication, motors, MDDS30, RPLIDAR A1, battery, Pi. Create a `hardware` branch off the CW2 repo. Confirm the module's Gen-AI policy with the supervisor. Start the blog immediately.

**Weeks 1-2 - mechanical + bring-up (parallel).**
- Vignesh: assemble chassis, mount drivetrain, wire power + E-stop + fused distribution.
- Shalaby: ROS 2 skeleton on Pi; Arduino serial motor control bring-up.
- Ashwin: RPLIDAR A1 ROS 2 driver running on the bench.

**Weeks 2-3 - drive + safety.**
- Closed-loop encoder motor control; **manual teleop driving validated on hardware** (reuse CW2 teleop node).
- Ashwin: port three-zone LiDAR safety (warn/slow/stop) to hardware; tune zones for the real sensor.

**Weeks 3-5 - following.**
- Ashwin + Shalaby: webcam person-detection -> bearing + distance -> drive commands; integrate with the safety layer so stop/slow overrides follow.

**Weeks 5-6 - integration + formal testing.**
- Full stack on the cart: follow + safety + teleop fallback.
- Run formal tests mirroring the CW2 sim metrics (follow-distance accuracy, safety-stop distances, repeatability) -> produces **sim-vs-real comparison data**, which is the strongest material for the report.

**Weeks 6-7 - demo + buffer.** Capture a new demonstration video (distinct from CW2). Hold buffer time for fixes.

**Week 7 -> deadlines - write-up.** Report and blog finalisation, presentation rehearsal.

**Fallback (de-risk):** if vision-follow runs short, the guaranteed demo is **teleop + autonomous three-zone LiDAR safety** (both low-risk, sim-validated), with vision-follow shown in whatever state it reaches. This protects the presentation grade regardless.

---

## 4. Mapping to assessment

| Component | Weight | Due | What to do |
|---|---|---|---|
| **Project blog** | 40% | Jul 2026 | Start now. Weekly multimedia posts per track - decisions, problems/fixes, planning & time management. This is the nearest deadline; do not leave it late. |
| **Project report** | 40% | 25 Sep 2026 | ~8-page research article. Frame as sim-to-real development of CW2. Sections: abstract, intro, related work (incl. commercial carts e.g. Foxtech), methodology, hardware design, implementation, testing & evaluation (sim vs real), discussion, conclusion + future work. |
| **Final presentation** | 20% | Oct 2026 | Oral + **live demo of the physical cart** (not the CW2 video). |

---

## 5. Academic-integrity checklist (from the module handbook)

- **Self-plagiarism:** CW2 was a separate submission. Cite it as prior work and *build on it* - never reproduce sections. The hardware realization + new evaluation is the new contribution.
- **Generative AI:** the handbook prohibits Gen-AI in assessments unless the module leader permits it; where allowed, you must declare the extent of use and the prompts. Confirm the policy with Sameer Kishore / your supervisor before using any AI on the submitted blog, report, or slides, and declare it if used.
- **Research ethics / human data:** if any test involves a real person being followed, or any people-data/images, clear it with your supervisor first. Keeping tests to team members and objects avoids triggering ethics approval.

---

## 6. Immediate next actions

1. Place the long-lead orders (chassis fab, motors, MDDS30, RPLIDAR, battery, Pi).
2. Stand up the project blog and post the kickoff entry.
3. Email the supervisor to (a) confirm the Gen-AI policy and (b) confirm a simulation-to-hardware dissertation framed as a development of CW2 is acceptable.
4. Create the `hardware` repo branch and port the ROS 2 skeleton to the Pi.
