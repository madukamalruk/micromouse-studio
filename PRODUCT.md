# mmsim — product context

Inferred from the owner's Slice 9 brief (2026-11-20), not from an interview.

- **What:** real-time software-in-the-loop micromouse simulator for Windows (C++20 host,
  C11 firmware DLLs, mms solvers). Phase 2 target robot: Neomouse V2.
- **Who:** (1) non-technical viewers, students and demo audiences who just want to pick a
  maze, press GO and watch; (2) robotics engineers who tune sensors, PID and speed
  profiles, compare runs and produce reports.
- **Success:** a newcomer runs a mouse in under 10 s without reading anything; an engineer
  gets numbers they trust (tabular, labelled with units) and exports in one click.
- **Reference:** Peter Harrison's micromouseonline simulator video (`3F9_k6CBJQk`):
  hardware-style robot faceplate, live sensor bars, maze-first layout.
- **Anti-references:** Neomouse V2 web dashboard look (olive/lime), stock ImGui styling.
- **Platform:** desktop, SDL2 + Dear ImGui (docking) + ImPlot, 1440x900 reference.
