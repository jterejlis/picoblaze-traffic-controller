# PicoBlaze Traffic Light Controller with Tram Priority & Fail-Safe Handling

A deterministic traffic light controller for a multi-approach intersection featuring a dedicated tram corridor, vehicular turning lanes, and pedestrian crossings. Designed and implemented in Xilinx PicoBlaze (KCPSM) assembly as part of the Computer Systems Architecture (ASK) course at University of Lower Silesia DSW.

---

## 1. Project Overview

The objective of this project is to implement a robust, collision-free traffic signal controller operating directly via memory-mapped I/O ports on an FPGA architecture.

Key features:
- Cyclic execution of three primary traffic phases.
- Real-time active tram priority via non-blocking sensor polling.
- Safe termination of conflicting phases via amber and all-red clearance intervals without stack overflow.
- Hardware fail-safe mode (lights_failure) switching all active signals to flashing amber upon fault detection.

---

## 2. Intersection Topology & Signal Mapping

The intersection manages 4 approaches, a central dual-direction tramway (North-South axis), and 20 independently addressed signal heads.

```
                                 [APPROACH 1 - NORTH]
                                |   |   ||   |   |   |   || ^ |
                                | T | T ||   |   |   |   || E |
                                | R | R || v | v | v | v || X |
                                | A | A || 1B| 1C| 1D| 1E|| I |
                                | M | M ||[R]|[S]|[S]|[L]|| T |
                                | 1A| 3A||   |   |   |   ||   |
                                | v | ^ ||   |   |   |   ||   |
-----------------4A [Ped]-------+---+---++---+---+---+---++---+-------2A [Ped]-----------------
<-- EXIT (WB)                   | # | # |                     | 2B [R]   <-- RIGHT LANE
                                | # | # |                     | 2C [S]   <-- (Approach 2)
--------------------------------+ # | # +    INTERSECTION     +--------------------------------
--> LEFT LANE:  4B [L+S]        | # | # |        CORE         | 2D [L+S] <-- LEFT LANE
                                | # | # |                     |              (Approach 2)
--------------------------------+ # | # +                     +--------------------------------
--> RIGHT LANE: 4C [S]          | # | # |                     |
                4D [R]          | # | # |                     | EXIT (EB) <--
-----------------4E [Ped]-------+---+---++---+---+---+---++---+-------2E [Ped]-----------------
                                | v | ^ || v |   |   |   ||   |
                                | 1A| 3A|| E | 3B| 3C| 3D|| 3E|
                                | T | T || X |[L]|[S]|[S]||[R]|
                                | R | R || I | ^ | ^ | ^ || ^ |
                                | A | A || T |   |   |   ||   |
                                | M | M ||   |   |   |   ||   |
                                [APPROACH 3 - SOUTH]

Legend:
- 1A, 3A         : Dedicated dual tram tracks (NS & SN) along the western edge
- 1B, 3E         : Right-turn lanes [R]
- 1C/1D, 3C/3D   : Dual straight/through lanes [S]
- 1E, 3E         : Left-turn lanes [L]
- 4B, 2D         : Left lane (Shared Left + Straight: [L+S])
- 4C/4D, 2C/2B   : Right lane (Straight [S] + dedicated Right-turn signal [R])
- EXIT (WB/EB)   : Clearance / outbound road lanes
- 4A, 4E         : Western crosswalks (spanning Approach 4 lanes)
- 2A, 2E         : Eastern crosswalks (spanning Approach 2 lanes)
- #              : Tram conflict points with westbound traffic

```

### Signal Encoding (Active-High)
Control signals are dispatched to ports as 3-bit bitmasks:
- RED + YELLOW = 110b (0x06) - preparatory phase
- RED          = 100b (0x04) - stop
- YELLOW       = 010b (0x02) - clearance / transition warning
- GREEN        = 001b (0x01) - proceed
- OFF          = 000b (0x00) - dark state (flashing fault mode)

*(Note: For hardware design consistency across all 20 I/O ports, tram signal heads 1A and 3A use the common 3-bit state encoding).*

---

## 3. Phase Sequencing & Conflict Prevention

Traffic flow transitions deterministically through a three-phase finite state machine:

| Phase | Active Green Signals | Description |
|---|---|---|
| Phase 1 (green_main_street) | 1A, 1C, 1D, 3A, 3C, 3D, 2A, 2E, 4A, 4E | Main arterial corridor, tramway pass-through, parallel pedestrian crossings. |
| Phase 2 (main_street_turns) | 1B, 1E, 3B, 3E, 2B, 4D | Protected turns from main avenue + non-conflicting right turns from side street. |
| Phase 3 (side_street_green) | 2C, 2D, 4B, 4C | Through and left movements from secondary cross-street. |

---

## 4. System Architecture & Tram Preemption

Time intervals are generated via a four-stage nested delay loop (timeloop -> tl2 -> tl3 -> tl4), calibrated against the system clock frequency.

```
            +---------------------------------+
            |          System Reset           |
            | (Init s0..sE, s7=0, s8=0, all_red)
            +----------------+----------------+
                             |
                             v
     +---------------->  mainLoop  <----------------+
     |                       |                      |
     |                       v                      |
     |            CALL green_main_street            |
     |            (s8=0: Tram ignored)              |
     |                       |                      |
     |                       v                      |
     |            CALL main_street_turns            |
     |            (s8=1: Tram monitored)            |
     |                       |                      |
     |                [ Is s7 == 1? ] --- YES ---> tram_override
     |                       | (NO)                (s7=0, jump)
     |                       v                          ^
     |            CALL side_street_green                |
     |            (s8=1: Tram monitored)                |
     |                       |                          |
     |                [ Is s7 == 1? ] --- YES ----------+
     |                       | (NO)
     +-----------------------+
```

### Tram Detection & Stack Integrity (Safe Exit via RET)
1. Low-Jitter Polling (tl3): The tram sensor is polled once every cycle of tl3 (~6.5 ms). This avoids timing jitter in the innermost delay loop (tl1).
2. Phase Filtering (s8): During green_main_street, register s8 is 0, ignoring tram triggers while the main transit corridor already has a green signal.
3. Preemption Flag (s7): If a tram is detected during conflicting phases (s8 == 1), s7 is set to 1, executing an immediate RET. The current phase safely completes its amber and all-red cycles before returning to mainLoop, preventing hardware stack overflow.

---

## 5. Fail-Safe Routine (lights_failure)

At the beginning of each phase, the failure input port is sampled (active-low 0). If a fault is present, standard phase sequencing stops, and execution enters an infinite loop toggling all operational warning lights between yellow (0x02) and off (0x00) at 1-second intervals.

---

## 6. AI Ethics Statement

- Source Code: The entire architecture, state transitions, assembly routines, I/O memory mapping, and timing loops were designed and written by me during my studies at University of Lower Silesia DSW for the Computer Systems Architecture laboratory.
- AI Assistance: Artificial intelligence was used as an editorial tool for translating original code comments from Polish to English, organizing comment layout, proofreading typos, and drafting this documentation file (README.md).
