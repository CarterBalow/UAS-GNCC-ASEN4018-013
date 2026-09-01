# RFP Summary — UAS/Rover Network (ASEN 4018, Sections 013/014)

Source: `ASEN 4018013_014 AY2627 RFP v2.pdf`. This doc distills the RFP with emphasis on what's relevant to the UAS dynamics-and-control (GNC) subteam.

## Project title
An Autonomous and Cooperative UAS Multirotor and Rover Network for Exploration, Search, and Payload Delivery/Retrieval

## PI
Melvin Rafi

## Concept
A multirotor UAS searches an area of interest to find a precise location (marked by an ArUco marker) that a ground rover cannot easily search itself. The UAS relays the marker's coordinates to the rover (or a base station), and the rover autonomously navigates to that location to deliver or retrieve a payload — reaching terrain the UAS can't land on. The two vehicles are meant to operate autonomously/semi-autonomously and cooperatively as one network.

## Mission objective
Autonomously and cooperatively explore an area of interest to locate and deliver/retrieve a payload at a desired location of interest.

## Section split
- Section 013: rover **payload delivery**
- Section 014: rover **payload retrieval**

(Which section Liam's team is in isn't recorded yet.)

## UAS requirements (most relevant to Liam's subteam)
1. Must house all hardware, sensors, comms, and electronics needed for safe mission fulfillment.
2. Envelope: ≤50 cm L × 50 cm W × 30 cm H, ≥15 min endurance, operable 15–25°C, ≤25 lb (11.3 kg).
3. Must take off, autonomously navigate to a predetermined area of interest, search it to detect an ArUco marker to within 30 cm radius accuracy, relay marker coordinates to the rover/base station, and return to start.
4. Test/demo flight envelope: altitude ≤2 m AGL, speed ≤0.5 m/s, must stay within visual line of sight — parameters must be compliant with and approved by CU Flight Ops.
5. Autonomous flight phases can be semi-autonomous where needed, but every autonomous phase must have a manual-control override.
6. Must comply with all FAA regulations and CU Flight Ops policies.

## Rover requirements (for context/coordination)
1. Must house all hardware/sensors/comms/electronics needed.
2. Envelope: ≤30 cm L × 30 cm W × 30 cm H (excluding delivery/retrieval mechanism), ≥60 min endurance, operable 15–25°C.
3. Must receive marker coordinates from the UAS/base station, autonomously navigate from start to the marker, deliver/retrieve payload within a 30 cm radius of the marker, and return to start. Delivery/retrieval itself may be manually controlled.
4. Must traverse terrain features up to 3 cm height without impeded movement.
5. Autonomous motion phases can be semi-autonomous where needed, but must have manual override.

## Definitions (matter for testing/CONOPS)
- Area of interest: ~5 m × 5 m test space (indoor or outdoor, with safety approvals)
- Location of interest: a single coordinate marked by an ArUco marker
- Payload: 8 cm × 8 cm × 8 cm, ≤1 kg
- Terrain feature: an object ≥3 cm height

## Open items for the GNC/dynamics-and-control subteam
- Nail down navigation/localization approach to hit the 30 cm marker-detection accuracy requirement (GPS-denied indoor testing is likely, given the 5m×5m area and 2m AGL ceiling — suggests vision-based or motion-capture-assisted localization).
- Define the autonomous search pattern/coverage algorithm for the area of interest.
- Design the coordinate hand-off/relay to the rover (direct comms vs. base station relay) and its latency/accuracy budget.
- Determine the manual-override architecture for each autonomous flight phase (per requirement 5).
- Confirm CU Flight Ops approval process for the 2 m AGL / 0.5 m/s / VLOS test profile.
- Requirements are explicitly stated as a starting point — team is expected to question, refine, and trade-study them, coordinating any changes with PI Melvin Rafi.
