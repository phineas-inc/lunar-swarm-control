# Lunar Swarm Control

Prototype operator dashboard for a swarm of survey drones on the Moon, designed for crew working in orbit.

- Real terrain: NASA LRO Wide Angle Camera mosaic (NASA/GSFC/Arizona State University, via NASA Moon Trek), ~83 m/px, Mare Tranquillitatis pit region (8.335°N, 33.222°E). The lander, beacons and survey sites are simulated.
- Autonomy under link delay: drones plan on board; unusual legs are proposed with a veto window longer than the round trip; low fuel and lost position fixes are handled without a link.
- Human factors: attention queue, per-drone intent / reasoning / confidence / track record, tap-then-confirm commands, large targets (orbit mode), low-glare mode.
- Position error model follows the ranging-geometry idea in DLR's swarm navigation paper; mission concept follows the EPFL lunar reconnaissance drone paper and the DLR/ESA supervised-autonomy work.

All numbers are simulated, not flight data.

```bash
npm install
npm run dev
```
