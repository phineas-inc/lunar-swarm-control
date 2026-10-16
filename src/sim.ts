// Lunar swarm simulation.
// Ideas taken from the reference papers:
//  - Robohub (DLR/ESA): operator picks between supervised autonomy and direct avatar-style control
//  - EPFL reconnaissance drone: limited-fuel hops from a service station, mapping hard-to-reach terrain
//  - DLR swarm navigation: anchors + inter-drone ranging; formation geometry (Fisher information) sets position error
//
// Autonomy under link delay (management by exception):
//  - Drones plan and act on board. Only unusual legs (long, or leaving a thin fuel margin) become proposals.
//  - "veto" policy: a proposal proceeds on its own when its window closes. The window is longer than the
//    round trip (2 x delay + reaction time), so a veto can physically arrive in time.
//  - "ask" policy: the drone holds safely until approved. "full": no proposals at all.
//  - Drones recover their own position fix and head home on low fuel without any command.
//  - A link blackout never stops the mission: commands are held, drones keep working under the policy.

// Real map: LRO WAC mosaic, Mare Tranquillitatis pit region, 480x288 source px at ~83 m/px
// => the 1600x960 world is ~40 x 24 km, so one unit is ~25 m.
export const W = 1600;
export const H = 960;
export const CELL = 20;
export const GW = W / CELL;
export const GH = H / CELL;
export const MPU = 25;
export const PIT = { lat: 8.335, lon: 33.222 };
export const MAP_KM = { w: 40, h: 24 };
export const SPEED = 70; // units/s
export const FUEL_PER_UNIT = 0.02;
export const COMM_RANGE = 450;
export const ANCHOR_RANGE = 900;
export const REVIEW_LEG = 250; // legs longer than this (units) need review

export type Mode = "idle" | "auto" | "waypoint" | "direct" | "rtb" | "charging" | "recover";
export type Autonomy = "ask" | "veto" | "full";
export type Actor = "AUTO" | "CREW" | "SYS";
export type Pt = { x: number; y: number };
export type Crater = Pt & { r: number; psr: boolean };
export type Target = Pt & { name: string; found: boolean };
export type Stats = { legs: number; vetoed: number; approved: number; overrides: number; recoveries: number };
export type Drone = Pt & {
  id: number;
  name: string;
  color: string;
  mode: Mode;
  resume: boolean; // go back to auto survey after charging
  fuel: number;
  flights: number;
  path: Pt[];
  goal: Pt | null;
  input: Pt;
  err: number; // position error, metres
  links: number;
  moved: number;
  awaiting: number | null; // proposal id the drone is holding for
  avoid: (Pt & { until: number }) | null;
  why: string;
  stats: Stats;
  savedMode: Mode | null;
  savedGoal: Pt | null;
};
export type Proposal = { pid: number; drone: number; goal: Pt; dist: number; fuelEst: number; why: string; createdAt: number; deadline: number | null };
export type Cmd = { at: number; id: number; type: "auto" | "goto" | "add" | "rtb" | "stop" | "input" | "veto" | "approve"; pts?: Pt[]; v?: Pt; pid?: number };

export type Sim = {
  t: number;
  drones: Drone[];
  craters: Crater[];
  targets: Target[];
  anchors: (Pt & { name: string })[];
  station: Pt;
  reserve: number; // full-tank equivalents left in the service station
  map: Uint8Array;
  mapDirty: boolean;
  queue: Cmd[];
  log: string[];
  latency: number;
  autonomy: Autonomy;
  proposals: Proposal[];
  nextPid: number;
  blackoutUntil: number;
  wasBlackout: boolean;
};

// Okabe-Ito colour-blind-safe set
const COLORS = ["#56B4E9", "#E69F00", "#CC79A7", "#009E73", "#F0E442", "#D55E00", "#B79CFF", "#D9D9D9"];
export const MAX_DRONES = COLORS.length;
export const START_DRONES = 6;

export function createSim(latency = 1.3, autonomy: Autonomy = "veto"): Sim {
  const station = { x: 200, y: 480 };
  const targets: Target[] = [
    { name: "Tranquillitatis Pit", x: 800, y: 480, found: false },
    { name: "Survey site A", x: 1220, y: 280, found: false },
    { name: "Survey site B", x: 520, y: 760, found: false },
    { name: "Survey site C", x: 1330, y: 790, found: false },
  ];
  const drones: Drone[] = COLORS.slice(0, START_DRONES).map((color, i) => ({
    id: i,
    name: `LUNA-${i + 1}`,
    color,
    x: station.x + (i % 3) * 14 - 14,
    y: station.y + Math.floor(i / 3) * 14 - 7,
    mode: "idle",
    resume: false,
    fuel: 100,
    flights: 0,
    path: [],
    goal: null,
    input: { x: 0, y: 0 },
    err: 1,
    links: 3,
    moved: 0,
    awaiting: null,
    avoid: null,
    why: "",
    stats: { legs: 0, vetoed: 0, approved: 0, overrides: 0, recoveries: 0 },
    savedMode: null,
    savedGoal: null,
  }));
  return {
    t: 0,
    drones,
    craters: [],
    targets,
    anchors: [
      { name: "Lander", ...station },
      { name: "Beacon A", x: 120, y: 160 },
      { name: "Beacon B", x: 130, y: 810 },
    ],
    station,
    reserve: 11,
    map: new Uint8Array(GW * GH),
    mapDirty: true,
    queue: [],
    log: ["[SYS] Swarm deployed at lander. Awaiting commands."],
    latency,
    autonomy,
    proposals: [],
    nextPid: 1,
    blackoutUntil: 0,
    wasBlackout: false,
  };
}

export function send(s: Sim, id: number, c: Omit<Cmd, "at" | "id">) {
  s.queue.push({ ...c, id, at: s.t + s.latency });
}

export function startBlackout(s: Sim, seconds: number) {
  s.blackoutUntil = s.t + seconds;
  log(s, `Link to operator lost for ${seconds}s. Drones continue under the "${s.autonomy}" policy.`, "SYS");
}

export const isBlackout = (s: Sim) => s.t < s.blackoutUntil;

function log(s: Sim, m: string, actor: Actor = "SYS") {
  s.log = [`T+${s.t.toFixed(0)}s [${actor}] ${m}`, ...s.log].slice(0, 12);
}

export function mappedFraction(s: Sim) {
  let n = 0;
  for (let i = 0; i < s.map.length; i++) n += s.map[i];
  return n / s.map.length;
}

const homeCost = (s: Sim, p: Pt) => Math.hypot(p.x - s.station.x, p.y - s.station.y) * FUEL_PER_UNIT;

// Localisation-aware survey planner: prefers nearby unmapped ground that stays inside radio range
// of an anchor or a neighbouring drone, and spreads drones apart.
function nearestUnmapped(s: Sim, d: Drone): { p: Pt; why: string } | null {
  let best: Pt | null = null;
  let bd = Infinity;
  let bestRefs = 0;
  for (let gy = 0; gy < GH; gy += 2) {
    for (let gx = 0; gx < GW; gx += 2) {
      if (s.map[gy * GW + gx]) continue;
      const p = { x: gx * CELL + CELL / 2, y: gy * CELL + CELL / 2 };
      let dist = Math.hypot(p.x - d.x, p.y - d.y);
      for (const o of s.drones) {
        if (o !== d && o.goal && Math.hypot(o.goal.x - p.x, o.goal.y - p.y) < 160) dist += 400;
      }
      if (d.avoid && d.avoid.until > s.t && Math.hypot(d.avoid.x - p.x, d.avoid.y - p.y) < 220) dist += 2000;
      const refs =
        s.anchors.filter((a) => Math.hypot(a.x - p.x, a.y - p.y) < ANCHOR_RANGE).length +
        s.drones.filter((o) => o !== d && Math.hypot(o.x - p.x, o.y - p.y) < COMM_RANGE).length;
      if (refs < 2) dist += 300;
      if (dist < bd) {
        bd = dist;
        best = p;
        bestRefs = refs;
      }
    }
  }
  if (!best) return null;
  return {
    p: best,
    why: `Nearest unmapped ground, spread from other drones; ${bestRefs} position reference${bestRefs === 1 ? "" : "s"} in range there${bestRefs < 2 ? " (weak, expect drift)" : ""}.`,
  };
}

const CMD_LABEL: Record<string, string> = { auto: "auto survey", goto: "go to point", add: "add waypoint", rtb: "return to lander", stop: "hold" };

function dropProposal(s: Sim, d: Drone) {
  s.proposals = s.proposals.filter((p) => p.drone !== d.id);
  d.awaiting = null;
}

function apply(s: Sim, c: Cmd) {
  const d = s.drones[c.id];
  if (c.type === "veto" || c.type === "approve") {
    const p = s.proposals.find((q) => q.pid === c.pid);
    if (!p) {
      log(s, `${c.type === "veto" ? "Veto" : "Approval"} for ${d.name} arrived after it had already started. Cannot recall.`, "SYS");
      return;
    }
    s.proposals = s.proposals.filter((q) => q !== p);
    d.awaiting = null;
    if (c.type === "veto") {
      d.stats.vetoed++;
      d.avoid = { x: p.goal.x, y: p.goal.y, until: s.t + 120 };
      d.goal = null;
      log(s, `${d.name} leg vetoed. It will choose another area.`, "CREW");
    } else {
      d.stats.approved++;
      d.goal = p.goal;
      d.why = p.why;
      log(s, `${d.name} leg approved.`, "CREW");
    }
    return;
  }
  if (d.awaiting !== null) dropProposal(s, d);
  if (c.type !== "input") {
    if (d.mode === "auto") d.stats.overrides++;
    log(s, `${d.name}: ${CMD_LABEL[c.type]}`, "CREW");
  }
  if (d.mode === "charging" && c.type !== "stop") {
    if (c.type === "auto") d.resume = true;
    return;
  }
  d.savedMode = null;
  switch (c.type) {
    case "auto":
      d.mode = "auto";
      d.path = [];
      d.goal = null;
      break;
    case "goto":
      d.mode = "waypoint";
      d.path = c.pts ?? [];
      d.goal = d.path[0] ?? null;
      break;
    case "add":
      d.mode = "waypoint";
      d.path.push(...(c.pts ?? []));
      d.goal = d.path[0] ?? null;
      break;
    case "rtb":
      d.mode = "rtb";
      d.path = [];
      d.goal = s.station;
      d.resume = false;
      break;
    case "stop":
      d.mode = "idle";
      d.path = [];
      d.goal = null;
      d.resume = false;
      d.input = { x: 0, y: 0 };
      break;
    case "input":
      d.mode = "direct";
      d.input = c.v ?? { x: 0, y: 0 };
      d.goal = null;
      break;
  }
}

function reveal(s: Sim, d: Drone) {
  const gx = Math.floor(d.x / CELL);
  const gy = Math.floor(d.y / CELL);
  for (let y = gy - 3; y <= gy + 3; y++) {
    for (let x = gx - 3; x <= gx + 3; x++) {
      if (x < 0 || y < 0 || x >= GW || y >= GH) continue;
      if ((x - gx) ** 2 + (y - gy) ** 2 > 10) continue;
      if (!s.map[y * GW + x]) {
        s.map[y * GW + x] = 1;
        s.mapDirty = true;
      }
    }
  }
  for (const t of s.targets) {
    if (!t.found && Math.hypot(t.x - d.x, t.y - d.y) < 55) {
      t.found = true;
      log(s, `${d.name} reached ${t.name}`, "AUTO");
    }
  }
}

// Position error from ranging geometry: Fisher information of range measurements to
// anchors and neighbouring drones. Collinear / few neighbours => large error.
function localize(s: Sim, d: Drone, dt: number) {
  let a = 0, b = 0, c = 0, n = 0;
  const add = (p: Pt) => {
    const dx = p.x - d.x, dy = p.y - d.y;
    const dist = Math.hypot(dx, dy) || 1;
    const sigma = (1 + dist / 300) * 0.25; // a unit is ~5x coarser than before, so range noise in units is smaller
    const ux = dx / dist, uy = dy / dist;
    a += (ux * ux) / (sigma * sigma);
    b += (ux * uy) / (sigma * sigma);
    c += (uy * uy) / (sigma * sigma);
    n++;
  };
  for (const an of s.anchors) if (Math.hypot(an.x - d.x, an.y - d.y) < ANCHOR_RANGE) add(an);
  for (const o of s.drones) if (o !== d && Math.hypot(o.x - d.x, o.y - d.y) < COMM_RANGE) add(o);
  d.links = n;
  const det = a * c - b * b;
  if (n >= 2 && det > 1.6e-3) {
    const geom = Math.sqrt((a + c) / det) * MPU; // metres
    d.err += (geom - d.err) * Math.min(1, dt * 2);
    d.moved = 0;
  } else {
    d.err += d.moved * MPU * 0.03; // dead-reckoning drift (metres)
    d.moved = 0;
  }
}

export function step(s: Sim, dt: number) {
  s.t += dt;
  const blackout = isBlackout(s);
  if (s.wasBlackout && !blackout) log(s, "Link restored. Held commands are being delivered.", "SYS");
  s.wasBlackout = blackout;

  const due = s.queue.filter((q) => q.at <= s.t);
  if (due.length) {
    if (blackout) due.forEach((q) => (q.at = s.blackoutUntil + s.latency));
    else {
      s.queue = s.queue.filter((q) => q.at > s.t);
      due.forEach((q) => apply(s, q));
    }
  }

  // proposals whose veto window closed proceed on their own; stale ones are dropped
  for (const p of [...s.proposals]) {
    const d = s.drones[p.drone];
    if (d.mode !== "auto" || d.awaiting !== p.pid) {
      s.proposals = s.proposals.filter((q) => q !== p);
      if (d.awaiting === p.pid) d.awaiting = null;
    } else if (p.deadline !== null && s.t >= p.deadline) {
      s.proposals = s.proposals.filter((q) => q !== p);
      d.awaiting = null;
      d.goal = p.goal;
      d.why = p.why;
      log(s, `${d.name} proceeds with its leg (no veto received in time).`, "AUTO");
    }
  }

  for (const d of s.drones) {
    const px = d.x, py = d.y;
    if (d.mode === "charging") {
      const need = Math.min(10 * dt, 100 - d.fuel);
      const pay = need / 100;
      if (s.reserve < pay) {
        d.mode = "idle";
        log(s, `Station propellant exhausted. ${d.name} grounded.`, "SYS");
      } else {
        s.reserve -= pay;
        d.fuel += need;
        if (d.fuel >= 99.9) {
          d.fuel = 100;
          d.mode = d.resume ? "auto" : "idle";
          log(s, `${d.name} refuelled (${s.reserve.toFixed(1)} tanks left).`, "AUTO");
        }
      }
    } else if (d.mode !== "idle") {
      // self-recovery of position fix, no operator needed
      if ((d.mode === "auto" || d.mode === "waypoint") && d.err > 60) {
        const ref = [...s.anchors].sort((a, b) => Math.hypot(a.x - d.x, a.y - d.y) - Math.hypot(b.x - d.x, b.y - d.y))[0];
        const dist = Math.hypot(ref.x - d.x, ref.y - d.y) || 1;
        if (d.awaiting !== null) dropProposal(s, d);
        d.savedMode = d.mode;
        d.savedGoal = d.goal;
        d.mode = "recover";
        d.goal = { x: ref.x + ((d.x - ref.x) / dist) * 200, y: ref.y + ((d.y - ref.y) / dist) * 200 };
        d.why = `Position error ±${d.err.toFixed(0)} m is too large, so it moves toward ${ref.name} to regain a good fix.`;
        d.stats.recoveries++;
        log(s, `${d.name} lost its fix (±${d.err.toFixed(0)} m). Moving toward ${ref.name} to recover.`, "AUTO");
      }
      let vx = 0, vy = 0;
      if (d.mode === "direct") {
        const m = Math.hypot(d.input.x, d.input.y) || 1;
        const on = d.input.x || d.input.y ? 1 : 0;
        vx = (d.input.x / m) * SPEED * on;
        vy = (d.input.y / m) * SPEED * on;
      } else {
        if (d.mode === "auto" && !d.goal && d.awaiting === null) {
          const cand = nearestUnmapped(s, d);
          if (cand) {
            const dist = Math.hypot(cand.p.x - d.x, cand.p.y - d.y);
            const fuelEst = dist * FUEL_PER_UNIT;
            const thin = d.fuel - fuelEst - homeCost(s, cand.p) < 35;
            if (s.autonomy !== "full" && (dist > REVIEW_LEG || thin)) {
              const reason = `${cand.why}${thin ? " Leaves a thin fuel margin." : ""}`;
              const pid = s.nextPid++;
              s.proposals.push({
                pid,
                drone: d.id,
                goal: cand.p,
                dist,
                fuelEst,
                why: reason,
                createdAt: s.t,
                deadline: s.autonomy === "veto" ? s.t + 2 * s.latency + 8 : null,
              });
              d.awaiting = pid;
              log(s, `${d.name} proposes a ${(dist * MPU / 1000).toFixed(1)} km leg${s.autonomy === "veto" ? " and will proceed unless vetoed" : " and waits for approval"}.`, "AUTO");
            } else {
              d.goal = cand.p;
              d.why = cand.why;
            }
          }
        }
        if (d.mode === "waypoint" && !d.goal) d.mode = "idle";
        if (d.goal) {
          const dx = d.goal.x - d.x, dy = d.goal.y - d.y, dist = Math.hypot(dx, dy);
          if (dist < 6) {
            if (d.mode === "rtb") {
              d.x = s.station.x + (d.id % 3) * 14 - 14;
              d.y = s.station.y + Math.floor(d.id / 3) * 14 - 7;
              d.mode = "charging";
              d.flights++;
              d.goal = null;
            } else if (d.mode === "waypoint") {
              d.path.shift();
              d.goal = d.path[0] ?? null;
              if (!d.goal) d.mode = "idle";
            } else {
              if (d.mode === "auto") d.stats.legs++;
              d.goal = null;
            }
          } else {
            vx = (dx / dist) * SPEED;
            vy = (dy / dist) * SPEED;
          }
        }
      }
      d.x = Math.min(W - 5, Math.max(5, d.x + vx * dt));
      d.y = Math.min(H - 5, Math.max(5, d.y + vy * dt));
      const moved = Math.hypot(d.x - px, d.y - py);
      d.fuel -= moved * FUEL_PER_UNIT;
      d.moved += moved;
      reveal(s, d);

      if (d.mode === "recover" && (d.err < 30 || !d.goal)) {
        d.mode = d.savedMode ?? "auto";
        d.goal = d.savedGoal;
        d.savedMode = null;
        d.why = "Fix recovered, resuming the previous task.";
        log(s, `${d.name} fix recovered (±${d.err.toFixed(0)} m). Resuming.`, "AUTO");
      }

      const home = homeCost(s, d);
      if (d.mode !== "rtb" && d.mode !== "idle" && d.fuel < home + 8) {
        d.resume = d.mode === "auto" || d.savedMode === "auto";
        if (d.awaiting !== null) dropProposal(s, d);
        d.mode = "rtb";
        d.path = [];
        d.goal = s.station;
        d.why = "Fuel is down to the return margin, so it heads home to refuel.";
        log(s, `${d.name} low fuel, returning to station on its own.`, "AUTO");
      }
      if (d.fuel <= 0) {
        d.fuel = 0;
        d.mode = "idle";
        d.goal = null;
        log(s, `${d.name} out of fuel at (${d.x.toFixed(0)}, ${d.y.toFixed(0)}). Stranded.`, "SYS");
      }
    }
    localize(s, d, dt);
  }
}

// Deploy one more drone from the lander. It leaves with a full tank, which comes out of the station's propellant.
export function addDrone(s: Sim): string | null {
  if (s.drones.length >= MAX_DRONES) return `Swarm is full (${MAX_DRONES} drones).`;
  if (s.reserve < 1) return "Not enough propellant at the station to fuel another drone.";
  const i = s.drones.length;
  s.drones.push({
    id: i,
    name: `LUNA-${i + 1}`,
    color: COLORS[i],
    x: s.station.x + (i % 3) * 14 - 14,
    y: s.station.y + Math.floor(i / 3) * 14 - 7,
    mode: "idle",
    resume: false,
    fuel: 100,
    flights: 0,
    path: [],
    goal: null,
    input: { x: 0, y: 0 },
    err: 1,
    links: 3,
    moved: 0,
    awaiting: null,
    avoid: null,
    why: "",
    stats: { legs: 0, vetoed: 0, approved: 0, overrides: 0, recoveries: 0 },
    savedMode: null,
    savedGoal: null,
  });
  s.reserve -= 1;
  log(s, `LUNA-${i + 1} deployed at the lander (1 tank used, ${s.reserve.toFixed(1)} left).`, "CREW");
  return null;
}
