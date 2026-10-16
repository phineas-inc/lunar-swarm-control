import { useCallback, useEffect, useRef, useState } from "react";
import { COMM_RANGE, type Pt, GH, GW, H, MAP_KM, MPU, PIT, W, createSim, isBlackout, mappedFraction, send, startBlackout, step, type Autonomy, type Mode, type Sim, type Stats } from "./sim";

// ---------- Human-factors design notes ----------
// 1. Situational awareness: attention queue (perceive) -> mode + intent per drone (comprehend) -> ghost
//    markers for commands still in transit (project). The operator never scans the map for trouble.
// 2. Alerts: three tiers, each coded by colour AND shape AND word; sorted by urgency; click to select.
// 3. Latency: commands show as dashed ghosts with a countdown and can be cancelled; no silent waiting.
// 4. Automation transparency: every drone states what it is doing and why.
// 5. Workload: manual flight is costly, so a workload gauge nudges the operator to hand off to auto.
// 6. Error tolerance: swarm-wide actions need a second press; pending commands can be cancelled.
// 7. Ergonomics: >=40px targets, >=12px text, Okabe-Ito palette, numbered drones, low-glare mode,
//    reduced motion respected, break reminder, aria-live for new warnings.

const MODE: Record<Mode, { label: string; icon: string }> = {
  idle: { label: "Holding", icon: "‖" },
  auto: { label: "Auto survey", icon: "◎" },
  waypoint: { label: "Waypoints", icon: "➜" },
  direct: { label: "Manual", icon: "✋" },
  rtb: { label: "Returning", icon: "⌂" },
  charging: { label: "Refuelling", icon: "⚡" },
  recover: { label: "Recovering fix", icon: "⟲" },
};

type Sev = "warning" | "caution" | "advisory";
const SEV: Record<Sev, { icon: string; color: string; rank: number }> = {
  warning: { icon: "▲", color: "#ff6b6b", rank: 0 },
  caution: { icon: "◆", color: "#ffb454", rank: 1 },
  advisory: { icon: "●", color: "#8ab4ff", rank: 2 },
};

type Cam = { x: number; y: number; z: number; tx: number; ty: number; tz: number };
const MAX_Z = 8;
function toLatLon(p: Pt) {
  const lat = PIT.lat - (p.y / H - 0.5) * (MAP_KM.h / 30.34);
  const lon = PIT.lon + ((p.x / W - 0.5) * (MAP_KM.w / 30.34)) / Math.cos((PIT.lat * Math.PI) / 180);
  return `${Math.abs(lat).toFixed(3)}°${lat >= 0 ? "N" : "S"} ${lon.toFixed(3)}°E`;
}

type Alert = { key: string; sev: Sev; drone: number | null; text: string; action: string };
type DView = { id: number; name: string; color: string; mode: Mode; fuel: number; err: number; links: number; flights: number; intent: string; pending: number; why: string; stats: Stats; conf: number; confWhy: string; awaiting: boolean };
type PView = { pid: number; drone: number; name: string; color: string; km: number; fuelEst: number; why: string; left: number | null };
type Snap = {
  drones: DView[];
  alerts: Alert[];
  mapped: number;
  found: number;
  targets: number;
  reserve: number;
  log: string[];
  t: number;
  workload: number;
  proposals: PView[];
  blackout: boolean;
  blackoutLeft: number;
  autonomy: Autonomy;
};

function snap(s: Sim): Snap {
  const alerts: Alert[] = [];
  const drones: DView[] = s.drones.map((d) => {
    const pending = s.queue.filter((q) => q.id === d.id).length;
    let intent = "Waiting for command";
    if (d.mode === "auto") intent = d.goal ? "Surveying nearest unmapped area" : "Choosing next area";
    if (d.mode === "waypoint") intent = `${d.path.length} waypoint${d.path.length === 1 ? "" : "s"} left`;
    if (d.mode === "direct") intent = "You are flying it";
    if (d.mode === "rtb") intent = d.resume ? "Low fuel · refuel then resume survey" : "Returning to lander";
    if (d.mode === "charging") intent = s.reserve > 0 ? "Taking on propellant" : "Station out of propellant";
    if (d.mode === "idle" && d.fuel === 0) intent = "Out of fuel";
    if (d.fuel === 0 && d.mode === "idle")
      alerts.push({ key: `strand${d.id}`, sev: "warning", drone: d.id, text: `${d.name} stranded, no fuel`, action: "Needs rescue plan" });
    else if (d.mode !== "rtb" && d.mode !== "charging" && d.fuel < 15)
      alerts.push({ key: `fuel${d.id}`, sev: "warning", drone: d.id, text: `${d.name} fuel critical (${d.fuel.toFixed(0)}%)`, action: "Recall now" });
    if (d.mode === "recover") alerts.push({ key: `rec${d.id}`, sev: "advisory", drone: d.id, text: `${d.name} is recovering its position fix`, action: "Handled on board, no action needed" });
    else if (d.err > 80) alerts.push({ key: `lost${d.id}`, sev: "warning", drone: d.id, text: `${d.name} position lost (±${d.err.toFixed(0)} m)`, action: "Move toward lander or other drones" });
    else if (d.err > 40) alerts.push({ key: `err${d.id}`, sev: "caution", drone: d.id, text: `${d.name} poorly localised (±${d.err.toFixed(0)} m)`, action: "Bring closer to beacons or drones" });
    if (d.links < 2 && d.mode !== "charging" && d.err <= 40)
      alerts.push({ key: `link${d.id}`, sev: "advisory", drone: d.id, text: `${d.name} has ${d.links} range link${d.links === 1 ? "" : "s"}`, action: "Drift will grow" });
    const home = Math.hypot(d.x - s.station.x, d.y - s.station.y) * 0.02;
    const margin = d.fuel - home;
    const conf = Math.max(0, Math.min(1, 1 - d.err / 100, margin / 40));
    const confWhy = `fix ±${d.err.toFixed(0)} m, ${d.links} refs, fuel margin ${Math.max(0, margin).toFixed(0)}% after the trip home`;
    if (d.awaiting !== null) intent = s.autonomy === "ask" ? "Holding for your approval" : "Holding briefly for a possible veto";
    return { id: d.id, name: d.name, color: d.color, mode: d.mode, fuel: d.fuel, err: d.err, links: d.links, flights: d.flights, intent, pending, why: d.why, stats: d.stats, conf, confWhy, awaiting: d.awaiting !== null };
  });
  if (s.reserve < 3) alerts.push({ key: "reserve", sev: s.reserve < 1 ? "warning" : "caution", drone: null, text: `Station propellant low (${s.reserve.toFixed(1)} tanks)`, action: "Limit refuels; plan remaining hops" });
  alerts.sort((a, b) => SEV[a.sev].rank - SEV[b.sev].rank);
  const manual = s.drones.filter((d) => d.mode === "direct").length;
  const workload = Math.min(1, manual * 0.35 + alerts.filter((a) => a.sev === "warning").length * 0.25 + alerts.filter((a) => a.sev === "caution").length * 0.1 + s.queue.length * 0.03);
  const blackout = isBlackout(s);
  const proposals: PView[] = blackout
    ? []
    : s.proposals
        .filter((p) => s.t >= p.createdAt + s.latency) // the operator only learns of it after the downlink delay
        .map((p) => ({ pid: p.pid, drone: p.drone, name: s.drones[p.drone].name, color: s.drones[p.drone].color, km: (p.dist * 25) / 1000, fuelEst: p.fuelEst, why: p.why, left: p.deadline === null ? null : Math.max(0, p.deadline - s.t) }));
  return { proposals, blackout, blackoutLeft: Math.max(0, s.blackoutUntil - s.t), autonomy: s.autonomy, drones, alerts, mapped: mappedFraction(s), found: s.targets.filter((t) => t.found).length, targets: s.targets.length, reserve: s.reserve, log: s.log, t: s.t, workload };
}

export default function DroneApp() {
  const canvas = useRef<HTMLCanvasElement>(null);
  const sim = useRef<Sim>(createSim());
  const fog = useRef<HTMLCanvasElement | null>(null);
  const moon = useRef<HTMLImageElement | null>(null);
  const cam = useRef<Cam>({ x: W / 2, y: H / 2, z: 1, tx: W / 2, ty: H / 2, tz: 1 });
  const followRef = useRef(false);
  const pendingRef = useRef<Pt | null>(null);
  const miniRef = useRef<{ x: number; y: number; w: number; h: number } | null>(null);
  const orbitRef = useRef(true);
  const [follow, setFollow] = useState(false);
  const [orbit, setOrbit] = useState(true);
  const [pending, setPending] = useState<Pt | null>(null);
  const [camInfo, setCamInfo] = useState("");
  useEffect(() => {
    const im = new Image();
    im.src = `${import.meta.env.BASE_URL}moon/tranquillitatis.jpg`;
    moon.current = im;
  }, []);
  const [selected, setSelected] = useState(0);
  const selRef = useRef(0);
  const [speed, setSpeed] = useState(4);
  const speedRef = useRef(4);
  const [latency, setLatency] = useState(1.3);
  const [autonomy, setAutonomy] = useState<Autonomy>("veto");
  const [lowGlare, setLowGlare] = useState(false);
  const [confirm, setConfirm] = useState<string | null>(null);
  const [view, setView] = useState<Snap>(() => snap(sim.current));
  const [minutes, setMinutes] = useState(0);
  const [announce, setAnnounce] = useState("");
  const seen = useRef(new Set<string>());
  selRef.current = selected;
  speedRef.current = speed;
  orbitRef.current = orbit;

  useEffect(() => {
    const fresh = view.alerts.filter((a) => !seen.current.has(a.key));
    const live = new Set(view.alerts.map((a) => a.key));
    seen.current.forEach((k) => !live.has(k) && seen.current.delete(k));
    fresh.forEach((a) => seen.current.add(a.key));
    if (fresh.length) setAnnounce(fresh.map((a) => `${a.sev}: ${a.text}. ${a.action}`).join(". "));
  }, [view.alerts]);

  // keyboard alternative to clicking the map
  const places = [
    { label: "Lander", p: sim.current.station },
    ...sim.current.anchors.slice(1).map((a) => ({ label: a.name, p: a })),
    ...sim.current.targets.filter((t) => t.found).map((t) => ({ label: t.name, p: t })),
  ];

  // ---- camera: world is W x H units; screen mapping = (p - cam) * z + centre ----
  const clampCam = (c: Cam) => {
    c.tz = Math.min(MAX_Z, Math.max(1, c.tz));
    c.z = Math.min(MAX_Z, Math.max(1, c.z));
    const hx = W / (2 * c.tz), hy = H / (2 * c.tz);
    c.tx = Math.min(W - hx, Math.max(hx, c.tx));
    c.ty = Math.min(H - hy, Math.max(hy, c.ty));
    const ax = W / (2 * c.z), ay = H / (2 * c.z);
    c.x = Math.min(W - ax, Math.max(ax, c.x));
    c.y = Math.min(H - ay, Math.max(ay, c.y));
  };
  const zoomAt = useCallback((factor: number, sx = W / 2, sy = H / 2, immediate = false) => {
    const c = cam.current;
    const base = immediate ? c.z : c.tz;
    const nz = Math.min(MAX_Z, Math.max(1, base * factor));
    const bx = immediate ? c.x : c.tx, by = immediate ? c.y : c.ty;
    const wx = bx + (sx - W / 2) / base, wy = by + (sy - H / 2) / base; // world point under the anchor stays put
    c.tz = nz;
    c.tx = wx - (sx - W / 2) / nz;
    c.ty = wy - (sy - H / 2) / nz;
    if (immediate) {
      c.z = nz;
      c.x = c.tx;
      c.y = c.ty;
    }
    clampCam(c);
  }, []);
  const panBy = useCallback((dx: number, dy: number, immediate = false) => {
    const c = cam.current;
    setFollow(false);
    followRef.current = false;
    c.tx += dx;
    c.ty += dy;
    if (immediate) {
      c.x = c.tx;
      c.y = c.ty;
    }
    clampCam(c);
  }, []);
  const fit = useCallback(() => {
    followRef.current = false;
    setFollow(false);
    Object.assign(cam.current, { tx: W / 2, ty: H / 2, tz: 1 });
  }, []);
  const toggleFollow = useCallback((on?: boolean) => {
    const v = on ?? !followRef.current;
    followRef.current = v;
    setFollow(v);
    if (v && cam.current.tz < 2) cam.current.tz = 2;
  }, []);

  const draw = useCallback(() => {
    const c = canvas.current;
    if (!c) return;
    const g = c.getContext("2d")!;
    const s = sim.current;
    const sel = selRef.current;
    const k = cam.current;
    const u = (W / (c.clientWidth || W)) * (orbitRef.current ? 1.15 : 1); // css px -> canvas px, so markers keep a constant physical size
    if (followRef.current) {
      k.tx = s.drones[sel].x;
      k.ty = s.drones[sel].y;
      clampCam(k);
    }
    // eased, no inertia: converges and stops exactly, so the view never keeps drifting
    const e = 0.28;
    k.x += (k.tx - k.x) * e;
    k.y += (k.ty - k.y) * e;
    k.z += (k.tz - k.z) * e;
    if (Math.abs(k.tx - k.x) < 0.05) k.x = k.tx;
    if (Math.abs(k.ty - k.y) < 0.05) k.y = k.ty;
    if (Math.abs(k.tz - k.z) < 0.001) k.z = k.tz;
    const z = k.z;
    const sx = (x: number) => (x - k.x) * z + W / 2;
    const sy = (y: number) => (y - k.y) * z + H / 2;

    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, W, H);
    g.fillStyle = "#2a2c32";
    g.fillRect(0, 0, W, H);

    // world layer: imagery + survey coverage
    g.save();
    g.translate(W / 2, H / 2);
    g.scale(z, z);
    g.translate(-k.x, -k.y);
    g.imageSmoothingEnabled = true;
    if (moon.current?.complete && moon.current.naturalWidth) g.drawImage(moon.current, 0, 0, W, H);
    if (!fog.current) {
      fog.current = document.createElement("canvas");
      fog.current.width = GW;
      fog.current.height = GH;
      s.mapDirty = true;
    }
    if (s.mapDirty) {
      const fc = fog.current.getContext("2d")!;
      const img = fc.createImageData(GW, GH);
      for (let i = 0; i < s.map.length; i++) {
        img.data[i * 4] = 6;
        img.data[i * 4 + 1] = 8;
        img.data[i * 4 + 2] = 14;
        img.data[i * 4 + 3] = s.map[i] ? 0 : 120;
      }
      fc.putImageData(img, 0, 0);
      s.mapDirty = false;
    }
    g.drawImage(fog.current, 0, 0, W, H);
    g.restore();

    // comm links
    g.lineWidth = 1.5 * u;
    g.strokeStyle = "rgba(255,255,255,0.16)";
    for (const a of s.drones)
      for (const b of s.drones)
        if (b.id > a.id && Math.hypot(a.x - b.x, a.y - b.y) < COMM_RANGE) {
          g.beginPath();
          g.moveTo(sx(a.x), sy(a.y));
          g.lineTo(sx(b.x), sy(b.y));
          g.stroke();
        }

    g.font = `600 ${14 * u}px ui-sans-serif, system-ui, sans-serif`;
    for (const an of s.anchors) {
      g.fillStyle = "#ff8a8a";
      g.fillRect(sx(an.x) - 7 * u, sy(an.y) - 7 * u, 14 * u, 14 * u);
      g.fillStyle = "#ffd5d5";
      g.fillText(an.name, sx(an.x) + 12 * u, sy(an.y) + 5 * u);
    }
    for (const t of s.targets) {
      if (!t.found) continue;
      const x = sx(t.x), y = sy(t.y), r = 11 * u;
      g.strokeStyle = "#fff";
      g.fillStyle = "rgba(255,255,255,0.15)";
      g.lineWidth = 2.5 * u;
      g.beginPath();
      g.moveTo(x, y - r);
      g.lineTo(x + r, y);
      g.lineTo(x, y + r);
      g.lineTo(x - r, y);
      g.closePath();
      g.fill();
      g.stroke();
      g.fillStyle = "#fff";
      g.fillText(t.name, x + r + 5 * u, y - 8 * u);
    }

    // ghost markers for commands still in transit
    const pulse = window.matchMedia("(prefers-reduced-motion: reduce)").matches ? 1 : 0.6 + 0.4 * Math.sin(s.t * 6);
    for (const q of s.queue) {
      const d = s.drones[q.id];
      const p = q.pts?.[0];
      if (!p || (q.type !== "goto" && q.type !== "add")) continue;
      g.strokeStyle = d.color;
      g.globalAlpha = pulse;
      g.setLineDash([4 * u, 5 * u]);
      g.lineWidth = 2.5 * u;
      g.beginPath();
      g.arc(sx(p.x), sy(p.y), 13 * u, 0, 7);
      g.stroke();
      g.setLineDash([]);
      g.fillStyle = d.color;
      g.font = `600 ${14 * u}px ui-monospace, monospace`;
      g.fillText(`${Math.max(0, q.at - s.t).toFixed(1)}s`, sx(p.x) + 17 * u, sy(p.y) + 5 * u);
      g.globalAlpha = 1;
    }

    // pending (unconfirmed) target
    const pd = pendingRef.current;
    if (pd) {
      const x = sx(pd.x), y = sy(pd.y);
      g.strokeStyle = "#fff";
      g.lineWidth = 3 * u;
      g.beginPath();
      g.arc(x, y, 16 * u, 0, 7);
      g.moveTo(x - 24 * u, y);
      g.lineTo(x + 24 * u, y);
      g.moveTo(x, y - 24 * u);
      g.lineTo(x, y + 24 * u);
      g.stroke();
    }

    for (const d of s.drones) {
      const isSel = d.id === sel;
      const x = sx(d.x), y = sy(d.y);
      if (d.goal || d.path.length) {
        g.setLineDash([7 * u, 7 * u]);
        g.strokeStyle = d.color + (isSel ? "ee" : "66");
        g.lineWidth = (isSel ? 2.5 : 1.5) * u;
        g.beginPath();
        g.moveTo(x, y);
        for (const p of d.path.length ? d.path : d.goal ? [d.goal] : []) g.lineTo(sx(p.x), sy(p.y));
        g.stroke();
        g.setLineDash([]);
      }
      // position uncertainty halo (real size on the ground)
      g.beginPath();
      g.arc(x, y, Math.max(14 * u, d.err * 1.5 * z), 0, 7);
      g.fillStyle = d.err > 40 ? "rgba(255,107,107,0.22)" : "rgba(255,255,255,0.08)";
      g.fill();
      if (d.err > 40) {
        g.strokeStyle = "#ff6b6b";
        g.setLineDash([3 * u, 4 * u]);
        g.lineWidth = 1.5 * u;
        g.stroke();
        g.setLineDash([]);
      }
      g.beginPath();
      g.arc(x, y, 13 * u, 0, 7);
      g.fillStyle = d.color;
      g.fill();
      g.lineWidth = 2 * u;
      g.strokeStyle = "#0b0d12";
      g.stroke();
      g.fillStyle = "#0b0d12";
      g.font = `700 ${15 * u}px ui-sans-serif, system-ui, sans-serif`;
      g.textAlign = "center";
      g.textBaseline = "middle";
      g.fillText(String(d.id + 1), x, y + u);
      g.textAlign = "start";
      g.textBaseline = "alphabetic";
      if (isSel) {
        g.strokeStyle = "#fff";
        g.lineWidth = 3 * u;
        g.beginPath();
        g.arc(x, y, 22 * u, 0, 7);
        g.stroke();
        g.fillStyle = "#fff";
        g.font = `600 ${15 * u}px ui-sans-serif, system-ui, sans-serif`;
        g.fillText(d.name, x + 28 * u, y - 20 * u);
      }
    }

    // scale bar: longest of 10/5/2/1/0.5 km that fits
    const kms = [10, 5, 2, 1, 0.5];
    const km = kms.find((v) => (v * 1000 * z) / MPU < 220 * u) ?? 0.5;
    const barLen = (km * 1000 * z) / MPU;
    g.fillStyle = "rgba(0,0,0,0.6)";
    g.fillRect(W - barLen - 36 * u, H - 44 * u, barLen + 24 * u, 34 * u);
    g.fillStyle = "#fff";
    g.fillRect(W - barLen - 24 * u, H - 24 * u, barLen, 4 * u);
    g.font = `600 ${14 * u}px ui-sans-serif, system-ui, sans-serif`;
    g.fillText(`${km} km`, W - barLen - 24 * u, H - 29 * u);

    // overview inset, shown when zoomed in (spatial orientation); tap it to jump
    if (z > 1.05) {
      const mw = 170 * u, mh = mw * (H / W), mx = 12 * u, my = H - mh - 12 * u;
      g.globalAlpha = 0.92;
      if (moon.current?.complete) g.drawImage(moon.current, mx, my, mw, mh);
      g.globalAlpha = 1;
      for (const d of s.drones) {
        g.fillStyle = d.color;
        g.fillRect(mx + (d.x / W) * mw - 2 * u, my + (d.y / H) * mh - 2 * u, 4 * u, 4 * u);
      }
      g.strokeStyle = "#fff";
      g.lineWidth = 2 * u;
      g.strokeRect(mx + ((k.x - W / (2 * z)) / W) * mw, my + ((k.y - H / (2 * z)) / H) * mh, mw / z, mh / z);
      g.strokeStyle = "rgba(255,255,255,0.5)";
      g.lineWidth = 1 * u;
      g.strokeRect(mx, my, mw, mh);
      miniRef.current = { x: mx, y: my, w: mw, h: mh };
    } else miniRef.current = null;

    // centre mark (what the lat/lon readout refers to)
    if (!followRef.current) {
      g.strokeStyle = "rgba(255,255,255,0.55)";
      g.lineWidth = 1.5 * u;
      g.beginPath();
      g.moveTo(W / 2 - 9 * u, H / 2);
      g.lineTo(W / 2 + 9 * u, H / 2);
      g.moveTo(W / 2, H / 2 - 9 * u);
      g.lineTo(W / 2, H / 2 + 9 * u);
      g.stroke();
    }
    // north reference (needed because there is no "up" in orbit)
    g.fillStyle = "rgba(0,0,0,0.6)";
    g.fillRect(W - 46 * u, 10 * u, 36 * u, 40 * u);
    g.fillStyle = "#fff";
    g.font = `700 ${15 * u}px ui-sans-serif, system-ui, sans-serif`;
    g.textAlign = "center";
    g.fillText("N", W - 28 * u, 28 * u);
    g.fillText("↑", W - 28 * u, 46 * u);
    g.textAlign = "start";
  }, []);

  // ---- map input: drag / pinch / wheel, with tolerance for unsteady hands ----
  const ptrs = useRef(new Map<number, Pt>());
  const gesture = useRef({ start: { x: 0, y: 0 }, dragged: false, pinch: 0 });
  const toCanvas = (e: { clientX: number; clientY: number }) => {
    const r = canvas.current!.getBoundingClientRect();
    return { x: ((e.clientX - r.left) / r.width) * W, y: ((e.clientY - r.top) / r.height) * H, k: W / r.width };
  };
  const tap = (p: Pt & { k: number }) => {
    const m = miniRef.current;
    const k = cam.current;
    if (m && p.x > m.x && p.x < m.x + m.w && p.y > m.y && p.y < m.y + m.h) {
      panBy(((p.x - m.x) / m.w) * W - k.tx, ((p.y - m.y) / m.h) * H - k.ty);
      return;
    }
    const s = sim.current;
    const reach = (orbitRef.current ? 34 : 24) * p.k; // generous hit area
    let best: number | null = null, bd = reach;
    for (const d of s.drones) {
      const dd = Math.hypot((d.x - k.x) * k.z + W / 2 - p.x, (d.y - k.y) * k.z + H / 2 - p.y);
      if (dd < bd) {
        bd = dd;
        best = d.id;
      }
    }
    if (best !== null) {
      setSelected(best);
      return;
    }
    // a tap only PLACES a target; sending needs an explicit confirm (accidental touches are common in microgravity)
    const w = { x: Math.min(W - 1, Math.max(1, k.x + (p.x - W / 2) / k.z)), y: Math.min(H - 1, Math.max(1, k.y + (p.y - H / 2) / k.z)) };
    pendingRef.current = w;
    setPending(w);
  };
  const onDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    e.currentTarget.setPointerCapture(e.pointerId);
    ptrs.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    gesture.current = { start: { x: e.clientX, y: e.clientY }, dragged: ptrs.current.size > 1, pinch: 0 };
  };
  const onMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const prev = ptrs.current.get(e.pointerId);
    if (!prev) return;
    const g = gesture.current;
    const k = W / canvas.current!.getBoundingClientRect().width;
    if (ptrs.current.size === 2) {
      ptrs.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
      const [a, b] = [...ptrs.current.values()];
      const dist = Math.hypot(a.x - b.x, a.y - b.y);
      if (g.pinch) {
        const mid = toCanvas({ clientX: (a.x + b.x) / 2, clientY: (a.y + b.y) / 2 });
        zoomAt(dist / g.pinch, mid.x, mid.y, true);
      }
      g.pinch = dist;
      g.dragged = true;
      return;
    }
    const dead = orbitRef.current ? 16 : 7; // ignore tremor / drift below this many px
    if (!g.dragged && Math.hypot(e.clientX - g.start.x, e.clientY - g.start.y) < dead) return;
    g.dragged = true;
    panBy((-(e.clientX - prev.x) * k) / cam.current.z, (-(e.clientY - prev.y) * k) / cam.current.z, true);
    ptrs.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
  };
  const onUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const was = ptrs.current.size;
    ptrs.current.delete(e.pointerId);
    if (was === 1 && !gesture.current.dragged) tap(toCanvas(e));
    if (ptrs.current.size < 2) gesture.current.pinch = 0;
  };
  useEffect(() => {
    const c = canvas.current;
    if (!c) return;
    const wheel = (e: WheelEvent) => {
      e.preventDefault();
      const p = toCanvas(e);
      zoomAt(e.deltaY < 0 ? 1.2 : 1 / 1.2, p.x, p.y, true);
    };
    c.addEventListener("wheel", wheel, { passive: false });
    return () => c.removeEventListener("wheel", wheel);
  }, [zoomAt]);

  useEffect(() => {
    let raf = 0;
    let last = performance.now();
    let acc = 0;
    let hud = 0;
    const loop = (now: number) => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      acc += dt * speedRef.current;
      while (acc > 0.05) {
        step(sim.current, 0.05);
        acc -= 0.05;
      }
      draw();
      if ((hud += dt) > 0.15) {
        hud = 0;
        setView(snap(sim.current));
        const c = cam.current;
        setCamInfo(`${toLatLon({ x: c.x, y: c.y })} · ${c.z.toFixed(1)}×${c.z > 4 ? " · finer than the 83 m/px imagery" : ""}`);
      }
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    const clock = setInterval(() => setMinutes((m) => m + 1), 60000);
    return () => {
      cancelAnimationFrame(raf);
      clearInterval(clock);
    };
  }, [draw]);

  useEffect(() => {
    sim.current.latency = latency;
  }, [latency]);

  useEffect(() => {
    sim.current.autonomy = autonomy;
  }, [autonomy]);

  useEffect(() => {
    if (!confirm) return;
    const t = setTimeout(() => setConfirm(null), 3000);
    return () => clearTimeout(t);
  }, [confirm]);

  useEffect(() => {
    const keys = new Set<string>();
    const dirs: Record<string, [number, number]> = {
      arrowup: [0, -1], w: [0, -1], arrowdown: [0, 1], s: [0, 1], arrowleft: [-1, 0], a: [-1, 0], arrowright: [1, 0], d: [1, 0],
    };
    const push = () => {
      let x = 0, y = 0;
      keys.forEach((k) => {
        x += dirs[k][0];
        y += dirs[k][1];
      });
      send(sim.current, selRef.current, { type: "input", v: { x, y } });
    };
    const typing = (e: KeyboardEvent) => e.target instanceof HTMLElement && /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName);
    const down = (e: KeyboardEvent) => {
      if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
      const k = e.key.toLowerCase();
      if (k >= "1" && k <= "6") return setSelected(Number(k) - 1);
      if (k === "+" || k === "=") return zoomAt(2);
      if (k === "-" || k === "_") return zoomAt(0.5);
      if (k === "0") return fit();
      if (k === "c") return toggleFollow();
      if (e.shiftKey && k.startsWith("arrow")) {
        e.preventDefault();
        const st = (W / cam.current.tz) * 0.3;
        panBy(k === "arrowleft" ? -st : k === "arrowright" ? st : 0, k === "arrowup" ? -st * 0.6 : k === "arrowdown" ? st * 0.6 : 0);
        return;
      }
      if (!(k in dirs) || e.repeat) return;
      e.preventDefault();
      keys.add(k);
      push();
    };
    const up = (e: KeyboardEvent) => {
      if (keys.delete(e.key.toLowerCase())) push();
    };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
    };
  }, [zoomAt, fit, toggleFollow, panBy]);

  const cmd = (type: "auto" | "rtb" | "stop", id = selected) => send(sim.current, id, { type });
  const cancelPending = () => {
    sim.current.queue = sim.current.queue.filter((q) => q.id !== selected);
  };
  const guarded = (key: string, fn: () => void) => () => {
    if (confirm === key) {
      setConfirm(null);
      fn();
    } else setConfirm(key);
  };

  const wl = view.workload;
  const btn = `${orbit ? "min-h-14 px-4 text-base" : "min-h-10 px-3 text-sm"} rounded-md border border-white/20 bg-white/[0.06] font-medium hover:bg-white/10 active:bg-white/20 disabled:opacity-40 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-white`;
  const mb = `grid place-items-center rounded-lg border border-white/30 bg-black/70 text-xl font-semibold text-white backdrop-blur hover:bg-black/85 active:bg-white/25 ${orbit ? "h-14 w-14" : "h-11 w-11"}`;
  const hold = (fn: () => void) => {
    let t: ReturnType<typeof setInterval> | undefined;
    const stop = () => t && clearInterval(t);
    return { onPointerDown: () => { fn(); stop(); t = setInterval(fn, 140); }, onPointerUp: stop, onPointerLeave: stop, onPointerCancel: stop, onClick: (e: React.MouseEvent) => e.preventDefault() };
  };
  const st = () => (W / cam.current.tz) * 0.22;
  const pd = pending ? { d: sim.current.drones[selected], ll: toLatLon(pending) } : null
  const pdKm = pd && pending ? (Math.hypot(pending.x - pd.d.x, pending.y - pd.d.y) * 25) / 1000 : 0;
  const closePending = () => { pendingRef.current = null; setPending(null); };
  const sendPending = (add: boolean) => {
    if (pending) send(sim.current, selected, { type: add ? "add" : "goto", pts: [pending] });
    closePending();
  };
  const warnings = view.alerts.filter((a) => a.sev === "warning");

  return (
    <div className="drone-root flex min-h-screen flex-col bg-[#07090d] p-3 text-[15px] text-slate-100 lg:h-screen lg:overflow-hidden lg:p-4" style={lowGlare ? { filter: "sepia(1) hue-rotate(-30deg) saturate(2.2) brightness(0.7)" } : undefined}>
      <style>{`.drone-root :focus-visible{outline:3px solid #fff;outline-offset:2px}`}</style>
      <div className="sr-only" role="status" aria-live="assertive">{announce}</div>
      <header className="mb-3 flex flex-wrap items-center justify-between gap-x-4 gap-y-2">
        <h1 className="text-lg font-semibold">Lunar Swarm Control</h1>
        <div className="flex flex-wrap items-center gap-3 text-sm text-slate-300">
          <span className={`rounded px-2 py-0.5 font-mono text-xs ${view.blackout ? "bg-amber-400/20 text-amber-200" : "bg-emerald-400/10 text-emerald-200"}`}>
            {view.blackout ? `◆ NO LINK ${view.blackoutLeft.toFixed(0)}s` : `● LINK ${latency.toFixed(1)}s`}
          </span>
          <span className="font-mono">MET {Math.floor(view.t / 60)}:{String(Math.floor(view.t % 60)).padStart(2, "0")}</span>
          <span>
            Operator workload{" "}
            <span className="inline-block h-2 w-20 overflow-hidden rounded bg-white/10 align-middle" role="meter" aria-valuenow={Math.round(wl * 100)} aria-valuemin={0} aria-valuemax={100} aria-label="Operator workload">
              <span className="block h-full" style={{ width: `${wl * 100}%`, background: wl > 0.66 ? "#ff6b6b" : wl > 0.33 ? "#ffb454" : "#6ee7b7" }} />
            </span>{" "}
            {wl > 0.66 ? "High" : wl > 0.33 ? "Medium" : "Low"}
          </span>
          <button className={btn} aria-pressed={orbit} onClick={() => setOrbit(!orbit)}>Orbit mode {orbit ? "on" : "off"}</button>
          <button className={btn} aria-pressed={lowGlare} onClick={() => setLowGlare(!lowGlare)}>Low-glare {lowGlare ? "on" : "off"}</button>
        </div>
      </header>

      {minutes >= 60 && (
        <div role="status" className="mb-3 rounded-md border border-sky-400/40 bg-sky-400/10 px-3 py-2 text-sm">● You have been on console {minutes} min. Hand drones to auto survey and take a short break.</div>
      )}

      <div className="grid gap-3 lg:min-h-0 lg:flex-1 lg:grid-cols-[minmax(0,1fr)_340px] lg:grid-rows-[auto_auto_minmax(0,1fr)_auto_auto]">
        <div className="flex min-h-0 flex-col lg:col-start-1 lg:row-span-5 lg:row-start-1 lg:order-none order-2">
          <div className="relative" style={{ aspectRatio: `${W}/${H}`, width: "min(100%, calc((100dvh - 330px) * 1.6667))" }}>
            <canvas
              ref={canvas}
              width={W}
              height={H}
              onPointerDown={onDown}
              onPointerMove={onMove}
              onPointerUp={onUp}
              onPointerCancel={onUp}
              role="img"
              aria-label="Lunar surface map with six drones. Keyboard users: use the drone list, the Send to menu, plus and minus to zoom, Shift with arrows to pan."
              className={`block h-full w-full touch-none rounded-lg border ${view.blackout ? "border-amber-400 opacity-70" : "border-white/15"}`}
            />
            {pending && pd && (
              <div role="dialog" aria-label="Confirm destination" className="absolute inset-x-2 bottom-2 z-10 rounded-lg border border-white/40 bg-[#0b0d12]/95 p-2 backdrop-blur sm:inset-x-auto sm:left-1/2 sm:-translate-x-1/2 sm:w-[min(92%,560px)]">
                <p className="text-sm">
                  Send <b style={{ color: pd.d.color }}>{pd.d.name}</b> to <span className="font-mono">{pd.ll}</span>: {pdKm.toFixed(1)} km, about {(pdKm * 1000 / 25 * 0.02).toFixed(0)}% fuel. Arrives at the drone after {latency.toFixed(1)} s.
                </p>
                <div className="mt-2 flex flex-wrap gap-2">
                  <button className={`${btn} flex-1`} onClick={() => sendPending(false)}>Send</button>
                  <button className={`${btn} flex-1`} onClick={() => sendPending(true)}>Add waypoint</button>
                  <button className={`${btn} flex-1`} onClick={closePending}>Cancel</button>
                </div>
              </div>
            )}
          </div>
          <div className="mt-2 flex flex-wrap items-center gap-2" role="toolbar" aria-label="Map controls">
            <button className={mb} aria-label="Zoom out" onClick={() => zoomAt(0.5)}>−</button>
            <button className={mb} aria-label="Zoom in" onClick={() => zoomAt(2)}>+</button>
            <button className={mb} aria-label="Fit whole area" title="Fit whole area" onClick={fit}>⤢</button>
            <button className={`${mb} ${follow ? "!border-white !bg-white/30" : ""}`} aria-pressed={follow} aria-label="Follow selected drone" title="Follow selected drone" onClick={() => toggleFollow()}>◎</button>
            <span className="mx-1 h-8 w-px bg-white/20" aria-hidden />
            <div className="flex gap-2" role="group" aria-label="Pan map (hold to keep moving)">
              <button className={mb} aria-label="Pan left" {...hold(() => panBy(-st(), 0))}>◀</button>
              <button className={mb} aria-label="Pan up" {...hold(() => panBy(0, -st() * 0.6))}>▲</button>
              <button className={mb} aria-label="Pan down" {...hold(() => panBy(0, st() * 0.6))}>▼</button>
              <button className={mb} aria-label="Pan right" {...hold(() => panBy(st(), 0))}>▶</button>
            </div>
            <span className="ml-1 font-mono text-sm text-slate-300">{camInfo}</span>
          </div>
          <div className="mt-2 flex flex-wrap items-center gap-x-5 gap-y-1 text-sm text-slate-400">
            <span>Mapped <b className="text-slate-100">{(view.mapped * 100).toFixed(0)}%</b></span>
            <span>Targets <b className="text-slate-100">{view.found}/{view.targets}</b></span>
            <span>Station propellant <b className="text-slate-100">{view.reserve.toFixed(1)}</b> tanks</span>
                        <span className="text-slate-400">Dashed ring = command still in transit · red halo = poor position fix</span>
          </div>
          <p className="mt-1 text-xs text-slate-400">
            Terrain: NASA LRO Wide Angle Camera global mosaic (NASA/GSFC/Arizona State University, via NASA Moon Trek), ~83 m/px, Mare Tranquillitatis pit region, about 40 × 24 km. Lighter patches are areas your drones have surveyed; the pit location is real, the lander, beacons and survey sites are simulated.
          </p>
          <details className="mt-2 text-sm text-slate-400">
            <summary className="cursor-pointer">Controls &amp; shortcuts</summary>
            <p className="mt-1">Map: drag to move, pinch or scroll to zoom, or use the large buttons. Tap to place a target, then confirm. + / − zoom, 0 fit, C follow, Shift+arrows pan.</p>
            <p className="mt-1">1–6 select drone · click terrain to send it (Shift adds waypoints) · WASD / arrows fly the selected drone manually. All commands reach the drone after the link delay.</p>
          </details>
        </div>

          <section aria-label="Attention queue" className="order-1 lg:col-start-2 lg:row-start-1 rounded-lg border border-white/15 bg-white/[0.04]">
            <h2 className="flex items-center justify-between px-3 pt-2 text-sm font-semibold text-slate-300">
              Needs attention <span className="font-mono">{view.alerts.length}</span>
            </h2>
            <ul className="max-h-40 overflow-auto p-2" >
              {view.alerts.length === 0 && <li className="px-1 py-2 text-sm text-slate-400">✓ Nothing needs you. Swarm is nominal.</li>}
              {view.alerts.map((a) => (
                <li key={a.key}>
                  <button onClick={() => a.drone !== null && setSelected(a.drone)} className="flex min-h-11 w-full items-start gap-2 rounded px-2 py-1.5 text-left hover:bg-white/10">
                    <span aria-hidden style={{ color: SEV[a.sev].color }} className="mt-0.5">{SEV[a.sev].icon}</span>
                    <span>
                      <span className="sr-only">{a.sev}: </span>
                      <span className="block text-sm" style={{ color: a.sev === "advisory" ? undefined : SEV[a.sev].color }}>{a.text}</span>
                      <span className="block text-xs text-slate-400">→ {a.action}</span>
                    </span>
                  </button>
                </li>
              ))}
            </ul>
          </section>

          {view.proposals.length > 0 && (
            <section aria-label="Drone proposals" className="order-2 rounded-lg border border-sky-400/50 bg-sky-400/[0.07] p-2 lg:col-start-2 lg:row-start-2">
              <h2 className="text-sm font-semibold text-sky-200">Drone proposals · {view.proposals.length}</h2>
              <ul className="mt-1 space-y-2">
                {view.proposals.slice(0, 2).map((p) => (
                  <li key={p.pid} className="rounded border border-white/15 bg-black/20 p-2 text-sm">
                    <p>
                      <b style={{ color: p.color }}>{p.name}</b> proposes a {p.km.toFixed(1)} km leg (~{p.fuelEst.toFixed(0)}% fuel).
                    </p>
                    <p className="text-xs text-slate-300">{p.why}</p>
                    <p className="mt-1 text-xs text-slate-300">
                      {p.left === null ? "Waiting for your approval." : `Proceeds in ${p.left.toFixed(0)} s unless vetoed.`} Your reply takes {latency.toFixed(1)} s to reach it.
                    </p>
                    <div className="mt-1.5 flex gap-2">
                      <button className={btn} onClick={() => send(sim.current, p.drone, { type: "approve", pid: p.pid })}>Approve</button>
                      <button className={`${btn} !border-red-400/60`} onClick={() => send(sim.current, p.drone, { type: "veto", pid: p.pid })}>Veto</button>
                    </div>
                  </li>
                ))}
                {view.proposals.length > 2 && <li className="text-xs text-slate-300">+{view.proposals.length - 2} more waiting</li>}
              </ul>
            </section>
          )}

          <section aria-label="Drones" className="order-3 min-h-0 space-y-1.5 lg:col-start-2 lg:row-start-3 lg:overflow-y-auto lg:pr-1">
            <h2 className="sr-only">Drones</h2>
            {view.drones.map((d) => {
              const open = d.id === selected;
              return (
                <div key={d.id} className={`rounded-lg border ${open ? "border-white/70 bg-white/10" : "border-white/15 bg-white/[0.03]"}`}>
                  <button onClick={() => setSelected(d.id)} aria-pressed={open} className="min-h-11 w-full p-2 text-left">
                    <div className="flex items-center justify-between gap-2">
                      <span className="flex items-center gap-2 font-medium">
                        <span className="grid h-6 w-6 place-items-center rounded-full text-sm font-bold text-black" style={{ background: d.color }}>{d.id + 1}</span>
                        {d.name}
                      </span>
                      <span className="rounded bg-white/10 px-2 py-0.5 text-xs">{MODE[d.mode].icon} {MODE[d.mode].label}{d.pending ? ` · ${d.pending} sent` : ""}</span>
                    </div>
                    <div className="mt-1.5 flex items-center gap-2">
                      <span className="h-2 flex-1 overflow-hidden rounded bg-white/10">
                        <span className="block h-full" style={{ width: `${d.fuel}%`, background: d.fuel < 15 ? "#ff6b6b" : d.fuel < 30 ? "#ffb454" : "#6ee7b7" }} />
                      </span>
                      <span className="w-24 text-right font-mono text-xs">{d.fuel < 30 && "▲ "}fuel {d.fuel.toFixed(0)}%</span>
                    </div>
                  </button>
                  {open && (
                    <div className="border-t border-white/15 p-2 text-sm">
                      <dl className="space-y-1.5 text-sm">
                        <div><dt className="text-xs uppercase tracking-wide text-slate-400">Doing</dt><dd className="text-slate-100">{d.intent}</dd></div>
                        {d.why && <div><dt className="text-xs uppercase tracking-wide text-slate-400">Why</dt><dd className="text-slate-300">{d.why}</dd></div>}
                        <div>
                          <dt className="text-xs uppercase tracking-wide text-slate-400">Self-assessed confidence</dt>
                          <dd className="flex items-center gap-2">
                            <span className="h-2 w-16 overflow-hidden rounded bg-white/10"><span className="block h-full" style={{ width: `${d.conf * 100}%`, background: d.conf > 0.7 ? "#6ee7b7" : d.conf > 0.4 ? "#ffb454" : "#ff6b6b" }} /></span>
                            <span className="text-slate-100">{d.conf > 0.7 ? "High" : d.conf > 0.4 ? "Medium" : "Low"} {(d.conf * 100).toFixed(0)}%</span>
                          </dd>
                          <dd className="text-xs text-slate-400">{d.confWhy}</dd>
                        </div>
                        <div>
                          <dt className="text-xs uppercase tracking-wide text-slate-400">Track record</dt>
                          <dd className="text-xs text-slate-300">{d.stats.legs} legs done · {d.stats.approved} approved · {d.stats.vetoed} vetoed · {d.stats.overrides} overridden · {d.stats.recoveries} self-recoveries</dd>
                        </div>
                      </dl>
                      <p className="mt-1 font-mono text-xs text-slate-400">±{d.err.toFixed(0)} m · {d.links} links · {d.flights} hops</p>
                      <div className="mt-2 flex flex-wrap gap-2">
                        <button className={btn} onClick={() => cmd("auto")}>Auto survey</button>
                        <button className={btn} onClick={() => cmd("rtb")}>Return</button>
                        <button className={btn} onClick={() => cmd("stop")}>Hold</button>
                        <button className={btn} onClick={cancelPending} disabled={!d.pending}>Cancel sent</button>
                      </div>
                    </div>
                  )}
                </div>
              );
            })}
          </section>

          <section className="order-4 space-y-2 lg:col-start-2 lg:row-start-4 lg:max-h-[46vh] lg:overflow-y-auto">
            <h2 className="text-sm font-semibold text-slate-300">Whole swarm</h2>
            <div className="flex flex-wrap gap-2">
              <button className={btn} onClick={() => sim.current.drones.forEach((d) => cmd("auto", d.id))}>Survey all</button>
              <button className={`${btn} ${confirm === "recall" ? "!border-amber-400 !text-amber-300" : ""}`} onClick={guarded("recall", () => sim.current.drones.forEach((d) => cmd("rtb", d.id)))}>
                {confirm === "recall" ? "Press again to recall" : "Recall all"}
              </button>
              <button className={`${btn} ${confirm === "reset" ? "!border-amber-400 !text-amber-300" : ""}`} onClick={guarded("reset", () => { sim.current = createSim(latency, autonomy); setView(snap(sim.current)); })}>
                {confirm === "reset" ? "Press again to reset" : "Reset"}
              </button>
            </div>
            <label className="block text-sm text-slate-300">
              Send selected drone to
              <select
                className="mt-1 block min-h-10 w-full rounded-md border border-white/20 bg-[#10131a] px-2 text-sm"
                value=""
                onChange={(e) => {
                  const pl = places[Number(e.target.value)];
                  if (pl) send(sim.current, selected, { type: "goto", pts: [{ x: pl.p.x, y: pl.p.y }] });
                }}
              >
                <option value="">Choose a place…</option>
                {places.map((pl, i) => <option key={pl.label} value={i}>{pl.label}</option>)}
              </select>
            </label>
            <fieldset>
              <legend className="text-sm text-slate-300">Autonomy policy</legend>
              <div className="mt-1 grid grid-cols-3 gap-1.5" role="radiogroup">
                {([["ask", "Ask first"], ["veto", "Veto window"], ["full", "Full auto"]] as const).map(([v, label]) => (
                  <button key={v} role="radio" aria-checked={autonomy === v} onClick={() => setAutonomy(v)} className={`${btn} px-1 ${autonomy === v ? "!border-white/80 bg-white/20" : ""}`}>{label}</button>
                ))}
              </div>
              <p className="mt-1 text-xs text-slate-300">
                {autonomy === "ask" && "Unusual legs wait for approval. Safest, but stalls if the link drops."}
                {autonomy === "veto" && "Unusual legs start on their own after a window longer than the round trip, unless you veto."}
                {autonomy === "full" && "Drones decide every leg. You can still override any time."}
                {" "}Always on board, with no link needed: return at low fuel, recover a lost fix, never exceed the fuel margin.
              </p>
            </fieldset>
            <div className="flex flex-wrap items-center gap-2 text-sm text-slate-300">
              Operator link
              <button className={btn} onClick={() => setLatency(0.2)}>Lunar crew 0.2 s</button>
              <button className={btn} onClick={() => setLatency(1.3)}>Earth 1.3 s</button>
              <button className={`${btn} !border-amber-400/60`} onClick={() => startBlackout(sim.current, 30)}>Cut link 30 s</button>
            </div>
            <label className="block text-sm text-slate-300">
              Link delay: {latency.toFixed(1)} s one-way
              <input type="range" min={0} max={3} step={0.1} value={latency} onChange={(e) => setLatency(Number(e.target.value))} className="block h-8 w-full" />
            </label>
            <div className="flex items-center gap-2 text-sm text-slate-300">
              Sim speed
              {[1, 4, 16].map((v) => (
                <button key={v} aria-pressed={speed === v} onClick={() => setSpeed(v)} className={`${btn} min-w-12 ${speed === v ? "!border-white/80 bg-white/20" : ""}`}>{v}×</button>
              ))}
            </div>
          </section>

          <section aria-label="Event log" role="log" className="order-5 [@media(max-height:700px)]:hidden lg:col-start-2 lg:row-start-5 rounded-lg border border-white/15 bg-black/30 p-2 font-mono text-xs leading-relaxed text-slate-400">
            {view.log.slice(0, 6).map((l, i) => {
              const m = l.match(/^(T\+\d+s) \[(AUTO|CREW|SYS)\] (.*)$/);
              const tone = { AUTO: "text-sky-300", CREW: "text-emerald-300", SYS: "text-slate-300" };
              return m ? (
                <div key={i} className={i === 0 ? "text-slate-100" : ""}>
                  {m[1]} <b className={tone[m[2] as "AUTO"]}>{m[2] === "AUTO" ? "DRONE" : m[2]}</b> {m[3]}
                </div>
              ) : <div key={i}>{l}</div>;
            })}
          </section>
      </div>
    </div>
  );
}
