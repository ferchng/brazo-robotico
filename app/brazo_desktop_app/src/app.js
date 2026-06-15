const geom = {
  hBase: 100,
  wBase: 35,
  l1: 113,
  w1: 25,
  offsetM3: 27,
  l2: 137,
  w2: 25,
  l3: 60,
  w3: 25,
  toolBodyLen: 60,
  toolBodyWidth: 50,
  jawLen: 45,
  jawWidth: 10,
  jawHeight: 16,
  depth: 36
};

const motors = [
  { id: 1, label: "Motor 1", role: "direccion", min: 60, max: 300, initial: 180 },
  { id: 2, label: "Motor 2", role: "codo 1", min: 90, max: 270, initial: 180 },
  { id: 3, label: "Motor 3", role: "codo 2", min: 90, max: 270, initial: 90 },
  { id: 4, label: "Motor 4", role: "muñeca de pinza", min: 75, max: 260, initial: 180 },
  { id: 5, label: "Motor 5", role: "rotador de pinza", min: 0, max: 340, initial: 170 },
  { id: 6, label: "Motor 6", role: "pinza", min: 0, max: 140, initial: 70 }
];

const floorTouch = 5;
const floorRisk = 10;
const mirrorX = true;
const offsetSign = 1;
const moveCooldownMs = 1000;

const canvas = document.getElementById("viewer");
const ctx = canvas.getContext("2d");
const motorsPanel = document.getElementById("motorsPanel");
const statusBox = document.getElementById("statusBox");
const logBox = document.getElementById("logBox");
const connState = document.getElementById("connState");
const queueState = document.getElementById("queueState");
const poseMetric = document.getElementById("poseMetric");
const floorMetric = document.getElementById("floorMetric");
const m4Metric = document.getElementById("m4Metric");
const tcpMetric = document.getElementById("tcpMetric");
const portSelect = document.getElementById("portSelect");
const baudInput = document.getElementById("baudInput");
const commandInput = document.getElementById("commandInput");
const targetXInput = document.getElementById("targetXInput");
const targetYInput = document.getElementById("targetYInput");
const targetZInput = document.getElementById("targetZInput");
const targetState = document.getElementById("targetState");

const state = Object.fromEntries(motors.map(m => [m.id, m.initial]));
const lastValid = Object.fromEntries(motors.map(m => [m.id, m.initial]));
const sliders = {};
const labels = {};
const angleInputs = {};
const applyButtons = {};
const readButtons = {};
const motorBadges = {};

let connected = false;
let commandQueue = [];
let queueBusy = false;
let lastMoveSentAt = 0;
let lastTcp = { x: 0, y: 0, z: 0 };
let pendingTargetSolution = null;
let rejectedTargetPreview = null;

const camera = { yaw: 0.9, pitch: 0.55, distance: 520 };
let dragging = false;
let lastMouseX = 0;
let lastMouseY = 0;

function log(text) {
  logBox.value += `${text}\n`;
  logBox.scrollTop = logBox.scrollHeight;
}

function setStatus(text) {
  statusBox.innerHTML = text;
}

function setConnState(text, cls = "") {
  connState.className = `hint ${cls}`.trim();
  connState.textContent = text;
}

function updateQueueState() {
  queueState.textContent = `Cola: ${commandQueue.length}`;
}

function degToRad(d) { return d * Math.PI / 180; }
function add(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
function sub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
function scale(v, k) { return { x: v.x * k, y: v.y * k }; }
function vec(len, ang) { return { x: len * Math.cos(ang), y: len * Math.sin(ang) }; }
function dot(a, b) { return a.x * b.x + a.y * b.y; }

function pointSegmentDistance(p, a, b) {
  const ab = sub(b, a);
  const ap = sub(p, a);
  const abLen2 = dot(ab, ab);
  if (abLen2 <= 1e-9) return Math.hypot(p.x - a.x, p.y - a.y);
  let t = dot(ap, ab) / abLen2;
  t = Math.max(0, Math.min(1, t));
  const proj = add(a, scale(ab, t));
  return Math.hypot(p.x - proj.x, p.y - proj.y);
}

function orient(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function onSegment(a, b, p) {
  return Math.min(a.x, b.x) <= p.x && p.x <= Math.max(a.x, b.x) &&
         Math.min(a.y, b.y) <= p.y && p.y <= Math.max(a.y, b.y);
}

function segmentsIntersect(a, b, c, d) {
  const o1 = orient(a, b, c);
  const o2 = orient(a, b, d);
  const o3 = orient(c, d, a);
  const o4 = orient(c, d, b);
  if (Math.abs(o1) < 1e-6 && onSegment(a, b, c)) return true;
  if (Math.abs(o2) < 1e-6 && onSegment(a, b, d)) return true;
  if (Math.abs(o3) < 1e-6 && onSegment(c, d, a)) return true;
  if (Math.abs(o4) < 1e-6 && onSegment(c, d, b)) return true;
  return ((o1 > 0) !== (o2 > 0)) && ((o3 > 0) !== (o4 > 0));
}

function pointInRect(p, xMin, xMax, yMin, yMax) {
  return xMin <= p.x && p.x <= xMax && yMin <= p.y && p.y <= yMax;
}

function getDynamicM4Max(m2, m3) {
  let maxM4 = 260;
  const reasons = [];
  if (m2 <= 100 && m3 <= 110) { maxM4 = Math.min(maxM4, 130); reasons.push("M2<=100 y M3<=110"); }
  if (m2 <= 120 && m3 <= 100) { maxM4 = Math.min(maxM4, 120); reasons.push("M2<=120 y M3<=100"); }
  if (m2 >= 240 && m3 >= 140) { maxM4 = Math.min(maxM4, 200); reasons.push("M2>=240 y M3>=140"); }
  if (m2 >= 260) { maxM4 = Math.min(maxM4, 185); reasons.push("M2>=260"); }
  if (m3 >= 250) { maxM4 = Math.min(maxM4, 230); reasons.push("M3>=250"); }
  if (m2 >= 185 && m3 >= 245) { maxM4 = Math.min(maxM4, 250); reasons.push("M2>=185 y M3>=245"); }
  if (m2 >= 200 && m3 >= 255) { maxM4 = Math.min(maxM4, 245); reasons.push("M2>=200 y M3>=255"); }
  return { max: maxM4, reasons };
}

function perpendicularOffset(length, angleRad) {
  return {
    x: length * Math.cos(angleRad + offsetSign * Math.PI / 2),
    y: length * Math.sin(angleRad + offsetSign * Math.PI / 2)
  };
}

function computePosePoints() {
  const a1 = degToRad((state[2] - 180) + 90);
  const a2 = degToRad(state[3] - 90);
  const a3 = degToRad(state[4] - 180);

  const p0 = { x: 0, y: geom.hBase };
  const p1 = add(p0, vec(geom.l1, a1));
  const p1o = add(p1, perpendicularOffset(geom.offsetM3, a1));
  const p2 = add(p1o, vec(geom.l2, a1 + a2));
  const p3 = add(p2, vec(geom.l3, a1 + a2 + a3));

  if (!mirrorX) return { p0, p1, p1o, p2, p3 };
  return {
    p0: { x: -p0.x, y: p0.y },
    p1: { x: -p1.x, y: p1.y },
    p1o: { x: -p1o.x, y: p1o.y },
    p2: { x: -p2.x, y: p2.y },
    p3: { x: -p3.x, y: p3.y }
  };
}

function computeToolPoints2D() {
  const pts = computePosePoints();
  const dir = sub(pts.p3, pts.p2);
  const dirLen = Math.hypot(dir.x, dir.y) || 1;
  const u = { x: dir.x / dirLen, y: dir.y / dirLen };
  const v = { x: -u.y, y: u.x };
  const bodyCenter = add(pts.p3, scale(u, geom.toolBodyLen / 2));
  const jawBaseCenter = add(pts.p3, scale(u, geom.toolBodyLen));
  const gap = 6 + (state[6] / 140) * 26;
  return { ...pts, u, v, bodyCenter, jawBaseCenter, gap };
}

function computeGripperEnvelope() {
  const t = computeToolPoints2D();
  const halfW = Math.max(geom.toolBodyWidth / 2, t.gap / 2 + geom.jawWidth);
  const tipCenter = add(t.jawBaseCenter, scale(t.u, geom.jawLen));
  const g0 = add(t.bodyCenter, add(scale(t.u, -geom.toolBodyLen / 2), scale(t.v, -halfW)));
  const g1 = add(t.bodyCenter, add(scale(t.u, -geom.toolBodyLen / 2), scale(t.v, halfW)));
  const g2 = add(tipCenter, scale(t.v, halfW));
  const g3 = add(tipCenter, scale(t.v, -halfW));
  return { ...t, g0, g1, g2, g3 };
}

function collidesWithBase(corners) {
  const xMin = -geom.wBase / 2;
  const xMax = geom.wBase / 2;
  const yMin = 0;
  const yMax = geom.hBase;
  for (const c of corners) {
    if (pointInRect(c, xMin, xMax, yMin, yMax)) return true;
  }
  const rectA = [
    { x: xMin, y: yMin }, { x: xMax, y: yMin }, { x: xMax, y: yMax }, { x: xMin, y: yMax }
  ];
  const rectB = [
    { x: xMax, y: yMin }, { x: xMax, y: yMax }, { x: xMin, y: yMax }, { x: xMin, y: yMin }
  ];
  for (let i = 0; i < 4; i++) {
    const a = corners[i];
    const b = corners[(i + 1) % 4];
    for (let j = 0; j < 4; j++) {
      if (segmentsIntersect(a, b, rectA[j], rectB[j])) return true;
    }
  }
  return false;
}

function collidesWithLink(corners, a, b, thickness) {
  const radius = thickness / 2;
  for (const c of corners) {
    if (pointSegmentDistance(c, a, b) <= radius) return true;
  }
  for (let i = 0; i < 4; i++) {
    const e0 = corners[i];
    const e1 = corners[(i + 1) % 4];
    if (segmentsIntersect(e0, e1, a, b)) return true;
    if (pointSegmentDistance(e0, a, b) <= radius) return true;
    if (pointSegmentDistance(e1, a, b) <= radius) return true;
  }
  return false;
}

function classifyPose() {
  const pts = computeGripperEnvelope();
  const corners = [pts.g0, pts.g1, pts.g2, pts.g3];
  const minY = Math.min(...corners.map(c => c.y));
  let reason = "OK";
  let invalid = false;
  if (minY <= floorTouch) {
    reason = "PISO";
    invalid = true;
  } else if (pointInRect(pts.bodyCenter, -geom.wBase / 2, geom.wBase / 2, 0, geom.hBase)) {
    reason = "COLISION_BASE";
    invalid = true;
  } else if (collidesWithBase(corners)) {
    reason = "COLISION_BASE";
    invalid = true;
  } else if (collidesWithLink(corners, pts.p0, pts.p1, geom.w1)) {
    reason = "COLISION_L1";
    invalid = true;
  } else if (collidesWithLink(corners, pts.p1o, pts.p2, geom.w2)) {
    reason = "COLISION_L2";
    invalid = true;
  } else if (minY <= floorRisk) {
    reason = "RIESGO_PISO";
    invalid = true;
  }
  return { invalid, reason, minY, pts };
}

function vec3(x, y, z) { return { x, y, z }; }
function add3(a, b) { return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }; }
function sub3(a, b) { return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }; }
function scale3(v, k) { return { x: v.x * k, y: v.y * k, z: v.z * k }; }
function dot3(a, b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
function cross3(a, b) {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
  };
}
function len3(v) { return Math.hypot(v.x, v.y, v.z); }
function norm3(v) {
  const l = len3(v) || 1;
  return { x: v.x / l, y: v.y / l, z: v.z / l };
}

function rotateAroundAxis(v, axis, ang) {
  const u = norm3(axis);
  const c = Math.cos(ang);
  const s = Math.sin(ang);
  const term1 = scale3(v, c);
  const term2 = scale3(cross3(u, v), s);
  const term3 = scale3(u, dot3(u, v) * (1 - c));
  return add3(add3(term1, term2), term3);
}

function yawTo3D(p2) {
  const yaw = degToRad(state[1] - 180);
  return {
    x: p2.x * Math.cos(yaw),
    y: p2.y,
    z: -p2.x * Math.sin(yaw)
  };
}

function liftVec2To3(v2) {
  const yaw = degToRad(state[1] - 180);
  return norm3({
    x: v2.x * Math.cos(yaw),
    y: v2.y,
    z: -v2.x * Math.sin(yaw)
  });
}

function planeNormal3() {
  const yaw = degToRad(state[1] - 180);
  return norm3({ x: -Math.sin(yaw), y: 0, z: -Math.cos(yaw) });
}

function yawTo3DForAngle(p2, m1Deg) {
  const yaw = degToRad(m1Deg - 180);
  return {
    x: p2.x * Math.cos(yaw),
    y: p2.y,
    z: -p2.x * Math.sin(yaw)
  };
}

function liftVec2To3ForAngle(v2, m1Deg) {
  const yaw = degToRad(m1Deg - 180);
  return norm3({
    x: v2.x * Math.cos(yaw),
    y: v2.y,
    z: -v2.x * Math.sin(yaw)
  });
}

function planeNormal3ForAngle(m1Deg) {
  const yaw = degToRad(m1Deg - 180);
  return norm3({ x: -Math.sin(yaw), y: 0, z: -Math.cos(yaw) });
}

function rotateYCamera(p, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  return { x: p.x * c + p.z * s, y: p.y, z: -p.x * s + p.z * c };
}

function rotateXCamera(p, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  return { x: p.x, y: p.y * c - p.z * s, z: p.y * s + p.z * c };
}

function projectPoint(p) {
  let v = rotateYCamera(p, camera.yaw);
  v = rotateXCamera(v, camera.pitch);
  const z = v.z + camera.distance;
  const f = 540 / Math.max(60, z);
  return {
    x: canvas.width / 2 + v.x * f,
    y: canvas.height * 0.78 - v.y * f,
    z
  };
}

function boxVerticesFromBasis(center, u, v, w, su, sv, sw) {
  const hu = su / 2;
  const hv = sv / 2;
  const hw = sw / 2;
  return [
    add3(add3(add3(center, scale3(u, -hu)), scale3(v, -hv)), scale3(w, -hw)),
    add3(add3(add3(center, scale3(u, hu)), scale3(v, -hv)), scale3(w, -hw)),
    add3(add3(add3(center, scale3(u, hu)), scale3(v, hv)), scale3(w, -hw)),
    add3(add3(add3(center, scale3(u, -hu)), scale3(v, hv)), scale3(w, -hw)),
    add3(add3(add3(center, scale3(u, -hu)), scale3(v, -hv)), scale3(w, hw)),
    add3(add3(add3(center, scale3(u, hu)), scale3(v, -hv)), scale3(w, hw)),
    add3(add3(add3(center, scale3(u, hu)), scale3(v, hv)), scale3(w, hw)),
    add3(add3(add3(center, scale3(u, -hu)), scale3(v, hv)), scale3(w, hw))
  ];
}

function drawBox(vertices, fill, stroke) {
  const faces = [
    [0,1,2,3], [4,5,6,7], [0,1,5,4],
    [1,2,6,5], [2,3,7,6], [3,0,4,7]
  ];
  const projected = vertices.map(projectPoint);
  const ordered = faces.map(face => ({
    face,
    depth: face.reduce((acc, i) => acc + projected[i].z, 0) / face.length
  })).sort((a, b) => b.depth - a.depth);

  for (const item of ordered) {
    ctx.beginPath();
    const first = projected[item.face[0]];
    ctx.moveTo(first.x, first.y);
    for (let i = 1; i < item.face.length; i++) {
      const p = projected[item.face[i]];
      ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.3;
    ctx.fill();
    ctx.stroke();
  }
}

function drawFloorPlane(y, color) {
  const a = projectPoint(vec3(-320, y, -140));
  const b = projectPoint(vec3(320, y, -140));
  const c = projectPoint(vec3(320, y, 140));
  const d = projectPoint(vec3(-320, y, 140));
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.lineTo(c.x, c.y);
  ctx.lineTo(d.x, d.y);
  ctx.closePath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  ctx.setLineDash([6, 6]);
  ctx.stroke();
  ctx.setLineDash([]);
}

function segmentBox3ForAngle(a2, b2, m1Deg, thickness, depth) {
  const a = yawTo3DForAngle(a2, m1Deg);
  const b = yawTo3DForAngle(b2, m1Deg);
  const u = norm3(sub3(b, a));
  const w = planeNormal3ForAngle(m1Deg);
  const v = norm3(cross3(w, u));
  const center = scale3(add3(a, b), 0.5);
  return boxVerticesFromBasis(center, u, v, w, len3(sub3(b, a)), thickness, depth);
}

function drawGhostPose(candidate, theme = "blue") {
  const palette = theme === "red"
    ? {
        linkFill: "rgba(239,68,68,0.18)",
        linkFillSoft: "rgba(248,113,113,0.14)",
        linkStroke: "#f87171",
        linkStrokeSoft: "#fca5a5",
        toolFill: "rgba(239,68,68,0.22)",
        toolStroke: "#f87171",
        jawFill: "rgba(239,68,68,0.24)",
        jawStroke: "#fca5a5"
      }
    : {
        linkFill: "rgba(59,130,246,0.18)",
        linkFillSoft: "rgba(59,130,246,0.14)",
        linkStroke: "#60a5fa",
        linkStrokeSoft: "#93c5fd",
        toolFill: "rgba(59,130,246,0.22)",
        toolStroke: "#60a5fa",
        jawFill: "rgba(59,130,246,0.24)",
        jawStroke: "#93c5fd"
      };

  const pose2d = (() => {
    const backup = { ...state };
    state[1] = candidate.m1;
    state[2] = candidate.m2;
    state[3] = candidate.m3;
    state[4] = candidate.m4;
    state[5] = candidate.m5;
    state[6] = candidate.m6;
    const result = computeToolPoints2D();
    state[1] = backup[1];
    state[2] = backup[2];
    state[3] = backup[3];
    state[4] = backup[4];
    state[5] = backup[5];
    state[6] = backup[6];
    return result;
  })();

  drawBox(segmentBox3ForAngle(pose2d.p0, pose2d.p1, candidate.m1, geom.w1, geom.depth), palette.linkFill, palette.linkStroke);
  drawBox(segmentBox3ForAngle(pose2d.p1, pose2d.p1o, candidate.m1, 12, geom.depth * 0.8), palette.linkFillSoft, palette.linkStrokeSoft);
  drawBox(segmentBox3ForAngle(pose2d.p1o, pose2d.p2, candidate.m1, geom.w2, geom.depth), palette.linkFill, palette.linkStroke);
  drawBox(segmentBox3ForAngle(pose2d.p2, pose2d.p3, candidate.m1, geom.w3, geom.depth), palette.linkFill, palette.linkStroke);

  const u3 = liftVec2To3ForAngle(pose2d.u, candidate.m1);
  let v3 = liftVec2To3ForAngle(pose2d.v, candidate.m1);
  let w3 = planeNormal3ForAngle(candidate.m1);
  const roll = degToRad(candidate.m5 - 170);
  v3 = rotateAroundAxis(v3, u3, roll);
  w3 = rotateAroundAxis(w3, u3, roll);

  const bodyCenter3 = yawTo3DForAngle(pose2d.bodyCenter, candidate.m1);
  const jawBase3 = yawTo3DForAngle(pose2d.jawBaseCenter, candidate.m1);
  const jawGap = pose2d.gap;

  drawBox(
    boxVerticesFromBasis(bodyCenter3, u3, v3, w3, geom.toolBodyLen, geom.toolBodyWidth, geom.depth * 0.75),
    palette.toolFill,
    palette.toolStroke
  );

  const upperJawCenter = add3(add3(jawBase3, scale3(u3, geom.jawLen / 2)), scale3(v3, jawGap / 2 + geom.jawWidth / 2));
  const lowerJawCenter = add3(add3(jawBase3, scale3(u3, geom.jawLen / 2)), scale3(v3, -(jawGap / 2 + geom.jawWidth / 2)));

  drawBox(
    boxVerticesFromBasis(upperJawCenter, u3, v3, w3, geom.jawLen, geom.jawWidth, geom.jawHeight),
    palette.jawFill,
    palette.jawStroke
  );
  drawBox(
    boxVerticesFromBasis(lowerJawCenter, u3, v3, w3, geom.jawLen, geom.jawWidth, geom.jawHeight),
    palette.jawFill,
    palette.jawStroke
  );
}

function drawWorldAxes() {
  const origin = projectPoint(vec3(0, 0, 0));
  const xEnd = projectPoint(vec3(120, 0, 0));
  const yEnd = projectPoint(vec3(0, 120, 0));
  const zEnd = projectPoint(vec3(0, 0, 120));

  function axisLine(end, color, label) {
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(end.x, end.y);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(end.x, end.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();

    ctx.fillStyle = color;
    ctx.font = "bold 14px Segoe UI";
    ctx.fillText(label, end.x + 8, end.y - 8);
  }

  axisLine(xEnd, "#ef4444", "X");
  axisLine(yEnd, "#22c55e", "Y");
  axisLine(zEnd, "#38bdf8", "Z");

  ctx.beginPath();
  ctx.arc(origin.x, origin.y, 4, 0, Math.PI * 2);
  ctx.fillStyle = "#e5e7eb";
  ctx.fill();
}

function segmentBox3(a2, b2, thickness, depth) {
  const a = yawTo3D(a2);
  const b = yawTo3D(b2);
  const u = norm3(sub3(b, a));
  const w = planeNormal3();
  const v = norm3(cross3(w, u));
  const center = scale3(add3(a, b), 0.5);
  return boxVerticesFromBasis(center, u, v, w, len3(sub3(b, a)), thickness, depth);
}

function drawScene() {
  const sim = classifyPose();
  const pts = sim.pts;
  const { max, reasons } = getDynamicM4Max(state[2], state[3]);
  const fk = window.BrazoKinematics
    ? window.BrazoKinematics.forwardKinematics({
        m1: state[1],
        m2: state[2],
        m3: state[3],
        m4: state[4],
        m5: state[5],
        m6: state[6]
      })
    : null;

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#050914";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  drawFloorPlane(0, "#94a3b8");
  drawFloorPlane(floorTouch, "#ef4444");
  drawFloorPlane(floorRisk, "#f59e0b");
  drawWorldAxes();

  const baseCenter = vec3(0, geom.hBase / 2, 0);
  drawBox(
    boxVerticesFromBasis(baseCenter, vec3(1,0,0), vec3(0,1,0), vec3(0,0,1), geom.wBase, geom.hBase, 90),
    "rgba(71,85,105,0.78)",
    "#cbd5e1"
  );
  drawBox(segmentBox3(pts.p0, pts.p1, geom.w1, geom.depth), "rgba(34,197,94,0.78)", "#86efac");
  drawBox(segmentBox3(pts.p1, pts.p1o, 12, geom.depth * 0.8), "rgba(245,158,11,0.78)", "#fcd34d");
  drawBox(segmentBox3(pts.p1o, pts.p2, geom.w2, geom.depth), "rgba(56,189,248,0.78)", "#7dd3fc");

  const link3Fill = sim.reason === "RIESGO_PISO" ? "rgba(245,158,11,0.78)" : (sim.invalid ? "rgba(239,68,68,0.78)" : "rgba(244,63,94,0.78)");
  const link3Stroke = sim.reason === "RIESGO_PISO" ? "#fcd34d" : (sim.invalid ? "#fca5a5" : "#fda4af");
  drawBox(segmentBox3(pts.p2, pts.p3, geom.w3, geom.depth), link3Fill, link3Stroke);

  const u3 = liftVec2To3(pts.u);
  let v3 = liftVec2To3(pts.v);
  let w3 = planeNormal3();
  const roll = degToRad(state[5] - 170);
  v3 = rotateAroundAxis(v3, u3, roll);
  w3 = rotateAroundAxis(w3, u3, roll);

  const bodyCenter3 = yawTo3D(pts.bodyCenter);
  drawBox(
    boxVerticesFromBasis(bodyCenter3, u3, v3, w3, geom.toolBodyLen, geom.toolBodyWidth, geom.depth * 0.75),
    sim.invalid ? "rgba(239,68,68,0.45)" : "rgba(167,139,250,0.42)",
    "#c4b5fd"
  );

  const jawGap = 6 + (state[6] / 140) * 26;
  const jawBase3 = yawTo3D(pts.jawBaseCenter);
  const upperJawCenter = add3(add3(jawBase3, scale3(u3, geom.jawLen / 2)), scale3(v3, jawGap / 2 + geom.jawWidth / 2));
  const lowerJawCenter = add3(add3(jawBase3, scale3(u3, geom.jawLen / 2)), scale3(v3, -(jawGap / 2 + geom.jawWidth / 2)));
  drawBox(boxVerticesFromBasis(upperJawCenter, u3, v3, w3, geom.jawLen, geom.jawWidth, geom.jawHeight), "rgba(250,204,21,0.78)", "#fde68a");
  drawBox(boxVerticesFromBasis(lowerJawCenter, u3, v3, w3, geom.jawLen, geom.jawWidth, geom.jawHeight), "rgba(250,204,21,0.78)", "#fde68a");

  if (pendingTargetSolution && pendingTargetSolution.candidate) {
    drawGhostPose(pendingTargetSolution.candidate, "blue");
  } else if (rejectedTargetPreview && rejectedTargetPreview.candidate) {
    drawGhostPose(rejectedTargetPreview.candidate, "red");
  }

  poseMetric.textContent = sim.invalid ? sim.reason : "PERMITIDA";
  poseMetric.className = sim.invalid ? "bad" : "ok";
  floorMetric.textContent = `${sim.minY.toFixed(1)} mm`;
  floorMetric.className = sim.reason === "PISO" ? "bad" : (sim.reason === "RIESGO_PISO" ? "warn" : "ok");
  m4Metric.textContent = `${max}°`;
  if (fk) {
    const tcp = fk.points.tcp;
    lastTcp = tcp;
    tcpMetric.textContent = `x ${tcp.x.toFixed(0)} | y ${tcp.y.toFixed(0)} | z ${tcp.z.toFixed(0)}`;
  } else {
    tcpMetric.textContent = "sin kinematics";
  }

  if (motorBadges[4]) {
    motorBadges[4].textContent = `M4 max actual: ${max}°`;
  }

  setStatus(
    `Estado pose: <span class="${sim.invalid ? "bad" : "ok"}">${sim.invalid ? sim.reason : "PERMITIDA"}</span>\n` +
    `Clearance piso: <span class="${sim.reason === "PISO" ? "bad" : (sim.reason === "RIESGO_PISO" ? "warn" : "ok")}">${sim.minY.toFixed(1)} mm</span>\n` +
    `M1=${state[1]}° | M5=${state[5]}° | M6=${state[6]}°\n` +
    `Reglas activas: ${reasons.length ? reasons.join(" | ") : "Sin regla dinámica activa."}`
  );
}

function setTargetState(text, cls = "") {
  targetState.className = `hint ${cls}`.trim();
  targetState.textContent = text;
}

function clearPendingTarget() {
  pendingTargetSolution = null;
  rejectedTargetPreview = null;
}

function syncLabels() {
  for (const id of Object.keys(labels)) {
    labels[id].textContent = `${state[id]}°`;
    if (angleInputs[id]) {
      angleInputs[id].value = state[id];
    }
    if (sliders[id]) {
      sliders[id].value = state[id];
    }
  }
}

function logicalDegreesFromRaw(id, rawDeg) {
  let deg = rawDeg;
  if (id === 6) deg -= 10;
  return Math.round(deg);
}

function classifyPoseForAngles(candidate) {
  const backup = { ...state };
  state[1] = candidate.m1;
  state[2] = candidate.m2;
  state[3] = candidate.m3;
  state[4] = candidate.m4;
  state[5] = candidate.m5 ?? state[5];
  state[6] = candidate.m6 ?? state[6];
  const result = classifyPose();
  const dynamic = getDynamicM4Max(state[2], state[3]);
  const invalidByDynamic = state[4] > dynamic.max;
  state[1] = backup[1];
  state[2] = backup[2];
  state[3] = backup[3];
  state[4] = backup[4];
  state[5] = backup[5];
  state[6] = backup[6];
  return {
    invalid: result.invalid || invalidByDynamic,
    reason: invalidByDynamic ? "REGLA_M4" : result.reason
  };
}

function isPoseSafe(candidate) {
  return !classifyPoseForAngles(candidate).invalid;
}

function distance3(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return Math.hypot(dx, dy, dz);
}

function solveTargetXYZ(target, toolOverride = null) {
  if (!window.BrazoKinematics) {
    return { ok: false, reason: "Kinematics no cargado." };
  }

  const currentTool = toolOverride || { m5: state[5], m6: state[6] };
  let best = null;

  function searchRange(range, step, seed) {
    let localBest = seed;
    for (let m1 = range.m1[0]; m1 <= range.m1[1]; m1 += step.m1) {
      for (let m2 = range.m2[0]; m2 <= range.m2[1]; m2 += step.m2) {
        for (let m3 = range.m3[0]; m3 <= range.m3[1]; m3 += step.m3) {
          const dynamic = getDynamicM4Max(m2, m3);
          const m4Max = Math.min(260, dynamic.max, range.m4[1]);
          for (let m4 = range.m4[0]; m4 <= m4Max; m4 += step.m4) {
            const candidate = { m1, m2, m3, m4, ...currentTool };
            const safe = classifyPoseForAngles(candidate);
            if (safe.invalid) continue;
            const fk = window.BrazoKinematics.forwardKinematics(candidate);
            const dist = distance3(fk.points.tcp, target);
            if (!localBest || dist < localBest.dist) {
              localBest = { candidate, dist, tcp: fk.points.tcp };
            }
          }
        }
      }
    }
    return localBest;
  }

  best = searchRange(
    {
      m1: [60, 300],
      m2: [90, 270],
      m3: [90, 270],
      m4: [75, 260]
    },
    {
      m1: 12,
      m2: 12,
      m3: 12,
      m4: 12
    },
    best
  );

  if (!best) {
    return { ok: false, reason: "No existe ninguna pose segura para ese objetivo.", nearest: null, tcp: null, dist: null };
  }

  let refined = searchRange(
    {
      m1: [Math.max(60, best.candidate.m1 - 18), Math.min(300, best.candidate.m1 + 18)],
      m2: [Math.max(90, best.candidate.m2 - 18), Math.min(270, best.candidate.m2 + 18)],
      m3: [Math.max(90, best.candidate.m3 - 18), Math.min(270, best.candidate.m3 + 18)],
      m4: [Math.max(75, best.candidate.m4 - 18), Math.min(260, best.candidate.m4 + 18)]
    },
    {
      m1: 4,
      m2: 4,
      m3: 4,
      m4: 4
    },
    best
  );

  refined = searchRange(
    {
      m1: [Math.max(60, refined.candidate.m1 - 6), Math.min(300, refined.candidate.m1 + 6)],
      m2: [Math.max(90, refined.candidate.m2 - 6), Math.min(270, refined.candidate.m2 + 6)],
      m3: [Math.max(90, refined.candidate.m3 - 6), Math.min(270, refined.candidate.m3 + 6)],
      m4: [Math.max(75, refined.candidate.m4 - 6), Math.min(260, refined.candidate.m4 + 6)]
    },
    {
      m1: 1,
      m2: 1,
      m3: 1,
      m4: 1
    },
    refined
  );

  if (refined.dist > 28) {
    return {
      ok: false,
      reason: `No hay solución segura suficientemente cercana. Error mínimo: ${refined.dist.toFixed(1)} mm.`
    };
  }

  return { ok: true, ...refined };
}

function findNearestSafePose(target, toolOverride = null) {
  if (!window.BrazoKinematics) {
    return null;
  }

  const currentTool = toolOverride || { m5: state[5], m6: state[6] };
  let best = null;

  function probe(range, step, seed) {
    let localBest = seed;
    for (let m1 = range.m1[0]; m1 <= range.m1[1]; m1 += step.m1) {
      for (let m2 = range.m2[0]; m2 <= range.m2[1]; m2 += step.m2) {
        for (let m3 = range.m3[0]; m3 <= range.m3[1]; m3 += step.m3) {
          const dynamic = getDynamicM4Max(m2, m3);
          const m4Max = Math.min(260, dynamic.max, range.m4[1]);
          for (let m4 = range.m4[0]; m4 <= m4Max; m4 += step.m4) {
            const candidate = { m1, m2, m3, m4, ...currentTool };
            if (!isPoseSafe(candidate)) continue;
            const fk = window.BrazoKinematics.forwardKinematics(candidate);
            const dist = distance3(fk.points.tcp, target);
            if (!localBest || dist < localBest.dist) {
              localBest = { candidate, tcp: fk.points.tcp, dist };
            }
          }
        }
      }
    }
    return localBest;
  }

  best = probe(
    { m1: [60, 300], m2: [90, 270], m3: [90, 270], m4: [75, 260] },
    { m1: 12, m2: 12, m3: 12, m4: 12 },
    best
  );

  if (!best) return null;

  best = probe(
    {
      m1: [Math.max(60, best.candidate.m1 - 18), Math.min(300, best.candidate.m1 + 18)],
      m2: [Math.max(90, best.candidate.m2 - 18), Math.min(270, best.candidate.m2 + 18)],
      m3: [Math.max(90, best.candidate.m3 - 18), Math.min(270, best.candidate.m3 + 18)],
      m4: [Math.max(75, best.candidate.m4 - 18), Math.min(260, best.candidate.m4 + 18)]
    },
    { m1: 4, m2: 4, m3: 4, m4: 4 },
    best
  );

  best = probe(
    {
      m1: [Math.max(60, best.candidate.m1 - 6), Math.min(300, best.candidate.m1 + 6)],
      m2: [Math.max(90, best.candidate.m2 - 6), Math.min(270, best.candidate.m2 + 6)],
      m3: [Math.max(90, best.candidate.m3 - 6), Math.min(270, best.candidate.m3 + 6)],
      m4: [Math.max(75, best.candidate.m4 - 6), Math.min(260, best.candidate.m4 + 6)]
    },
    { m1: 1, m2: 1, m3: 1, m4: 1 },
    best
  );

  return best;
}

function fkForPose(pose) {
  return window.BrazoKinematics.forwardKinematics({
    m1: pose.m1,
    m2: pose.m2,
    m3: pose.m3,
    m4: pose.m4,
    m5: pose.m5,
    m6: pose.m6
  });
}

function applyPoseToUi(candidate) {
  state[1] = candidate.m1;
  state[2] = candidate.m2;
  state[3] = candidate.m3;
  state[4] = candidate.m4;
  state[5] = candidate.m5;
  state[6] = candidate.m6;

  Object.keys(sliders).forEach(id => {
    sliders[id].value = state[id];
  });

  syncLabels();
  drawScene();
}

function appendMove(id, angle) {
  if (!connected) return;
  commandQueue.push({ id, angle });
  updateQueueState();
}

function enqueuePoseSequence(sequence, basePose = null) {
  if (!connected) return;
  let previous = basePose ? { ...basePose } : {
    m1: state[1],
    m2: state[2],
    m3: state[3],
    m4: state[4],
    m5: state[5],
    m6: state[6]
  };
  for (const pose of sequence) {
    if (pose.m1 !== previous.m1) appendMove(1, pose.m1);
    if (pose.m2 !== previous.m2) appendMove(2, pose.m2);
    if (pose.m3 !== previous.m3) appendMove(3, pose.m3);
    if (pose.m4 !== previous.m4) appendMove(4, pose.m4);
    if (pose.m5 !== previous.m5) appendMove(5, pose.m5);
    if (pose.m6 !== previous.m6) appendMove(6, pose.m6);
    previous = { ...pose };
  }
  processQueue();
}

function findTransitPose(toolValues) {
  const candidates = [
    { m1: state[1], m2: 180, m3: 140, m4: 180, m5: toolValues.m5, m6: toolValues.m6 },
    { m1: state[1], m2: 180, m3: 150, m4: 170, m5: toolValues.m5, m6: toolValues.m6 },
    { m1: state[1], m2: 170, m3: 140, m4: 170, m5: toolValues.m5, m6: toolValues.m6 },
    { m1: state[1], m2: 190, m3: 140, m4: 170, m5: toolValues.m5, m6: toolValues.m6 },
    { m1: state[1], m2: 180, m3: 160, m4: 180, m5: toolValues.m5, m6: toolValues.m6 }
  ];
  for (const candidate of candidates) {
    if (isPoseSafe(candidate)) return candidate;
  }
  return null;
}

function buildPoseTransition(startPose, endPose, order) {
  const current = { ...startPose };
  const sequence = [];
  for (const key of order) {
    if (current[key] === endPose[key]) continue;
    const next = { ...current, [key]: endPose[key] };
    if (!isPoseSafe(next)) {
      return null;
    }
    sequence.push({ ...next });
    Object.assign(current, next);
  }
  return sequence;
}

function buildInterpolatedPath(startPose, endPose, maxStepDeg = 4) {
  const delta = {
    m1: endPose.m1 - startPose.m1,
    m2: endPose.m2 - startPose.m2,
    m3: endPose.m3 - startPose.m3,
    m4: endPose.m4 - startPose.m4,
    m5: endPose.m5 - startPose.m5,
    m6: endPose.m6 - startPose.m6
  };

  const maxDelta = Math.max(
    Math.abs(delta.m1),
    Math.abs(delta.m2),
    Math.abs(delta.m3),
    Math.abs(delta.m4),
    Math.abs(delta.m5),
    Math.abs(delta.m6)
  );

  const steps = Math.max(1, Math.ceil(maxDelta / maxStepDeg));
  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    const pose = {
      m1: Math.round(startPose.m1 + delta.m1 * t),
      m2: Math.round(startPose.m2 + delta.m2 * t),
      m3: Math.round(startPose.m3 + delta.m3 * t),
      m4: Math.round(startPose.m4 + delta.m4 * t),
      m5: Math.round(startPose.m5 + delta.m5 * t),
      m6: Math.round(startPose.m6 + delta.m6 * t)
    };
    if (!isPoseSafe(pose)) {
      return null;
    }
  }
  return [{ ...endPose }];
}

function planSafeSequence(startPose, targetPose) {
  const directInterpolated = buildInterpolatedPath(startPose, targetPose, 4);
  if (directInterpolated) {
    return { ok: true, sequence: directInterpolated, strategy: "interpolada" };
  }

  const direct = buildPoseTransition(startPose, targetPose, ["m1", "m2", "m3", "m4", "m5", "m6"]);
  if (direct) {
    return { ok: true, sequence: direct, strategy: "directa" };
  }

  const transit = findTransitPose({ m5: targetPose.m5, m6: targetPose.m6 });
  if (!transit) {
    return { ok: false, reason: "No encontré una pose de tránsito segura." };
  }

  const toTransit = buildPoseTransition(startPose, transit, ["m4", "m3", "m2", "m1", "m5", "m6"]);
  if (!toTransit) {
    return { ok: false, reason: "No pude construir un tránsito seguro desde la pose actual." };
  }

  const fromTransitInterpolated = buildInterpolatedPath(transit, targetPose, 4);
  if (fromTransitInterpolated) {
    return {
      ok: true,
      sequence: [...toTransit, ...fromTransitInterpolated],
      strategy: "transito+interpolada"
    };
  }

  const fromTransit = buildPoseTransition(transit, targetPose, ["m1", "m2", "m3", "m4", "m5", "m6"]);
  if (!fromTransit) {
    return { ok: false, reason: "No pude construir un acercamiento seguro al objetivo final." };
  }

  return {
    ok: true,
    sequence: [...toTransit, ...fromTransit],
    strategy: "transito"
  };
}

function planSegmentToTarget(startPose, target, label) {
  const solved = solveTargetXYZ(target, { m5: startPose.m5, m6: startPose.m6 });
  if (!solved.ok) {
    return { ok: false, reason: `${label}: ${solved.reason}` };
  }
  const plan = planSafeSequence(startPose, solved.candidate);
  if (!plan.ok) {
    return { ok: false, reason: `${label}: ${plan.reason}` };
  }
  return {
    ok: true,
    endPose: solved.candidate,
    endTcp: solved.tcp,
    sequence: plan.sequence,
    dist: solved.dist,
    strategy: plan.strategy
  };
}

function planCartesianTarget(startPose, target) {
  const startFk = fkForPose(startPose);
  let currentPose = { ...startPose };
  let currentTcp = { ...startFk.points.tcp };
  const fullSequence = [];
  const notes = [];
  const blockedAxes = [];

  for (const axis of ["x", "y", "z"]) {
    if (Math.abs(target[axis] - currentTcp[axis]) < 1) {
      continue;
    }

    const waypoint = { ...currentTcp, [axis]: target[axis] };
    const segment = planSegmentToTarget(currentPose, waypoint, `Eje ${axis.toUpperCase()}`);
    if (segment.ok) {
      fullSequence.push(...segment.sequence);
      currentPose = { ...segment.endPose };
      currentTcp = { ...segment.endTcp };
      notes.push(`${axis.toUpperCase()} directo`);
    } else {
      blockedAxes.push(axis);
      notes.push(`${axis.toUpperCase()} bloqueado`);
    }
  }

  for (const axis of blockedAxes) {
    if (Math.abs(target[axis] - currentTcp[axis]) < 1) {
      continue;
    }

    const directRetry = planSegmentToTarget(currentPose, { ...currentTcp, [axis]: target[axis] }, `Reintento ${axis.toUpperCase()}`);
    if (directRetry.ok) {
      fullSequence.push(...directRetry.sequence);
      currentPose = { ...directRetry.endPose };
      currentTcp = { ...directRetry.endTcp };
      notes.push(`${axis.toUpperCase()} directo tras otros ejes`);
      continue;
    }

    const helperAxes = ["x", "y", "z"].filter(a => a !== axis);
    let solvedByDetour = false;
    const offsets = [20, 40, 60, 80];

    for (const amount of offsets) {
      if (solvedByDetour) break;
      for (const s1 of [-1, 1]) {
        if (solvedByDetour) break;
        for (const s2 of [-1, 1]) {
          const helperTarget = {
            x: currentTcp.x,
            y: currentTcp.y,
            z: currentTcp.z
          };
          helperTarget[helperAxes[0]] += s1 * amount;
          helperTarget[helperAxes[1]] += s2 * amount;

          const helperSeg = planSegmentToTarget(currentPose, helperTarget, `Desvio ${axis.toUpperCase()}`);
          if (!helperSeg.ok) continue;

          const axisTarget = {
            x: helperSeg.endTcp.x,
            y: helperSeg.endTcp.y,
            z: helperSeg.endTcp.z
          };
          axisTarget[axis] = target[axis];

          const axisSeg = planSegmentToTarget(helperSeg.endPose, axisTarget, `Avance ${axis.toUpperCase()}`);
          if (!axisSeg.ok) continue;

          fullSequence.push(...helperSeg.sequence, ...axisSeg.sequence);
          currentPose = { ...axisSeg.endPose };
          currentTcp = { ...axisSeg.endTcp };
          notes.push(`${axis.toUpperCase()} con desvío 2 ejes`);
          solvedByDetour = true;
          break;
        }
      }
    }

    if (!solvedByDetour) {
      return { ok: false, reason: `No pude resolver el eje ${axis.toUpperCase()} con caminos directos ni desvíos mínimos.` };
    }
  }

  const finalSegment = planSegmentToTarget(currentPose, target, "Ajuste final");
  if (!finalSegment.ok) {
    return { ok: false, reason: finalSegment.reason };
  }

  fullSequence.push(...finalSegment.sequence);
  currentPose = { ...finalSegment.endPose };
  currentTcp = { ...finalSegment.endTcp };
  notes.push("ajuste final");

  return {
    ok: true,
    sequence: fullSequence,
    endPose: currentPose,
    endTcp: currentTcp,
    dist: distance3(currentTcp, target),
    strategy: notes.join(" | ")
  };
}

function syncFromHardwarePose(updates) {
  for (const [idStr, raw] of Object.entries(updates)) {
    const id = Number(idStr);
    const motor = motors.find(m => m.id === id);
    if (!motor) continue;
    let logical = logicalDegreesFromRaw(id, Number(raw));
    logical = Math.max(motor.min, Math.min(motor.max, logical));
    state[id] = logical;
    if (sliders[id]) sliders[id].value = logical;
    if (id === 2 || id === 3 || id === 4) lastValid[id] = logical;
  }
  syncLabels();
  drawScene();
}

function parseSerialLine(line) {
  const posMatches = [...line.matchAll(/POS ID\s+(\d+):\s+\d+\s+\(~([0-9.+-]+)\s+deg\)/g)];
  if (posMatches.length > 0) {
    const updates = {};
    for (const match of posMatches) updates[Number(match[1])] = Number(match[2]);
    syncFromHardwarePose(updates);
    return;
  }
  const compactMatches = [...line.matchAll(/M(\d+)=\d+\(([0-9.+-]+)deg\)/g)];
  if (compactMatches.length > 0) {
    const updates = {};
    for (const match of compactMatches) updates[Number(match[1])] = Number(match[2]);
    syncFromHardwarePose(updates);
  }
}

function applyStateValidation() {
  const { max } = getDynamicM4Max(state[2], state[3]);
  if (sliders[4]) sliders[4].max = max;
  if (state[4] > max) {
    state[4] = max;
    sliders[4].value = max;
  }
  const sim = classifyPose();
  if (sim.invalid) {
    state[2] = lastValid[2];
    state[3] = lastValid[3];
    state[4] = lastValid[4];
    sliders[2].value = state[2];
    sliders[3].value = state[3];
    sliders[4].value = state[4];
    return false;
  }
  lastValid[2] = state[2];
  lastValid[3] = state[3];
  lastValid[4] = state[4];
  return true;
}

function enqueueMove(id, angle) {
  if (!connected) return;
  commandQueue = commandQueue.filter(item => item.id !== id);
  commandQueue.push({ id, angle });
  updateQueueState();
  processQueue();
}

async function sendSerial(text) {
  const result = await window.desktopAPI.writeSerial(text.endsWith("\n") ? text : `${text}\n`);
  if (!result.ok) {
    log(`[ERROR] ${result.error}`);
  } else {
    log(`> ${text.trim()}`);
  }
}

async function processQueue() {
  if (!connected || queueBusy) return;
  queueBusy = true;
  while (commandQueue.length > 0 && connected) {
    const item = commandQueue.shift();
    updateQueueState();
    const wait = moveCooldownMs - (Date.now() - lastMoveSentAt);
    if (wait > 0) await new Promise(resolve => setTimeout(resolve, wait));
    lastMoveSentAt = Date.now();
    await sendSerial(`${item.id} ${item.angle}`);
  }
  queueBusy = false;
}

async function refreshPorts() {
  const ports = await window.desktopAPI.listPorts();
  portSelect.innerHTML = "";
  if (!ports.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "Sin puertos detectados";
    portSelect.appendChild(option);
    return;
  }
  for (const port of ports) {
    const option = document.createElement("option");
    option.value = port.path;
    option.textContent = `${port.path}${port.manufacturer ? ` - ${port.manufacturer}` : ""}`;
    portSelect.appendChild(option);
  }
}

async function connectSerial() {
  const path = portSelect.value;
  if (!path) {
    await window.desktopAPI.showError({ title: "Puerto", message: "Elegí un puerto COM antes de conectar." });
    return;
  }
  const result = await window.desktopAPI.connectSerial({ path, baudRate: Number(baudInput.value) || 115200 });
  if (!result.ok) {
    await window.desktopAPI.showError({ title: "Conexión", message: result.error });
    return;
  }
  connected = true;
  setConnState(`Conectado a ${path}`, "ok");
  log(`[INFO] Conectado a ${path}`);
  await sendSerial("posall");
}

async function disconnectSerial() {
  await window.desktopAPI.disconnectSerial();
  connected = false;
  commandQueue = [];
  updateQueueState();
  setConnState("Sin conectar");
}

function createMotorCards() {
  motorsPanel.innerHTML = "";
  for (const motor of motors) {
    const card = document.createElement("section");
    card.className = "motor-card";
    card.innerHTML = `
      <div class="motor-head">
        <strong>${motor.label}</strong>
        <span class="motor-role">${motor.role}</span>
      </div>
      <div class="motor-range">Rango: ${motor.min}..${motor.max}°</div>
      <div class="slider-row">
        <input data-id="${motor.id}" type="range" min="${motor.min}" max="${motor.max}" step="1" value="${motor.initial}">
        <span class="value" data-value="${motor.id}">${motor.initial}°</span>
      </div>
      ${motor.id === 4 ? `<div class="hint" data-badge="4">M4 max actual: 260°</div>` : ""}
      ${motor.id === 6 ? `<div class="axis"><span>cerrar</span><span>abrir</span></div>` : ""}
    `;
    motorsPanel.appendChild(card);
  }

  for (const motor of motors) {
    sliders[motor.id] = motorsPanel.querySelector(`input[data-id="${motor.id}"]`);
    labels[motor.id] = motorsPanel.querySelector(`[data-value="${motor.id}"]`);
    sliders[motor.id].dataset.lastCommitted = String(motor.initial);
  }
  motorBadges[4] = motorsPanel.querySelector('[data-badge="4"]');

  function commitSliderMove(motorId) {
    const slider = sliders[motorId];
    if (!connected || !slider) return;
    const committed = Number(slider.dataset.lastCommitted || "NaN");
    if (committed === state[motorId]) return;
    slider.dataset.lastCommitted = String(state[motorId]);
    enqueueMove(motorId, state[motorId]);
  }

  for (const motor of motors) {
    sliders[motor.id].addEventListener("input", () => {
      state[motor.id] = Number(sliders[motor.id].value);
      const valid = applyStateValidation();
      clearPendingTarget();
      syncLabels();
      drawScene();
      if (!valid) {
        sliders[motor.id].dataset.lastCommitted = String(state[motor.id]);
      }
    });

    sliders[motor.id].addEventListener("change", () => {
      commitSliderMove(motor.id);
    });

    sliders[motor.id].addEventListener("mouseup", () => {
      commitSliderMove(motor.id);
    });

    sliders[motor.id].addEventListener("touchend", () => {
      commitSliderMove(motor.id);
    });
  }
}

function rotateYCamera(p, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  return { x: p.x * c + p.z * s, y: p.y, z: -p.x * s + p.z * c };
}

function rotateXCamera(p, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  return { x: p.x, y: p.y * c - p.z * s, z: p.y * s + p.z * c };
}

function projectPoint(p) {
  let v = rotateYCamera(p, camera.yaw);
  v = rotateXCamera(v, camera.pitch);
  const z = v.z + camera.distance;
  const f = 540 / Math.max(60, z);
  return {
    x: canvas.width / 2 + v.x * f,
    y: canvas.height * 0.78 - v.y * f,
    z
  };
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.round(rect.width);
  canvas.height = Math.round(rect.height);
  drawScene();
}

canvas.addEventListener("mousedown", e => {
  dragging = true;
  canvas.classList.add("dragging");
  lastMouseX = e.clientX;
  lastMouseY = e.clientY;
});

window.addEventListener("mouseup", () => {
  dragging = false;
  canvas.classList.remove("dragging");
});

window.addEventListener("mousemove", e => {
  if (!dragging) return;
  const dx = e.clientX - lastMouseX;
  const dy = e.clientY - lastMouseY;
  lastMouseX = e.clientX;
  lastMouseY = e.clientY;
  camera.yaw += dx * 0.01;
  camera.pitch = Math.max(-1.2, Math.min(1.2, camera.pitch + dy * 0.01));
  drawScene();
});

canvas.addEventListener("wheel", e => {
  e.preventDefault();
  camera.distance = Math.max(260, Math.min(900, camera.distance + e.deltaY * 0.35));
  drawScene();
}, { passive: false });

document.getElementById("refreshPortsBtn").addEventListener("click", refreshPorts);
document.getElementById("connectBtn").addEventListener("click", connectSerial);
document.getElementById("disconnectBtn").addEventListener("click", disconnectSerial);
document.getElementById("scanBtn").addEventListener("click", () => sendSerial("scan"));
document.getElementById("posallBtn").addEventListener("click", () => sendSerial("posall"));
document.getElementById("stopQueueBtn").addEventListener("click", () => {
  commandQueue = [];
  updateQueueState();
  log("[INFO] Cola vaciada.");
});
document.getElementById("copyTcpBtn").addEventListener("click", () => {
  clearPendingTarget();
  targetXInput.value = Math.round(lastTcp.x);
  targetYInput.value = Math.round(lastTcp.y);
  targetZInput.value = Math.round(lastTcp.z);
  setTargetState("Objetivo copiado desde el TCP actual.");
  drawScene();
});
document.getElementById("solveTargetBtn").addEventListener("click", () => {
  const target = {
    x: Number(targetXInput.value),
    y: Number(targetYInput.value),
    z: Number(targetZInput.value)
  };
  setTargetState("Resolviendo objetivo...", "warn");
  setTimeout(() => {
    const startPose = {
      m1: state[1], m2: state[2], m3: state[3], m4: state[4], m5: state[5], m6: state[6]
    };
    const plan = planCartesianTarget(startPose, target);
    if (!plan.ok) {
      const nearest = findNearestSafePose(target, { m5: state[5], m6: state[6] });
      pendingTargetSolution = null;
      rejectedTargetPreview = nearest
        ? {
            candidate: nearest.candidate,
            tcp: nearest.tcp,
            dist: nearest.dist,
            reason: plan.reason
          }
        : null;
      setTargetState(
        rejectedTargetPreview
          ? `${plan.reason} Se muestra en rojo la pose segura más cercana. Error ${rejectedTargetPreview.dist.toFixed(1)} mm.`
          : plan.reason,
        "bad"
      );
      log(`[OBJETIVO] Rechazado: ${plan.reason}`);
      drawScene();
      return;
    }

    pendingTargetSolution = {
      target,
      startPose,
      candidate: plan.endPose,
      plan: { sequence: plan.sequence, strategy: plan.strategy },
      dist: plan.dist
    };
    rejectedTargetPreview = null;
    drawScene();

    setTargetState(
      `Objetivo calculado. Estrategia: ${plan.strategy}. Pasos: ${plan.sequence.length}. Error ${plan.dist.toFixed(1)} mm. TCP previsto: x ${plan.endTcp.x.toFixed(0)} | y ${plan.endTcp.y.toFixed(0)} | z ${plan.endTcp.z.toFixed(0)}. El brazo azul muestra la pose prevista.`,
      "ok"
    );
    log(`[OBJETIVO] Calculado. Estrategia ${plan.strategy}. Pasos ${plan.sequence.length}. Error ${plan.dist.toFixed(1)} mm.`);
  }, 20);
});
document.getElementById("moveTargetBtn").addEventListener("click", () => {
  if (!pendingTargetSolution) {
    setTargetState("Primero calculá un objetivo válido.", "warn");
    return;
  }

  if (connected) {
    enqueuePoseSequence(pendingTargetSolution.plan.sequence, pendingTargetSolution.startPose);
  }

  applyPoseToUi(pendingTargetSolution.candidate);

  setTargetState(
    `Moviendo objetivo calculado. Estrategia: ${pendingTargetSolution.plan.strategy}. Pasos: ${pendingTargetSolution.plan.sequence.length}.`,
    "ok"
  );
  log(`[OBJETIVO] Movimiento iniciado. Estrategia ${pendingTargetSolution.plan.strategy}.`);
  clearPendingTarget();
  drawScene();
});
document.getElementById("sendCommandBtn").addEventListener("click", () => {
  const cmd = commandInput.value.trim();
  if (!cmd) return;
  sendSerial(cmd);
  commandInput.value = "";
});
commandInput.addEventListener("keydown", e => {
  if (e.key !== "Enter") return;
  e.preventDefault();
  document.getElementById("sendCommandBtn").click();
});

window.desktopAPI.onSerialLine(line => {
  log(line);
  parseSerialLine(line);
});
window.desktopAPI.onSerialError(message => {
  log(`[ERROR SERIE] ${message}`);
  setConnState(`Error: ${message}`, "bad");
});
window.desktopAPI.onSerialClosed(() => {
  connected = false;
  setConnState("Puerto cerrado");
});

window.addEventListener("resize", resizeCanvas);

createMotorCards = function() {
  motorsPanel.innerHTML = "";
  for (const motor of motors) {
    const card = document.createElement("section");
    card.className = "motor-card";
    card.innerHTML = `
      <div class="motor-head">
        <strong>${motor.label}</strong>
        <span class="motor-role">${motor.role}</span>
      </div>
      <div class="motor-range">Rango: ${motor.min}..${motor.max}°</div>
      <div class="slider-row">
        <input data-id="${motor.id}" type="range" min="${motor.min}" max="${motor.max}" step="1" value="${motor.initial}">
        <span class="value" data-value="${motor.id}">${motor.initial}°</span>
      </div>
      <div class="motor-controls">
        <input class="motor-number" data-angle-input="${motor.id}" type="number" min="${motor.min}" max="${motor.max}" step="1" value="${motor.initial}">
        <button data-apply="${motor.id}">Aplicar</button>
        <button data-read="${motor.id}" class="secondary">Leer</button>
      </div>
      ${motor.id === 4 ? `<div class="hint" data-badge="4">M4 max actual: 260°</div>` : ""}
      ${motor.id === 6 ? `<div class="axis"><span>cerrar</span><span>abrir</span></div>` : ""}
    `;
    motorsPanel.appendChild(card);
  }

  for (const motor of motors) {
    sliders[motor.id] = motorsPanel.querySelector(`input[data-id="${motor.id}"]`);
    labels[motor.id] = motorsPanel.querySelector(`[data-value="${motor.id}"]`);
    angleInputs[motor.id] = motorsPanel.querySelector(`[data-angle-input="${motor.id}"]`);
    applyButtons[motor.id] = motorsPanel.querySelector(`[data-apply="${motor.id}"]`);
    readButtons[motor.id] = motorsPanel.querySelector(`[data-read="${motor.id}"]`);
    sliders[motor.id].dataset.lastCommitted = String(motor.initial);
  }
  motorBadges[4] = motorsPanel.querySelector('[data-badge="4"]');

  function clampMotorValue(motorId, value) {
    const motor = motors.find(m => m.id === motorId);
    if (!motor) return value;
    let nextValue = Math.round(Number(value));
    if (Number.isNaN(nextValue)) nextValue = state[motorId];
    nextValue = Math.max(motor.min, Math.min(motor.max, nextValue));
    if (motorId === 4) {
      const { max } = getDynamicM4Max(state[2], state[3]);
      nextValue = Math.min(nextValue, max);
    }
    return nextValue;
  }

  function previewMotorValue(motorId, value) {
    state[motorId] = clampMotorValue(motorId, value);
    const valid = applyStateValidation();
    clearPendingTarget();
    syncLabels();
    drawScene();
    return valid;
  }

  function applyMotorMove(motorId) {
    const nextValue = clampMotorValue(motorId, angleInputs[motorId]?.value ?? sliders[motorId].value);
    const valid = previewMotorValue(motorId, nextValue);
    sliders[motorId].dataset.lastCommitted = String(state[motorId]);
    if (!valid || !connected) return;
    enqueueMove(motorId, state[motorId]);
  }

  for (const motor of motors) {
    sliders[motor.id].addEventListener("input", () => {
      const valid = previewMotorValue(motor.id, sliders[motor.id].value);
      if (!valid) {
        sliders[motor.id].dataset.lastCommitted = String(state[motor.id]);
      }
    });

    sliders[motor.id].addEventListener("change", () => {
      sliders[motor.id].dataset.lastCommitted = String(state[motor.id]);
    });

    sliders[motor.id].addEventListener("mouseup", () => {
      sliders[motor.id].dataset.lastCommitted = String(state[motor.id]);
    });

    sliders[motor.id].addEventListener("touchend", () => {
      sliders[motor.id].dataset.lastCommitted = String(state[motor.id]);
    });

    angleInputs[motor.id].addEventListener("input", () => {
      previewMotorValue(motor.id, angleInputs[motor.id].value);
    });

    angleInputs[motor.id].addEventListener("keydown", e => {
      if (e.key !== "Enter") return;
      e.preventDefault();
      applyMotorMove(motor.id);
    });

    applyButtons[motor.id].addEventListener("click", () => {
      applyMotorMove(motor.id);
    });

    readButtons[motor.id].addEventListener("click", () => {
      sendSerial(`read ${motor.id}`);
    });
  }
};

createMotorCards();
syncLabels();
drawScene();
updateQueueState();
refreshPorts();
