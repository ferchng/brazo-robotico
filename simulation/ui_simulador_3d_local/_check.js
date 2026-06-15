
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

    const floorTouch = 5;
    const floorRisk = 30;
    const mirrorX = true;
    const offsetSign = 1;

    const canvas = document.getElementById("viewer");
    const ctx = canvas.getContext("2d");
    const statusEl = document.getElementById("status");
    const sliders = {
      1: document.getElementById("m1"),
      2: document.getElementById("m2"),
      3: document.getElementById("m3"),
      4: document.getElementById("m4"),
      5: document.getElementById("m5"),
      6: document.getElementById("m6")
    };
    const labels = {
      1: document.getElementById("m1v"),
      2: document.getElementById("m2v"),
      3: document.getElementById("m3v"),
      4: document.getElementById("m4v"),
      5: document.getElementById("m5v"),
      6: document.getElementById("m6v")
    };
    const m4maxEl = document.getElementById("m4max");

    const state = { 1: 180, 2: 180, 3: 90, 4: 180, 5: 170, 6: 70 };
    const lastValid = { ...state };
    const camera = { yaw: 0.9, pitch: 0.55, distance: 520 };
    let dragging = false;
    let lastMouseX = 0;
    let lastMouseY = 0;

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
      const m2Deg = state[2];
      const m3Deg = state[3];
      const m4Deg = state[4];
      const a1 = degToRad((m2Deg - 180) + 90);
      const a2 = degToRad(m3Deg - 90);
      const a3 = degToRad(m4Deg - 180);

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
        z: p2.x * Math.sin(yaw)
      };
    }

    function liftVec2To3(v2) {
      const yaw = degToRad(state[1] - 180);
      return norm3({
        x: v2.x * Math.cos(yaw),
        y: v2.y,
        z: v2.x * Math.sin(yaw)
      });
    }

    function planeNormal3() {
      const yaw = degToRad(state[1] - 180);
      return norm3({ x: -Math.sin(yaw), y: 0, z: Math.cos(yaw) });
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
      const corners = [
        add3(add3(add3(center, scale3(u, -hu)), scale3(v, -hv)), scale3(w, -hw)),
        add3(add3(add3(center, scale3(u, hu)), scale3(v, -hv)), scale3(w, -hw)),
        add3(add3(add3(center, scale3(u, hu)), scale3(v, hv)), scale3(w, -hw)),
        add3(add3(add3(center, scale3(u, -hu)), scale3(v, hv)), scale3(w, -hw)),
        add3(add3(add3(center, scale3(u, -hu)), scale3(v, -hv)), scale3(w, hw)),
        add3(add3(add3(center, scale3(u, hu)), scale3(v, -hv)), scale3(w, hw)),
        add3(add3(add3(center, scale3(u, hu)), scale3(v, hv)), scale3(w, hw)),
        add3(add3(add3(center, scale3(u, -hu)), scale3(v, hv)), scale3(w, hw))
      ];
      return corners;
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

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#050914";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      drawFloorPlane(0, "#94a3b8");
      drawFloorPlane(floorTouch, "#ef4444");
      drawFloorPlane(floorRisk, "#f59e0b");

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

      drawBox(
        boxVerticesFromBasis(upperJawCenter, u3, v3, w3, geom.jawLen, geom.jawWidth, geom.jawHeight),
        "rgba(250,204,21,0.78)",
        "#fde68a"
      );
      drawBox(
        boxVerticesFromBasis(lowerJawCenter, u3, v3, w3, geom.jawLen, geom.jawWidth, geom.jawHeight),
        "rgba(250,204,21,0.78)",
        "#fde68a"
      );

      m4maxEl.textContent = `M4 max actual: ${max}°`;
      statusEl.innerHTML =
        `Estado pose: <span class="${sim.invalid ? "bad" : "ok"}">${sim.invalid ? sim.reason : "PERMITIDA"}</span>\n` +
        `Clearance piso: <span class="${sim.reason === "PISO" ? "bad" : (sim.reason === "RIESGO_PISO" ? "warn" : "ok")}">${sim.minY.toFixed(1)} mm</span>\n` +
        `M1=${state[1]}° | M5=${state[5]}° | M6=${state[6]}°\n` +
        `Reglas activas: ${reasons.length ? reasons.join(" | ") : "Sin regla dinámica activa."}`;
    }

    function syncLabels() {
      Object.keys(labels).forEach(id => {
        labels[id].textContent = `${state[id]}°`;
      });
    }

    function setStateFromSliders() {
      Object.keys(sliders).forEach(id => {
        state[id] = Number(sliders[id].value);
      });

      const { max } = getDynamicM4Max(state[2], state[3]);
      sliders[4].max = max;
      if (state[4] > max) {
        state[4] = max;
        sliders[4].value = max;
      }

      const sim = classifyPose();
      if (sim.invalid) {
        state[1] = Number(sliders[1].value);
        state[5] = Number(sliders[5].value);
        state[6] = Number(sliders[6].value);
        state[2] = lastValid[2];
        state[3] = lastValid[3];
        state[4] = lastValid[4];
        sliders[2].value = state[2];
        sliders[3].value = state[3];
        sliders[4].value = state[4];
      } else {
        lastValid[2] = state[2];
        lastValid[3] = state[3];
        lastValid[4] = state[4];
      }

      syncLabels();
      drawScene();
    }

    Object.values(sliders).forEach(slider => {
      slider.addEventListener("input", setStateFromSliders);
    });

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

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.round(rect.width);
      canvas.height = Math.round(rect.height);
      drawScene();
    }

    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();
    syncLabels();
    setStateFromSliders();
  