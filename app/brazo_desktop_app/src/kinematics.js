(function () {
  "use strict";

  const defaultGeom = {
    hBase: 100,
    l1: 113,
    offsetM3: 27,
    l2: 137,
    l3: 60,
    toolBodyLen: 60,
    jawLen: 45
  };

  function degToRad(deg) {
    return deg * Math.PI / 180;
  }

  function add3(a, b) {
    return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
  }

  function scale3(v, k) {
    return { x: v.x * k, y: v.y * k, z: v.z * k };
  }

  function sub2(a, b) {
    return { x: a.x - b.x, y: a.y - b.y };
  }

  function len2(v) {
    return Math.hypot(v.x, v.y);
  }

  function normalize2(v) {
    const l = len2(v) || 1;
    return { x: v.x / l, y: v.y / l };
  }

  function vec2(length, angleRad) {
    return { x: length * Math.cos(angleRad), y: length * Math.sin(angleRad) };
  }

  function to3DFromYaw(point2, yawRad) {
    return {
      x: point2.x * Math.cos(yawRad),
      y: point2.y,
      z: -point2.x * Math.sin(yawRad)
    };
  }

  function vector2To3D(vec2d, yawRad) {
    return normalize3({
      x: vec2d.x * Math.cos(yawRad),
      y: vec2d.y,
      z: -vec2d.x * Math.sin(yawRad)
    });
  }

  function normalize3(v) {
    const l = Math.hypot(v.x, v.y, v.z) || 1;
    return { x: v.x / l, y: v.y / l, z: v.z / l };
  }

  function cross3(a, b) {
    return {
      x: a.y * b.z - a.z * b.y,
      y: a.z * b.x - a.x * b.z,
      z: a.x * b.y - a.y * b.x
    };
  }

  function rotateAroundAxis(v, axis, ang) {
    const u = normalize3(axis);
    const c = Math.cos(ang);
    const s = Math.sin(ang);
    const dot = v.x * u.x + v.y * u.y + v.z * u.z;
    return {
      x: v.x * c + (u.y * v.z - u.z * v.y) * s + u.x * dot * (1 - c),
      y: v.y * c + (u.z * v.x - u.x * v.z) * s + u.y * dot * (1 - c),
      z: v.z * c + (u.x * v.y - u.y * v.x) * s + u.z * dot * (1 - c)
    };
  }

  function forwardKinematics(inputAngles, geom = defaultGeom) {
    const angles = {
      m1: inputAngles.m1,
      m2: inputAngles.m2,
      m3: inputAngles.m3,
      m4: inputAngles.m4,
      m5: inputAngles.m5 ?? 170,
      m6: inputAngles.m6 ?? 70
    };

    const yaw = degToRad(angles.m1 - 180);
    const a1 = degToRad((angles.m2 - 180) + 90);
    const a2 = degToRad(angles.m3 - 90);
    const a3 = degToRad(angles.m4 - 180);

    const p0_2d = { x: 0, y: geom.hBase };
    const p1_2d = {
      x: p0_2d.x + vec2(geom.l1, a1).x,
      y: p0_2d.y + vec2(geom.l1, a1).y
    };

    const offset = {
      x: geom.offsetM3 * Math.cos(a1 + Math.PI / 2),
      y: geom.offsetM3 * Math.sin(a1 + Math.PI / 2)
    };
    const p1o_2d = { x: p1_2d.x + offset.x, y: p1_2d.y + offset.y };

    const p2_2d = {
      x: p1o_2d.x + vec2(geom.l2, a1 + a2).x,
      y: p1o_2d.y + vec2(geom.l2, a1 + a2).y
    };

    const p3_2d = {
      x: p2_2d.x + vec2(geom.l3, a1 + a2 + a3).x,
      y: p2_2d.y + vec2(geom.l3, a1 + a2 + a3).y
    };

    const toolDir2 = normalize2(sub2(p3_2d, p2_2d));
    const toolBodyCenter2 = {
      x: p3_2d.x + toolDir2.x * (geom.toolBodyLen / 2),
      y: p3_2d.y + toolDir2.y * (geom.toolBodyLen / 2)
    };
    const jawBaseCenter2 = {
      x: p3_2d.x + toolDir2.x * geom.toolBodyLen,
      y: p3_2d.y + toolDir2.y * geom.toolBodyLen
    };

    const gap = 6 + (angles.m6 / 140) * 26;
    const tcp2 = {
      x: jawBaseCenter2.x + toolDir2.x * geom.jawLen,
      y: jawBaseCenter2.y + toolDir2.y * geom.jawLen
    };

    p0_2d.x *= -1;
    p1_2d.x *= -1;
    p1o_2d.x *= -1;
    p2_2d.x *= -1;
    p3_2d.x *= -1;
    toolBodyCenter2.x *= -1;
    jawBaseCenter2.x *= -1;
    tcp2.x *= -1;

    const toolU = vector2To3D(toolDir2, yaw);
    let toolV = vector2To3D({ x: -toolDir2.y, y: toolDir2.x }, yaw);
    const toolWBase = normalize3({ x: -Math.sin(yaw), y: 0, z: -Math.cos(yaw) });
    toolV = rotateAroundAxis(toolV, toolU, degToRad(angles.m5 - 170));
    const toolW = rotateAroundAxis(toolWBase, toolU, degToRad(angles.m5 - 170));

    const p0 = { x: 0, y: 0, z: 0 };
    const m2 = to3DFromYaw(p0_2d, yaw);
    const m3 = to3DFromYaw(p1o_2d, yaw);
    const m4 = to3DFromYaw(p2_2d, yaw);
    const m5 = to3DFromYaw(p3_2d, yaw);
    const toolBodyCenter = to3DFromYaw(toolBodyCenter2, yaw);
    const jawBaseCenter = to3DFromYaw(jawBaseCenter2, yaw);
    const tcp = to3DFromYaw(tcp2, yaw);

    const upperJaw = add3(
      add3(jawBaseCenter, scale3(toolU, geom.jawLen / 2)),
      scale3(toolV, gap / 2)
    );
    const lowerJaw = add3(
      add3(jawBaseCenter, scale3(toolU, geom.jawLen / 2)),
      scale3(toolV, -gap / 2)
    );

    return {
      angles,
      points: {
        origin: p0,
        m2,
        m3,
        m4,
        m5,
        toolBodyCenter,
        jawBaseCenter,
        tcp,
        upperJaw,
        lowerJaw
      },
      axes: {
        toolU,
        toolV,
        toolW
      },
      derived: {
        yawRad: yaw,
        gripGap: gap
      }
    };
  }

  window.BrazoKinematics = {
    defaultGeom,
    forwardKinematics
  };
})();
