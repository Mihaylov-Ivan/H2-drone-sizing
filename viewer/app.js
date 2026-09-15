import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COLORS = {
  fuselage: 0xc9d2dc,
  wing: 0x7aa8d4,
  boom: 0x5f6b78,
  tail: 0x9bb8a5,
  prop: 0xd4a574,
  motor: 0x3d4550,
  accent: 0x6eb6ff,
  battery: 0xc4a35a,
  fuelCell: 0x6aa84f,
  fuelCellControl: 0x8fbc6b,
  hydrogen: 0x5b8def,
  payload: 0xc27a6a,
  cube: 0xb07cc6,
  pdb: 0x6e7f91,
  coolingInlet: 0x32c5ff,
  coolingExhaust: 0xff7043,
  spar: 0xd4b483,
  longeron: 0xc4ccd6,
  landingSleeve: 0xb7c4b0,
  landingTube: 0xd4b483,
  landingFoot: 0x6e7f91,
};

/** @type {Record<string, number|string>} */
let P = {};
/** @type {THREE.Group|null} */
let aircraft = null;
const groups = {};

const viewport = document.getElementById('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1419);
scene.fog = new THREE.Fog(0x0f1419, 8, 28);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0.55, 0, 0.05);

const hemi = new THREE.HemisphereLight(0xdde8f5, 0x1a222c, 1.15);
scene.add(hemi);
const key = new THREE.DirectionalLight(0xffffff, 1.05);
key.position.set(2.5, -3.5, 4);
scene.add(key);
const fill = new THREE.DirectionalLight(0x8eb6ff, 0.35);
fill.position.set(-2, 2, 1.5);
scene.add(fill);

const grid = new THREE.GridHelper(6, 24, 0x2a3442, 0x1c2430);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

const axes = new THREE.AxesHelper(0.6);
scene.add(axes);

function mm(name, fallback = 0) {
  const v = P[name];
  return (typeof v === 'number' ? v : fallback) / 1000;
}

function num(name, fallback = 0) {
  const v = P[name];
  return typeof v === 'number' ? v : fallback;
}

function str(name, fallback = '') {
  const v = P[name];
  return typeof v === 'string' ? v : fallback;
}

function paramsFromCad(doc) {
  /** @type {Record<string, number|string>} */
  const out = {};
  for (const row of doc.parameters || []) {
    out[row.name] = row.value;
  }
  return out;
}

/** Approximate NACA 4-digit mean thickness envelope (unit chord, closed TE). */
function nacaThicknessPoints(code, n = 36) {
  const digits = String(code || '0012').replace(/[^0-9]/g, '');
  const t = digits.length >= 4 ? parseInt(digits.slice(-2), 10) / 100 : 0.12;
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const x = i / n;
    const yt =
      5 *
      t *
      (0.2969 * Math.sqrt(x) -
        0.126 * x -
        0.3516 * x * x +
        0.2843 * x * x * x -
        0.1015 * x * x * x * x);
    pts.push({ x, yt: Math.max(yt, 0.0005) });
  }
  return pts;
}

function makeMaterial(color, opts = {}) {
  return new THREE.MeshStandardMaterial({
    color,
    metalness: 0.08,
    roughness: 0.62,
    flatShading: false,
    side: THREE.DoubleSide,
    ...opts,
  });
}

/**
 * Central pod loft.
 * square_filleted (default): midbody is a rounded square; nose/tail morph from
 * that square base to a circular tip. circular: legacy surface of revolution.
 */
function fuselageMesh() {
  const sections = ['nose', 'center_a', 'center_b', 'tail']
    .map((name) => ({
      name,
      start: mm(`fuselage_${name}_start_x`),
      end: mm(`fuselage_${name}_end_x`),
      dia: mm(`fuselage_${name}_diameter`, mm('fuselage_max_diameter', 180)),
    }))
    .filter((s) => s.end > s.start + 1e-6);

  const g = new THREE.Group();
  g.name = 'fuselage';
  if (!sections.length) return g;

  const x0 = sections[0].start;
  const x1 = sections[sections.length - 1].end;
  const length = x1 - x0;
  const maxR = Math.max(...sections.map((s) => s.dia / 2));
  const cross = String(str('fuselage_cross_section', 'rectangular_filleted')).toLowerCase();
  const halfW0 = 0.5 * mm('fuselage_width', mm('fuselage_max_diameter', 180));
  const halfH0 = 0.5 * mm('fuselage_height', mm('fuselage_max_diameter', 180));
  // CAD stores fillet in mm; mm() converts to metres.
  const filletM = Math.max(
    0,
    mm('fuselage_fillet_radius', Math.min(20, 0.35 * Math.min(halfW0, halfH0) * 1000)),
  );
  const useRect = cross !== 'circular';
  const forceSquare = cross === 'square_filleted';

  function halfAt(x) {
    let sec = sections[sections.length - 1];
    for (const s of sections) {
      if (x <= s.end + 1e-9) {
        sec = s;
        break;
      }
    }
    const R = sec.dia / 2;
    const u = (x - sec.start) / Math.max(sec.end - sec.start, 1e-9);
    // Scale factor for nose/tail taper (1 at full section, 0 at tip).
    let scale = 1;
    if (sec.name === 'nose') {
      const t = Math.min(Math.max(u, 0), 1);
      scale = Math.sqrt(Math.max(0, 1 - (1 - t) * (1 - t)));
    } else if (sec.name === 'tail') {
      const t = Math.min(Math.max(u, 0), 1);
      const rTip = Math.max(0.18, 0.008 / Math.max(R, 1e-6));
      scale = rTip + (1 - rTip) * Math.sqrt(Math.max(0, 1 - t * t));
    } else {
      const idx = sections.indexOf(sec);
      const rA = sections[Math.max(0, idx - 1)].dia / 2;
      const rB = R;
      const rC = sections[Math.min(sections.length - 1, idx + 1)].dia / 2;
      const rStart = 0.5 * (rA + rB);
      const rEnd = 0.5 * (rB + rC);
      const s = u * u * (3 - 2 * u);
      const r = rStart + (rEnd - rStart) * s;
      scale = r / Math.max(maxR, 1e-6);
    }
    let hw = halfW0 * scale;
    let hh = halfH0 * scale;
    if (forceSquare) {
      const s = Math.max(hw, hh);
      hw = hh = s;
    }
    return [hw, hh, scale];
  }

  /** morph: 0 = circle, 1 = rounded rectangle. */
  function morphAt(x) {
    if (!useRect) return 0;
    let sec = sections[sections.length - 1];
    for (const s of sections) {
      if (x <= s.end + 1e-9) {
        sec = s;
        break;
      }
    }
    const u = (x - sec.start) / Math.max(sec.end - sec.start, 1e-9);
    if (sec.name === 'nose') return Math.min(Math.max(u, 0), 1);
    if (sec.name === 'tail') return 1 - Math.min(Math.max(u, 0), 1);
    return 1;
  }

  function roundedRectPoint(halfW, halfH, fillet, theta) {
    const aw = Math.max(halfW, 1e-6);
    const ah = Math.max(halfH, 1e-6);
    const rf = Math.min(Math.max(fillet, 0), 0.49 * Math.min(aw, ah));
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    const tx = Math.abs(c) > 1e-9 ? aw / Math.abs(c) : 1e9;
    const ty = Math.abs(s) > 1e-9 ? ah / Math.abs(s) : 1e9;
    const t = Math.min(tx, ty);
    let y = t * c;
    let z = t * s;
    const flatY = aw - rf;
    const flatZ = ah - rf;
    if (rf > 1e-9 && Math.abs(y) > flatY && Math.abs(z) > flatZ) {
      const cx = Math.sign(y) * flatY;
      const cz = Math.sign(z) * flatZ;
      const ang = Math.atan2(z - cz, y - cx);
      y = cx + rf * Math.cos(ang);
      z = cz + rf * Math.sin(ang);
    }
    return [y, z];
  }

  function sectionPoint(halfW, halfH, fillet, theta, morph) {
    // Rectangular modes morph from an inscribed ellipse (not a max-radius circle)
    // so nose/tail never bulge past the local half-width / half-height. The ellipse
    // is tangent to the midbody flats at the side and top/bottom stations.
    if (!useRect) {
      const rc = Math.max(halfW, halfH);
      return [rc * Math.cos(theta), rc * Math.sin(theta)];
    }
    const ey = halfW * Math.cos(theta);
    const ez = halfH * Math.sin(theta);
    if (morph <= 1e-6) return [ey, ez];
    const [sy, sz] = roundedRectPoint(halfW, halfH, fillet, theta);
    const m = Math.min(Math.max(morph, 0), 1);
    return [ey + (sy - ey) * m, ez + (sz - ez) * m];
  }

  const axial = 72;
  const radial = 64;
  const positions = [];
  const indices = [];
  const tipEps = 1e-4;

  for (let i = 0; i <= axial; i++) {
    const t = i / axial;
    const x = x0 + t * length;
    let [halfW, halfH] = halfAt(x);
    let morph = morphAt(x);
    let fillet = filletM * Math.min(halfW / Math.max(halfW0, 1e-6), halfH / Math.max(halfH0, 1e-6));
    if (i === 0) {
      halfW = tipEps;
      halfH = tipEps;
      morph = 0;
    }
    if (i === axial) {
      const tip = Math.max(Math.min(halfW0, halfH0) * 0.02, tipEps);
      halfW = tip;
      halfH = tip;
      morph = 0;
    }
    for (let j = 0; j < radial; j++) {
      const a = (j / radial) * Math.PI * 2;
      const [y, z] = sectionPoint(halfW, halfH, fillet, a, morph);
      positions.push(x, y, z);
    }
  }

  for (let i = 0; i < axial; i++) {
    for (let j = 0; j < radial; j++) {
      const j2 = (j + 1) % radial;
      const a = i * radial + j;
      const b = i * radial + j2;
      const c = (i + 1) * radial + j;
      const d = (i + 1) * radial + j2;
      indices.push(a, c, b, b, c, d);
    }
  }

  const tipCenter = positions.length / 3;
  positions.push(x1, 0, 0);
  const lastRing = axial * radial;
  for (let j = 0; j < radial; j++) {
    const j2 = (j + 1) % radial;
    indices.push(tipCenter, lastRing + j2, lastRing + j);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();

  const mesh = new THREE.Mesh(geo, makeMaterial(COLORS.fuselage));
  mesh.userData.maxRadius_m = maxR;
  g.add(mesh);
  return g;
}

/**
 * Tapered lifting surface lofted from NACA thickness (horizontal).
 * LE at x=le, span along ±y, thickness about local z.
 */
function horizontalSurface({
  le,
  span,
  rootChord,
  tipChord,
  z,
  thickness,
  naca,
  sweepDeg = 0,
  dihedralDeg = 0,
  twistRootDeg = 0,
  twistTipDeg = 0,
  color,
}) {
  const half = span / 2;
  const profile = nacaThicknessPoints(naca, 28);
  const stations = 18;
  const positions = [];
  const indices = [];
  const tScaleRoot = thickness / (2 * Math.max(...profile.map((p) => p.yt)) * rootChord);

  for (let i = 0; i <= stations; i++) {
    const eta = i / stations;
    const y = -half + eta * span;
    const chord = rootChord + (tipChord - rootChord) * Math.abs(y) / half;
    const sweep = (sweepDeg * Math.PI) / 180;
    const dihedral = (dihedralDeg * Math.PI) / 180;
    const twist = (((twistRootDeg + (twistTipDeg - twistRootDeg) * Math.abs(y) / half) * Math.PI) / 180);
    const leX = le + Math.abs(y) * Math.tan(sweep);
    const z0 = z + Math.abs(y) * Math.tan(dihedral);
    const tScale = tScaleRoot * (chord / rootChord);

    for (const side of [1, -1]) {
      for (const p of profile) {
        const xc = p.x * chord;
        const zt = side * p.yt * chord * tScale;
        const xr = xc * Math.cos(twist) + zt * Math.sin(twist);
        const zr = -xc * Math.sin(twist) + zt * Math.cos(twist);
        positions.push(leX + xr, y, z0 + zr);
      }
    }
  }

  const nProf = profile.length;
  const ring = nProf * 2;
  for (let i = 0; i < stations; i++) {
    for (let j = 0; j < nProf - 1; j++) {
      const a = i * ring + j;
      const b = a + 1;
      const c = (i + 1) * ring + j;
      const d = c + 1;
      indices.push(a, c, b, b, c, d);
      const a2 = a + nProf;
      const b2 = b + nProf;
      const c2 = c + nProf;
      const d2 = d + nProf;
      indices.push(a2, b2, c2, b2, d2, c2);
    }
    // LE/TE caps between upper/lower
    const uLE = i * ring;
    const lLE = uLE + nProf;
    const uLE2 = (i + 1) * ring;
    const lLE2 = uLE2 + nProf;
    indices.push(uLE, lLE, uLE2, lLE, lLE2, uLE2);
    const uTE = uLE + nProf - 1;
    const lTE = lLE + nProf - 1;
    const uTE2 = uLE2 + nProf - 1;
    const lTE2 = lLE2 + nProf - 1;
    indices.push(uTE, uTE2, lTE, lTE, uTE2, lTE2);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return new THREE.Mesh(geo, makeMaterial(color));
}

/**
 * Vertical fin in the x–z plane at lateral station y, root on boom waterline.
 */
function verticalFin({ le, y, height, rootChord, tipChord, thickness, naca, color, z0 = 0 }) {
  const profile = nacaThicknessPoints(naca, 24);
  const stations = 12;
  const positions = [];
  const indices = [];
  const tScaleRoot = thickness / (2 * Math.max(...profile.map((p) => p.yt)) * rootChord);

  for (let i = 0; i <= stations; i++) {
    const eta = i / stations;
    const z = z0 + eta * height;
    const chord = rootChord + (tipChord - rootChord) * eta;
    const tScale = tScaleRoot * (chord / rootChord);
    for (const side of [1, -1]) {
      for (const p of profile) {
        const xc = p.x * chord;
        const yt = side * p.yt * chord * tScale;
        positions.push(le + xc, y + yt, z);
      }
    }
  }

  const nProf = profile.length;
  const ring = nProf * 2;
  for (let i = 0; i < stations; i++) {
    for (let j = 0; j < nProf - 1; j++) {
      const a = i * ring + j;
      const b = a + 1;
      const c = (i + 1) * ring + j;
      const d = c + 1;
      indices.push(a, c, b, b, c, d);
      const a2 = a + nProf;
      const b2 = b + nProf;
      const c2 = c + nProf;
      const d2 = d + nProf;
      indices.push(a2, b2, c2, b2, d2, c2);
    }
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return new THREE.Mesh(geo, makeMaterial(color));
}

function boomMeshes() {
  const g = new THREE.Group();
  g.name = 'booms';
  const x0 = mm('boom_start_x');
  const x1 = mm('boom_end_x');
  const L = x1 - x0;
  const r = mm('boom_OD', 21) / 2;
  const z = mm('boom_z', mm('wing_z', 100));
  const ys = [mm('boom_left_y', -350), mm('boom_right_y', 350)];
  const mat = makeMaterial(COLORS.boom);
  for (const y of ys) {
    const geo = new THREE.CylinderGeometry(r, r, L, 20);
    geo.rotateZ(Math.PI / 2);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set((x0 + x1) / 2, y, z);
    g.add(mesh);
  }
  return g;
}

function propDisk(diameter, opacity = 0.22) {
  const geo = new THREE.CircleGeometry(diameter / 2, 48);
  const mat = makeMaterial(COLORS.prop, {
    transparent: true,
    opacity,
    depthWrite: false,
    side: THREE.DoubleSide,
    metalness: 0,
    roughness: 1,
  });
  return new THREE.Mesh(geo, mat);
}

/** CFRP-style tube between two aircraft points (x,y,z). */
function tubeBetween(a, b, radius, material) {
  const pa = new THREE.Vector3(a[0], a[1], a[2]);
  const pb = new THREE.Vector3(b[0], b[1], b[2]);
  const dir = new THREE.Vector3().subVectors(pb, pa);
  const len = dir.length();
  if (len < 1e-6) return null;
  const geo = new THREE.CylinderGeometry(radius, radius, len, 16);
  const mesh = new THREE.Mesh(geo, material);
  mesh.position.copy(pa).add(pb).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  return mesh;
}

/**
 * Structural tubes: twin booms already carry the lift motors. Only add the
 * short fuselage→pusher bulkhead stub (motors sit directly on the boom rods).
 */
function mountTubeMeshes() {
  const g = new THREE.Group();
  g.name = 'mounts';
  const mat = makeMaterial(COLORS.boom);
  const boomR = mm('boom_OD', 21) / 2;
  const pusherR = Math.max(boomR * 0.85, 0.006);

  const fuseEnd = mm('fuselage_total_length', 780);
  const px = mm('pusher_motor_x');
  const py = mm('pusher_motor_y');
  const pz = mm('pusher_motor_z');
  // Short bulkhead stub only if the motor centre is clearly aft of the skin.
  if (px > fuseEnd + 0.005) {
    const stalk = tubeBetween([fuseEnd, 0, 0], [px, py, pz], pusherR, mat);
    if (stalk) g.add(stalk);
  }
  return g;
}

function addBoxAt(g, x, y, z, L, W, H, color) {
  if (!(L > 0) || !(W > 0) || !(H > 0)) return;
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(L, W, H),
    makeMaterial(color, { transparent: true, opacity: 0.92, metalness: 0.05, roughness: 0.55 }),
  );
  mesh.position.set(x, y, z);
  g.add(mesh);
}

function addCylinderX(g, x, y, z, length, diameter, color) {
  if (!(length > 0) || !(diameter > 0)) return;
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(diameter / 2, diameter / 2, length, 28),
    makeMaterial(color, { transparent: true, opacity: 0.92, metalness: 0.08, roughness: 0.45 }),
  );
  mesh.rotation.z = Math.PI / 2;
  mesh.position.set(x, y, z);
  g.add(mesh);
}

/** Packaging envelopes inside the central pod. */
function internalPackMeshes() {
  const g = new THREE.Group();
  g.name = 'internals';

  // Payload — box (width/height may be screening fallbacks from CAD export).
  if (P.payload_length != null) {
    addBoxAt(
      g,
      mm('payload_x'),
      mm('payload_y'),
      mm('payload_z'),
      mm('payload_length'),
      mm('payload_width', 80),
      mm('payload_height', 80),
      COLORS.payload,
    );
  }

  // Fuel cell stack — box
  if (P.fuel_cell_length != null) {
    addBoxAt(
      g,
      mm('fuel_cell_x'),
      mm('fuel_cell_y'),
      mm('fuel_cell_z'),
      mm('fuel_cell_length'),
      mm('fuel_cell_width'),
      mm('fuel_cell_height'),
      COLORS.fuelCell,
    );
  }

  // FCPM control box — aft of fuel cell, over H₂ cylinder
  if (P.fuel_cell_control_box_length != null) {
    addBoxAt(
      g,
      mm('fuel_cell_control_box_x'),
      mm('fuel_cell_control_box_y'),
      mm('fuel_cell_control_box_z'),
      mm('fuel_cell_control_box_length'),
      mm('fuel_cell_control_box_width'),
      mm('fuel_cell_control_box_height'),
      COLORS.fuelCellControl,
    );
  }

  // Buffer battery — box
  if (P.battery_length != null) {
    addBoxAt(
      g,
      mm('battery_x'),
      mm('battery_y'),
      mm('battery_z'),
      mm('battery_length'),
      mm('battery_width'),
      mm('battery_height'),
      COLORS.battery,
    );
  }

  // H2 cylinder — axial cylinder
  if (P.hydrogen_cylinder_length != null && P.hydrogen_cylinder_diameter != null) {
    addCylinderX(
      g,
      mm('hydrogen_cylinder_x'),
      mm('hydrogen_cylinder_y'),
      mm('hydrogen_cylinder_z'),
      mm('hydrogen_cylinder_length'),
      mm('hydrogen_cylinder_diameter'),
      COLORS.hydrogen,
    );
  }

  // Cube Orange — over payload in nose
  if (P.cube_orange_length != null) {
    addBoxAt(
      g,
      mm('cube_orange_x'),
      mm('cube_orange_y'),
      mm('cube_orange_z'),
      mm('cube_orange_length'),
      mm('cube_orange_width'),
      mm('cube_orange_height'),
      COLORS.cube,
    );
  }

  // Dual PDBs on H₂ cylinder top
  for (const prefix of ['pdb', 'pdb2']) {
    if (P[`${prefix}_length`] == null) continue;
    addBoxAt(
      g,
      mm(`${prefix}_x`),
      mm(`${prefix}_y`),
      mm(`${prefix}_z`),
      mm(`${prefix}_length`),
      mm(`${prefix}_width`),
      mm(`${prefix}_height`),
      COLORS.pdb,
    );
  }

  return g;
}

function propulsionMeshes() {
  const g = new THREE.Group();
  g.name = 'props';
  const motorMat = makeMaterial(COLORS.motor);
  const frontX = mm('lift_motor_front_x');
  const rearX = mm('lift_motor_rear_x');
  const yAbs = mm('lift_motor_y_abs');
  const z = mm('lift_motor_z', mm('wing_z', 100));
  const liftD = mm('lift_prop_diameter');
  const motorOd = mm('lift_motor_OD', 80);
  const motorLen = mm('lift_motor_body_length', 30);

  for (const x of [frontX, rearX]) {
    for (const y of [-yAbs, yAbs]) {
      const body = new THREE.Mesh(
        new THREE.CylinderGeometry(motorOd / 2, motorOd / 2, motorLen, 20),
        motorMat,
      );
      // Cylinder default axis is Y; rotate onto +Z so it matches horizontal lift prop disks.
      body.rotation.x = Math.PI / 2;
      body.position.set(x, y, z);
      g.add(body);
      const disk = propDisk(liftD);
      disk.position.set(x, y, z + motorLen / 2 + 0.002);
      g.add(disk);
    }
  }

  const px = mm('pusher_motor_x');
  const py = mm('pusher_motor_y');
  const pz = mm('pusher_motor_z');
  const pOd = mm('pusher_motor_OD', 55);
  const pLen = mm('pusher_motor_body_length', 34);
  const pD = mm('pusher_prop_diameter');
  const pBody = new THREE.Mesh(
    new THREE.CylinderGeometry(pOd / 2, pOd / 2, pLen, 20),
    motorMat,
  );
  pBody.rotation.z = Math.PI / 2;
  pBody.position.set(px, py, pz);
  g.add(pBody);
  const pDisk = propDisk(pD, 0.28);
  pDisk.rotation.y = Math.PI / 2;
  pDisk.position.set(px + pLen / 2 + 0.01, py, pz);
  g.add(pDisk);

  return g;
}

/** +1 = right / starboard skin, −1 = left / port skin. */
function coolingSideSign(sideName, fallback = 'left') {
  return String(sideName || fallback).toLowerCase() === 'right' ? 1 : -1;
}

/** NACA-style side-inlet ramp face: flush at −x, full recess depth at +x. */
function nacaRampGeometry(length, height, recess, outSign) {
  const hx = 0.5 * length;
  const hz = 0.5 * height;
  const dy = -outSign * recess; // inward (toward fuselage centreline)
  const positions = new Float32Array([
    // ramp face
    -hx, 0, -hz, hx, dy, -hz, hx, dy, hz, -hx, 0, hz,
    // −z wall
    -hx, 0, -hz, hx, 0, -hz, hx, dy, -hz,
    // +z wall
    -hx, 0, hz, hx, dy, hz, hx, 0, hz,
    // aft face
    hx, 0, -hz, hx, 0, hz, hx, dy, hz, hx, dy, -hz,
  ]);
  const indices = [
    0, 1, 2, 0, 2, 3,
    4, 5, 6,
    7, 8, 9,
    10, 11, 12, 10, 12, 13,
  ];
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return geo;
}

/**
 * Cross-fuselage IE-SOAR cooling: NACA-style flush inlet + fan-side exhaust grille.
 * Driven entirely by fc_cooling_* CAD parameters (drawing geometry only).
 */
function coolingMeshes() {
  const g = new THREE.Group();
  g.name = 'cooling';
  if (P.fc_cooling_inlet_panel_length == null) return g;

  const halfW = 0.5 * mm('fuselage_width', mm('fuselage_max_diameter', 180));
  const skinEps = 0.0012;
  const inletMat = makeMaterial(COLORS.coolingInlet, {
    emissive: COLORS.coolingInlet,
    emissiveIntensity: 0.12,
    roughness: 0.4,
    metalness: 0.12,
  });
  const exhaustMat = makeMaterial(COLORS.coolingExhaust, {
    emissive: COLORS.coolingExhaust,
    emissiveIntensity: 0.12,
    roughness: 0.42,
    metalness: 0.1,
  });
  const voidMat = makeMaterial(0x0a1218, { roughness: 0.95, metalness: 0.0 });
  const fanMat = makeMaterial(0x2a3340, { metalness: 0.35, roughness: 0.35 });
  const fanDiskMat = makeMaterial(COLORS.coolingExhaust, {
    transparent: true,
    opacity: 0.35,
    emissive: COLORS.coolingExhaust,
    emissiveIntensity: 0.2,
    side: THREE.DoubleSide,
  });
  fanDiskMat.userData.baseOpacity = 0.35;
  const plenumMat = makeMaterial(0x7ec8ff, {
    transparent: true,
    opacity: 0.18,
    depthWrite: false,
    roughness: 1,
  });
  plenumMat.userData.baseOpacity = 0.18;

  function panelPlate(cx, cy, cz, length, height, colorMat, outSign) {
    const t = 0.0035; // frame web thickness
    const y = cy + outSign * skinEps;
    // Open frame so grille slots remain readable.
    const webs = [
      [length, 0.002, t], // bottom
      [length, 0.002, t], // top
      [t, 0.002, height], // forward
      [t, 0.002, height], // aft
    ];
    const zs = [-0.5 * height + 0.5 * t, 0.5 * height - 0.5 * t, 0, 0];
    const xs = [0, 0, -0.5 * length + 0.5 * t, 0.5 * length - 0.5 * t];
    for (let i = 0; i < 4; i++) {
      const [dx, dy, dz] = webs[i];
      const web = new THREE.Mesh(new THREE.BoxGeometry(dx, dy, dz), colorMat);
      web.position.set(cx + xs[i], y, cz + zs[i]);
      g.add(web);
    }
    const edge = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(length, 0.0022, height)),
      new THREE.LineBasicMaterial({ color: colorMat.color, transparent: true, opacity: 0.9 }),
    );
    edge.position.set(cx, y, cz);
    g.add(edge);
  }

  function stackedSlots(cx, ySkin, cz, n, L, w, gap, mat, depthY, outSign, louvreDeg = 0) {
    const totalH = n * w + Math.max(0, n - 1) * gap;
    const louvreRad = (louvreDeg * Math.PI) / 180;
    const d = Math.max(depthY, 0.002);
    for (let i = 0; i < n; i++) {
      const z = cz - 0.5 * totalH + 0.5 * w + i * (w + gap);
      const slot = new THREE.Mesh(new THREE.BoxGeometry(L, d, w * 0.92), mat);
      // Sit proud of the skin so openings remain visible with an opaque fuselage.
      slot.position.set(cx, ySkin + outSign * (0.5 * d + skinEps), z);
      if (louvreRad) slot.rotation.x = outSign * louvreRad;
      g.add(slot);
    }
  }

  function addPlenum(outSign, panelL, panelH, cx, cz, gapKey) {
    const gap = mm(gapKey, 0);
    if (!(gap > 1e-4) || P.fuel_cell_width == null) return;
    const fcW = mm('fuel_cell_width');
    const fcL = mm('fuel_cell_length', panelL * 1000);
    const fcH = mm('fuel_cell_height', panelH * 1000);
    const fcY = mm('fuel_cell_y', 0);
    const fcX = mm('fuel_cell_x', cx * 1000);
    const fcZ = mm('fuel_cell_z', cz * 1000);
    const faceY = fcY + outSign * 0.5 * fcW;
    const plenum = new THREE.Mesh(
      new THREE.BoxGeometry(Math.min(fcL, panelL), gap, Math.min(fcH, panelH)),
      plenumMat,
    );
    plenum.position.set(fcX, faceY + outSign * 0.5 * gap, fcZ);
    g.add(plenum);
  }

  // --- Inlet (NACA-style flush side opening) ---
  {
    const side = str('fc_cooling_inlet_side', 'left');
    const outSign = coolingSideSign(side, 'left');
    const cx = mm('fc_cooling_inlet_center_x');
    const cy = mm('fc_cooling_inlet_center_y', outSign * halfW * 1000);
    const cz = mm('fc_cooling_inlet_center_z');
    const panelL = mm('fc_cooling_inlet_panel_length');
    const panelH = mm('fc_cooling_inlet_panel_height');
    const recess = Math.max(mm('fc_cooling_inlet_recess_depth', 8), 0.002);
    const rampL = Math.max(mm('fc_cooling_inlet_ramp_length', panelL * 1000), 0.01);
    const sidewallDeg = num('fc_cooling_inlet_sidewall_angle', 7);
    const n = Math.max(1, Math.round(num('fc_cooling_inlet_slot_count', 1)));
    const slotL = mm('fc_cooling_inlet_slot_length', panelL * 1000);
    const slotW = mm('fc_cooling_inlet_slot_width', 6);
    const slotGap = mm('fc_cooling_inlet_slot_gap', 3);

    panelPlate(cx, cy, cz, panelL, panelH, inletMat, outSign);

    const cavity = new THREE.Mesh(
      new THREE.BoxGeometry(panelL * 0.98, recess, panelH * 0.96),
      voidMat,
    );
    cavity.position.set(cx, cy - outSign * (0.5 * recess + skinEps * 0.5), cz);
    g.add(cavity);

    const ramp = new THREE.Mesh(nacaRampGeometry(rampL, panelH * 0.94, recess, outSign), inletMat);
    ramp.position.set(cx, cy - outSign * skinEps, cz);
    g.add(ramp);

    const wallAng = (sidewallDeg * Math.PI) / 180;
    for (const sz of [-1, 1]) {
      const wall = new THREE.Mesh(
        new THREE.BoxGeometry(rampL * 0.9, recess * 0.95, 0.0015),
        inletMat,
      );
      wall.position.set(cx, cy - outSign * 0.45 * recess, cz + sz * (0.5 * panelH - 0.002));
      wall.rotation.x = sz * wallAng;
      g.add(wall);
    }

    stackedSlots(cx, cy, cz, n, slotL, slotW, slotGap, inletMat, 0.0025, outSign, 0);
    addPlenum(outSign, panelL, panelH, cx, cz, 'fc_cooling_inlet_plenum_gap_actual');
  }

  // --- Exhaust (flush fan-side grille + shrouded fans) ---
  {
    const side = str('fc_cooling_exhaust_side', 'right');
    const outSign = coolingSideSign(side, 'right');
    const cx = mm('fc_cooling_exhaust_center_x');
    const cy = mm('fc_cooling_exhaust_center_y', outSign * halfW * 1000);
    const cz = mm('fc_cooling_exhaust_center_z');
    const panelL = mm('fc_cooling_exhaust_panel_length');
    const panelH = mm('fc_cooling_exhaust_panel_height');
    const shroud = Math.max(mm('fc_cooling_exhaust_shroud_depth', 10), 0.002);
    const louvreDeg = num('fc_cooling_exhaust_louvre_angle', 15);
    const n = Math.max(1, Math.round(num('fc_cooling_exhaust_slot_count', 1)));
    const slotL = mm('fc_cooling_exhaust_slot_length', panelL * 1000);
    const slotW = mm('fc_cooling_exhaust_slot_width', 6.5);
    const slotGap = mm('fc_cooling_exhaust_slot_gap', 3);
    const fanCount = Math.max(1, Math.round(num('fc_cooling_fan_count', 2)));
    const fanD = mm('fc_cooling_fan_diameter', 80);
    const fanZ = mm('fc_cooling_fan_center_z', cz * 1000);
    const fanY = mm('fc_cooling_fan_center_y', cy * 1000);

    panelPlate(cx, cy, cz, panelL, panelH, exhaustMat, outSign);

    const duct = new THREE.Mesh(
      new THREE.BoxGeometry(panelL * 0.96, shroud, panelH * 0.94),
      voidMat,
    );
    duct.position.set(cx, cy - outSign * (0.5 * shroud + skinEps * 0.5), cz);
    g.add(duct);

    stackedSlots(cx, cy, cz, n, slotL, slotW, slotGap, exhaustMat, 0.003, outSign, louvreDeg);

    for (let i = 0; i < fanCount; i++) {
      const key = `fc_cooling_fan_${i + 1}_center_x`;
      const spacing = mm('fc_cooling_fan_center_spacing', 86);
      const fx = P[key] != null ? mm(key) : cx + (i - (fanCount - 1) / 2) * spacing;
      // Fans face the side wall (axis along ±Y): cylinder default is Y; disk/torus need XZ plane.
      const hub = new THREE.Mesh(
        new THREE.CylinderGeometry(fanD * 0.12, fanD * 0.12, shroud * 0.85, 16),
        fanMat,
      );
      hub.position.set(fx, fanY - outSign * 0.45 * shroud, fanZ);
      g.add(hub);

      const rim = new THREE.Mesh(
        new THREE.TorusGeometry(fanD * 0.48, fanD * 0.035, 8, 28),
        fanMat,
      );
      rim.rotation.x = Math.PI / 2;
      rim.position.set(fx, fanY - outSign * 0.15 * shroud, fanZ);
      g.add(rim);

      const disk = new THREE.Mesh(new THREE.CircleGeometry(fanD * 0.46, 28), fanDiskMat);
      disk.rotation.x = Math.PI / 2;
      disk.position.set(fx, fanY - outSign * 0.05 * shroud, fanZ);
      g.add(disk);
    }

    addPlenum(outSign, panelL, panelH, cx, cz, 'fc_cooling_exhaust_plenum_gap_actual');
  }

  return g;
}

/**
 * Wing primary/secondary spars (spanwise tubes at chord fractions) and
 * top/bottom fuselage longeron CFRP tubes (printed sleeve keepout is modelled, not drawn).
 */
function structureMeshes() {
  const g = new THREE.Group();
  g.name = 'structure';

  const sparMat = makeMaterial(COLORS.spar, { metalness: 0.22, roughness: 0.38 });
  const longMat = makeMaterial(COLORS.longeron, { metalness: 0.18, roughness: 0.42 });

  function addPolylineTube(pts, radius, material) {
    for (let i = 0; i < pts.length - 1; i++) {
      const mesh = tubeBetween(pts[i], pts[i + 1], radius, material);
      if (mesh) g.add(mesh);
    }
  }

  // --- Main-wing spars ---
  if (P.wing_primary_spar_OD != null || P.wing_secondary_spar_OD != null) {
    const le = mm('wing_le_x');
    const span = mm('wing_span');
    const rootChord = mm('wing_root_chord');
    const tipChord = mm('wing_tip_chord');
    const zWing = mm('wing_z', mm('lift_motor_z', 100));
    const sweep = (num('wing_sweep') * Math.PI) / 180;
    const dihedral = (num('wing_dihedral') * Math.PI) / 180;
    const half = Math.max(span / 2, 1e-6);
    const stations = 28;

    function sparStations(chordFrac, yAbsMax) {
      const pts = [];
      const ySpan = 2 * yAbsMax;
      for (let i = 0; i <= stations; i++) {
        const y = -yAbsMax + (i / stations) * ySpan;
        const chord = rootChord + (tipChord - rootChord) * (Math.abs(y) / half);
        const leX = le + Math.abs(y) * Math.tan(sweep);
        const z0 = zWing + Math.abs(y) * Math.tan(dihedral);
        pts.push([leX + chordFrac * chord, y, z0]);
      }
      return pts;
    }

    const primaryOd = mm('wing_primary_spar_OD', 13);
    const primaryFrac = num('wing_primary_spar_chord_fraction', 0.25);
    // Stop short of the geometric tip so the tube end-cap stays inside the tip rib
    // (full-span stations made the thick recommended spar poke outboard).
    if (primaryOd > 0) {
      const yMax = Math.max(half - Math.max(primaryOd / 2, 0.012), 0.5 * half);
      addPolylineTube(sparStations(primaryFrac, yMax), primaryOd / 2, sparMat);
    }

    const secondaryOd = mm('wing_secondary_spar_OD', 0);
    const secondaryFrac = num('wing_secondary_spar_chord_fraction', 0.65);
    if (secondaryOd > 0) {
      const yMax = Math.max(half - Math.max(secondaryOd / 2, 0.012), 0.5 * half);
      addPolylineTube(sparStations(secondaryFrac, yMax), secondaryOd / 2, sparMat);
    }
  }

  // --- Fuselage longerons (CFRP tubes only; printed sleeve is not drawn) ---
  if (P.fuselage_longeron_OD != null) {
    const fuseL = mm('fuselage_total_length', 780);
    const tubeOd = mm('fuselage_longeron_OD', 6.5);
    const zTop = mm('fuselage_longeron_top_z', 100);
    const zBot = mm('fuselage_longeron_bottom_z', -100);
    const tubeR = Math.max(tubeOd / 2, 0.001);

    for (const z of [zTop, zBot]) {
      const tube = new THREE.Mesh(
        new THREE.CylinderGeometry(tubeR, tubeR, fuseL, 16),
        longMat,
      );
      tube.rotation.z = Math.PI / 2;
      tube.position.set(0.5 * fuseL, 0, z);
      g.add(tube);
    }
  }

  return g;
}

/**
 * Tripod landing gear: CFRP tubes inside 2 mm printed sleeves.
 * One nose leg under the fuselage + two wing legs at the boom / primary-spar station.
 */
function landingGearMeshes() {
  const g = new THREE.Group();
  g.name = 'landing';
  if (num('landing_legs_enabled', 0) < 0.5) return g;

  const tubeOd = mm('landing_leg_tube_OD', 12);
  const sleeveOd = mm('landing_leg_sleeve_OD', tubeOd + 4);
  const tubeR = Math.max(tubeOd / 2, 0.001);
  const sleeveR = Math.max(sleeveOd / 2, tubeR + 0.001);
  const sleeveMat = makeMaterial(COLORS.landingSleeve, { metalness: 0.08, roughness: 0.72, transparent: true, opacity: 0.88 });
  const tubeMat = makeMaterial(COLORS.landingTube, { metalness: 0.22, roughness: 0.38 });
  const footMat = makeMaterial(COLORS.landingFoot, { metalness: 0.15, roughness: 0.55 });

  for (const name of ['nose', 'wing_left', 'wing_right']) {
    const x = mm(`landing_leg_${name}_x`);
    const y = mm(`landing_leg_${name}_y`);
    const mountZ = mm(`landing_leg_${name}_mount_z`);
    const footZ = mm(`landing_leg_${name}_foot_z`);
    const length = Math.max(mm(`landing_leg_${name}_length`, Math.abs(mountZ - footZ)), 0.02);
    const zMid = 0.5 * (mountZ + footZ);

    const sleeve = new THREE.Mesh(
      new THREE.CylinderGeometry(sleeveR, sleeveR, length, 18),
      sleeveMat,
    );
    sleeve.rotation.x = Math.PI / 2;
    sleeve.position.set(x, y, zMid);
    g.add(sleeve);

    const tube = new THREE.Mesh(
      new THREE.CylinderGeometry(tubeR, tubeR, Math.max(length - 0.004, 0.015), 14),
      tubeMat,
    );
    tube.rotation.x = Math.PI / 2;
    tube.position.set(x, y, zMid);
    g.add(tube);

    const foot = new THREE.Mesh(
      new THREE.CylinderGeometry(sleeveR * 1.55, sleeveR * 1.55, Math.max(sleeveOd * 0.35, 0.006), 16),
      footMat,
    );
    foot.rotation.x = Math.PI / 2;
    foot.position.set(x, y, footZ + Math.max(sleeveOd * 0.18, 0.003));
    g.add(foot);
  }
  return g;
}

function buildAircraft() {
  if (aircraft) scene.remove(aircraft);
  aircraft = new THREE.Group();
  aircraft.name = 'aircraft';

  groups.fuselage = fuselageMesh();
  groups.wing = horizontalSurface({
    le: mm('wing_le_x'),
    span: mm('wing_span'),
    rootChord: mm('wing_root_chord'),
    tipChord: mm('wing_tip_chord'),
    z: mm('wing_z', mm('lift_motor_z', 100)),
    thickness: mm('wing_thickness', 28),
    naca: str('wing_NACA', 'NACA4412'),
    sweepDeg: num('wing_sweep'),
    dihedralDeg: num('wing_dihedral'),
    twistRootDeg: num('wing_twist_root'),
    twistTipDeg: num('wing_twist_tip'),
    color: COLORS.wing,
  });
  groups.wing.name = 'wing';

  groups.booms = boomMeshes();
  groups.mounts = mountTubeMeshes();

  groups.ht = horizontalSurface({
    le: mm('horizontal_tail_le_x'),
    span: mm('horizontal_tail_span'),
    rootChord: mm('horizontal_tail_root_chord'),
    tipChord: mm('horizontal_tail_tip_chord'),
    z: mm('horizontal_tail_z', 30),
    thickness: mm('horizontal_tail_thickness', 22),
    naca: str('horizontal_tail_NACA', 'NACA0012'),
    color: COLORS.tail,
  });
  groups.ht.name = 'ht';

  const vtGroup = new THREE.Group();
  vtGroup.name = 'vt';
  const boomYs = [mm('boom_left_y', -350), mm('boom_right_y', 350)];
  const boomZ = mm('boom_z', mm('wing_z', 100));
  for (const y of boomYs) {
    vtGroup.add(
      verticalFin({
        le: mm('vertical_tail_le_x'),
        y,
        height: mm('vertical_tail_height'),
        rootChord: mm('vertical_tail_root_chord'),
        tipChord: mm('vertical_tail_tip_chord'),
        thickness: mm('vertical_tail_thickness', 12),
        naca: str('vertical_tail_NACA', 'NACA0012'),
        color: COLORS.tail,
        z0: boomZ,
      }),
    );
  }
  groups.vt = vtGroup;
  groups.props = propulsionMeshes();
  groups.internals = internalPackMeshes();
  groups.cooling = coolingMeshes();
  groups.structure = structureMeshes();
  groups.landing = landingGearMeshes();

  for (const key of ['fuselage', 'wing', 'booms', 'mounts', 'ht', 'vt', 'props', 'internals', 'cooling', 'structure', 'landing']) {
    aircraft.add(groups[key]);
  }
  scene.add(aircraft);
  applyVisibility();
  applyMaterials();
  updateDims();
  fitCamera();
}

function fmt(m) {
  if (m >= 1) return `${m.toFixed(3)} m`;
  return `${(m * 1000).toFixed(1)} mm`;
}

function updateDims() {
  const rows = [
    ['Fuselage L × W × H', `${fmt(mm('fuselage_total_length'))} × ${fmt(mm('fuselage_width', mm('fuselage_max_diameter')))} × ${fmt(mm('fuselage_height', mm('fuselage_max_diameter')))} (${str('fuselage_cross_section', 'rectangular_filleted')})`],
    ['Wing span', fmt(mm('wing_span'))],
    ['Wing root / tip', `${fmt(mm('wing_root_chord'))} / ${fmt(mm('wing_tip_chord'))}`],
    ['Wing airfoil', str('wing_NACA', '—')],
    ['Boom spacing', fmt(mm('boom_spacing'))],
    ['Boom length', fmt(mm('boom_length'))],
    ['HT span', fmt(mm('horizontal_tail_span'))],
    ['VT height (each)', fmt(mm('vertical_tail_height'))],
    ['Lift prop ⌀', fmt(mm('lift_prop_diameter'))],
    ['Pusher prop ⌀', fmt(mm('pusher_prop_diameter'))],
  ];
  if (P.wing_primary_spar_OD != null) {
    rows.push([
      'Wing spars ⌀',
      `pri ${fmt(mm('wing_primary_spar_OD'))} @ ${(100 * num('wing_primary_spar_chord_fraction', 0.25)).toFixed(0)}%c` +
        (P.wing_secondary_spar_OD != null
          ? ` · sec ${fmt(mm('wing_secondary_spar_OD'))} @ ${(100 * num('wing_secondary_spar_chord_fraction', 0.65)).toFixed(0)}%c`
          : ''),
    ]);
  }
  if (P.fuselage_longeron_OD != null) {
    rows.push([
      'Fuselage longerons',
      `tube ⌀ ${fmt(mm('fuselage_longeron_OD'))} · z ${fmt(mm('fuselage_longeron_top_z'))}/${fmt(mm('fuselage_longeron_bottom_z'))}`,
    ]);
  }
  if (num('landing_legs_enabled', 0) >= 0.5) {
    rows.push([
      'Landing legs',
      `tube ⌀ ${fmt(mm('landing_leg_tube_OD'))} · sleeve ⌀ ${fmt(mm('landing_leg_sleeve_OD'))} (2 mm wall) · ground z ${fmt(mm('landing_ground_z'))}`,
    ]);
  }
  if (P.fc_cooling_inlet_panel_length != null) {
    rows.push(['FC cooling arch.', str('fc_cooling_architecture', 'cross_fuselage')]);
    rows.push([
      'FC cooling inlet',
      `${fmt(mm('fc_cooling_inlet_panel_length'))} × ${fmt(mm('fc_cooling_inlet_panel_height'))} · ${str('fc_cooling_inlet_side', 'left')} · ${Math.round(num('fc_cooling_inlet_slot_count', 0))} slots · recess ${fmt(mm('fc_cooling_inlet_recess_depth'))}`,
    ]);
    rows.push([
      'FC cooling exhaust',
      `${fmt(mm('fc_cooling_exhaust_panel_length'))} × ${fmt(mm('fc_cooling_exhaust_panel_height'))} · ${str('fc_cooling_exhaust_side', 'right')} · louvre ${num('fc_cooling_exhaust_louvre_angle', 0).toFixed(0)}° · shroud ${fmt(mm('fc_cooling_exhaust_shroud_depth'))}`,
    ]);
    rows.push([
      'FC fans',
      `${Math.round(num('fc_cooling_fan_count', 2))} × ⌀ ${fmt(mm('fc_cooling_fan_diameter'))} @ ${fmt(mm('fc_cooling_fan_center_spacing'))} spacing`,
    ]);
    rows.push([
      'FC plenum gaps (act/min)',
      `in ${fmt(mm('fc_cooling_inlet_plenum_gap_actual'))}/${fmt(mm('fc_cooling_inlet_plenum_gap_min'))} · ex ${fmt(mm('fc_cooling_exhaust_plenum_gap_actual'))}/${fmt(mm('fc_cooling_exhaust_plenum_gap_min'))}`,
    ]);
  }
  if (P.payload_length != null) {
    rows.push(['Payload L×W×H', `${fmt(mm('payload_length'))} × ${fmt(mm('payload_width', 80))} × ${fmt(mm('payload_height', 80))}`]);
  }
  if (P.fuel_cell_length != null) {
    rows.push(['Fuel cell L×W×H', `${fmt(mm('fuel_cell_length'))} × ${fmt(mm('fuel_cell_width'))} × ${fmt(mm('fuel_cell_height'))}`]);
  }
  if (P.battery_length != null) {
    rows.push(['Battery L×W×H', `${fmt(mm('battery_length'))} × ${fmt(mm('battery_width'))} × ${fmt(mm('battery_height'))}`]);
  }
  if (P.hydrogen_cylinder_length != null) {
    rows.push(['H₂ cylinder L×⌀', `${fmt(mm('hydrogen_cylinder_length'))} × ${fmt(mm('hydrogen_cylinder_diameter'))}`]);
  }
  const dl = document.getElementById('dim-list');
  dl.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}

function fitCamera() {
  const box = new THREE.Box3().setFromObject(aircraft);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const span = Math.max(size.x, size.y, size.z, 1);
  controls.target.copy(center);
  camera.position.set(center.x - span * 0.55, center.y - span * 0.95, center.z + span * 0.45);
  controls.update();
}

function applyVisibility() {
  const map = {
    fuselage: 'toggle-fuselage',
    wing: 'toggle-wing',
    booms: 'toggle-booms',
    mounts: 'toggle-mounts',
    ht: 'toggle-ht',
    vt: 'toggle-vt',
    props: 'toggle-props',
    internals: 'toggle-internals',
    cooling: 'toggle-cooling',
    structure: 'toggle-structure',
    landing: 'toggle-landing',
  };
  for (const [g, id] of Object.entries(map)) {
    if (groups[g]) groups[g].visible = document.getElementById(id).checked;
  }
  grid.visible = document.getElementById('toggle-grid').checked;
  axes.visible = document.getElementById('toggle-axes').checked;

  // Auto-ghost the fuselage when internals are shown so packs remain visible.
  const showInternals = document.getElementById('toggle-internals').checked;
  const soft = document.getElementById('toggle-transparent').checked;
  if (groups.fuselage) {
    groups.fuselage.traverse((obj) => {
      if (!obj.isMesh || !obj.material) return;
      const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
      for (const m of mats) {
        const ghost = showInternals || soft;
        m.transparent = ghost;
        m.opacity = showInternals ? 0.28 : soft ? 0.72 : 1;
        m.depthWrite = !ghost;
        m.needsUpdate = true;
      }
    });
  }
}

function applyMaterials() {
  const wire = document.getElementById('toggle-wire').checked;
  const soft = document.getElementById('toggle-transparent').checked;
  const showInternals = document.getElementById('toggle-internals').checked;
  if (!aircraft) return;
  aircraft.traverse((obj) => {
    if (!obj.isMesh || !obj.material) return;
    const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    const inFuselage = obj.parent === groups.fuselage || (obj.parent && obj.parent.parent === groups.fuselage);
    const inPropsDisk = obj.parent?.name === 'props' || obj.parent?.parent?.name === 'props';
    for (const m of mats) {
      m.wireframe = wire;
      if (inPropsDisk && m.transparent && m.opacity < 0.5) {
        continue;
      }
      if (inFuselage && showInternals) {
        m.transparent = true;
        m.opacity = 0.28;
        m.depthWrite = false;
      } else {
        m.transparent = soft || (m.opacity < 1 && m.transparent);
        m.opacity = soft ? Math.min(m.opacity || 1, 0.72) : m.userData?.baseOpacity || (m.transparent ? m.opacity : 1);
        if (!soft && !m.transparent) m.opacity = 1;
        m.depthWrite = !m.transparent;
      }
      m.needsUpdate = true;
    }
  });
  applyVisibility();
}

function setSourceLabel(text) {
  document.getElementById('source-label').textContent = text;
}

async function loadCad(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Failed to load ${url}`);
  const doc = await res.json();
  P = paramsFromCad(doc);
  setSourceLabel(`${doc.model_version || 'CAD'} · ${doc.schema || 'parameters'}`);
  buildAircraft();
}

function loadCadObject(doc, label) {
  P = paramsFromCad(doc);
  setSourceLabel(label || `${doc.model_version || 'CAD'} · loaded file`);
  buildAircraft();
}

function onResize() {
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
  renderer.setSize(w, h, false);
}

for (const id of [
  'toggle-fuselage',
  'toggle-wing',
  'toggle-booms',
  'toggle-mounts',
  'toggle-ht',
  'toggle-vt',
  'toggle-props',
  'toggle-internals',
  'toggle-cooling',
  'toggle-structure',
  'toggle-landing',
  'toggle-grid',
  'toggle-axes',
]) {
  document.getElementById(id).addEventListener('change', applyVisibility);
}
document.getElementById('toggle-wire').addEventListener('change', applyMaterials);
document.getElementById('toggle-transparent').addEventListener('change', applyMaterials);
document.getElementById('btn-reset').addEventListener('click', fitCamera);
document.getElementById('file-input').addEventListener('change', async (ev) => {
  const file = ev.target.files?.[0];
  if (!file) return;
  const doc = JSON.parse(await file.text());
  loadCadObject(doc, file.name);
});

window.addEventListener('resize', onResize);
onResize();

(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

const qs = new URLSearchParams(location.search);
const cadUrl = qs.get('cad') || './cad_parameters.json';
loadCad(cadUrl).catch((err) => {
  setSourceLabel('No CAD loaded — use “Load CAD JSON” or start serve.py with a run folder.');
  console.warn(err);
});
