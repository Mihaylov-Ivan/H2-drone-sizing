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
 * Single closed surface-of-revolution for the central pod.
 * Section diameters from CAD are lofted; named nose/tail sections get
 * elliptical fairings even when the CAD diameter is constant (this design
 * stores mass-section max diameters, not a full body of revolution).
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

  // Sample radius along fuselage centreline (piecewise + fairings).
  function radiusAt(x) {
    // Find owning section
    let sec = sections[sections.length - 1];
    for (const s of sections) {
      if (x <= s.end + 1e-9) {
        sec = s;
        break;
      }
    }
    const R = sec.dia / 2;
    const u = (x - sec.start) / Math.max(sec.end - sec.start, 1e-9); // 0..1 in section

    if (sec.name === 'nose') {
      // Prolate half-ellipse from tip (r=0) to section max radius.
      const t = Math.min(Math.max(u, 0), 1);
      return R * Math.sqrt(Math.max(0, 1 - (1 - t) * (1 - t)));
    }
    if (sec.name === 'tail') {
      // Aft boat-tail: ellipse from full radius down to ~18% tip (central pod, not a single boom).
      const t = Math.min(Math.max(u, 0), 1);
      const rTip = Math.max(0.18 * R, 0.008);
      return rTip + (R - rTip) * Math.sqrt(Math.max(0, 1 - t * t));
    }
    // Midbody: linear loft between adjacent section diameters at station ends.
    const idx = sections.indexOf(sec);
    const rA = sections[Math.max(0, idx - 1)].dia / 2;
    const rB = R;
    const rC = sections[Math.min(sections.length - 1, idx + 1)].dia / 2;
    // Smoothstep blend of neighbouring radii across the section.
    const rStart = 0.5 * (rA + rB);
    const rEnd = 0.5 * (rB + rC);
    const s = u * u * (3 - 2 * u);
    return rStart + (rEnd - rStart) * s;
  }

  const axial = 64;
  const radial = 48;
  const positions = [];
  const indices = [];
  const tipEps = 1e-4;

  for (let i = 0; i <= axial; i++) {
    const t = i / axial;
    const x = x0 + t * length;
    let r = radiusAt(x);
    if (i === 0) r = tipEps; // close nose to a point
    if (i === axial) r = Math.max(radiusAt(x1 - 1e-6) * 0.02, tipEps); // close aft tip
    for (let j = 0; j < radial; j++) {
      const a = (j / radial) * Math.PI * 2;
      positions.push(x, r * Math.cos(a), r * Math.sin(a));
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

  // Cap aft tip with a tiny fan so the boat-tail is closed.
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
  // Keep a faint reference of max diameter for scale checks (invisible to picking).
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
 * Vertical fin in the x–z plane at lateral station y.
 */
function verticalFin({ le, y, height, rootChord, tipChord, thickness, naca, color }) {
  const profile = nacaThicknessPoints(naca, 24);
  const stations = 12;
  const positions = [];
  const indices = [];
  const tScaleRoot = thickness / (2 * Math.max(...profile.map((p) => p.yt)) * rootChord);

  for (let i = 0; i <= stations; i++) {
    const eta = i / stations;
    const z = eta * height;
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
  const ys = [mm('boom_left_y', -350), mm('boom_right_y', 350)];
  const mat = makeMaterial(COLORS.boom);
  for (const y of ys) {
    const geo = new THREE.CylinderGeometry(r, r, L, 20);
    geo.rotateZ(Math.PI / 2);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set((x0 + x1) / 2, y, 0);
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
 * Structural tubes that carry the lift motors and pusher.
 * Layout matches the twin-boom / wing-mount screening geometry:
 * outboard longitudinal rails at the lift stations, lateral links to the
 * main booms, forward boom stubs to the front row, and an aft pusher stalk.
 */
function mountTubeMeshes() {
  const g = new THREE.Group();
  g.name = 'mounts';
  const mat = makeMaterial(COLORS.boom);
  const boomR = mm('boom_OD', 21) / 2;
  const armR = Math.max(boomR * 0.75, mm('wing_secondary_spar_OD', 10) / 2);
  const pusherR = Math.max(armR * 0.9, 0.006);

  const frontX = mm('lift_motor_front_x');
  const rearX = mm('lift_motor_rear_x');
  const yAbs = mm('lift_motor_y_abs');
  const zMot = mm('lift_motor_z', mm('wing_z', 100));
  const boomYs = [mm('boom_left_y', -350), mm('boom_right_y', 350)];
  const boomX0 = mm('boom_start_x');

  for (let i = 0; i < 2; i++) {
    const sign = i === 0 ? -1 : 1;
    const yMot = sign * yAbs;
    const yBoom = boomYs[i];

    // Outboard rail holding front and rear lift motors.
    const rail = tubeBetween([frontX, yMot, zMot], [rearX, yMot, zMot], armR, mat);
    if (rail) g.add(rail);

    // Forward boom stub from main-boom start up to the front motor station.
    const fwd = tubeBetween([boomX0, yBoom, 0], [frontX, yBoom, zMot * 0.35], boomR * 0.85, mat);
    if (fwd) g.add(fwd);
    const fwdUp = tubeBetween([frontX, yBoom, zMot * 0.35], [frontX, yMot, zMot], armR, mat);
    if (fwdUp) g.add(fwdUp);

    // Lateral / vertical links from twin boom onto the outboard rail at each row.
    for (const x of [frontX, rearX]) {
      const boomX = Math.min(Math.max(x, boomX0), mm('boom_end_x', rearX));
      const link = tubeBetween([boomX, yBoom, 0], [x, yMot, zMot], armR, mat);
      if (link) g.add(link);
    }
  }

  // Pusher mount stalk: fuselage aft → pusher motor centreline.
  const fuseEnd = mm('fuselage_total_length', 780);
  const px = mm('pusher_motor_x');
  const py = mm('pusher_motor_y');
  const pz = mm('pusher_motor_z');
  const stalk = tubeBetween([fuseEnd, 0, 0], [px, py, pz], pusherR, mat);
  if (stalk) g.add(stalk);

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
      }),
    );
  }
  groups.vt = vtGroup;
  groups.props = propulsionMeshes();

  for (const key of ['fuselage', 'wing', 'booms', 'mounts', 'ht', 'vt', 'props']) {
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
    ['Fuselage L × ⌀', `${fmt(mm('fuselage_total_length'))} × ${fmt(mm('fuselage_max_diameter'))}`],
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
  };
  for (const [g, id] of Object.entries(map)) {
    if (groups[g]) groups[g].visible = document.getElementById(id).checked;
  }
  grid.visible = document.getElementById('toggle-grid').checked;
  axes.visible = document.getElementById('toggle-axes').checked;
}

function applyMaterials() {
  const wire = document.getElementById('toggle-wire').checked;
  const soft = document.getElementById('toggle-transparent').checked;
  if (!aircraft) return;
  aircraft.traverse((obj) => {
    if (!obj.isMesh || !obj.material) return;
    const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
    for (const m of mats) {
      m.wireframe = wire;
      if (obj.parent?.name === 'props' && m.transparent && m.opacity < 0.5) {
        continue;
      }
      m.transparent = soft;
      m.opacity = soft ? 0.72 : 1;
      m.depthWrite = !soft;
      m.needsUpdate = true;
    }
  });
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
