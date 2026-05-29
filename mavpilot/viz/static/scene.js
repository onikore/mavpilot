// 3D scene: three.js setup, drone model, trail, command visualizations.
// initScene() builds everything and returns a frame(drone, activeCommand,
// marker, trailData) callback driven by main.js's animation loop.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const TRAIL_MAX = 600;
const TRAIL_MS = 30000;
const ARROW_POOL = 24;

// NED → Three.js: ned_x=N → -Z, ned_y=E → +X, ned_z=D → -Y.
function nedToThree(x, y, z) {
  return new THREE.Vector3(y, -z, -x);
}

export function initScene(stage, loading) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0d1a);
  scene.fog = new THREE.Fog(0x0a0d1a, 60, 250);

  const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 1000);
  camera.position.set(18, 14, 18);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  stage.appendChild(renderer.domElement);
  if (loading) loading.style.display = 'none';

  function resize() {
    const w = stage.clientWidth;
    const h = stage.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  resize();
  window.addEventListener('resize', resize);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 3, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = Math.PI * 0.49;
  controls.minDistance = 2;
  controls.maxDistance = 200;

  // Lights
  scene.add(new THREE.AmbientLight(0x6080a0, 1.2));
  const dirLight = new THREE.DirectionalLight(0xffffff, 1.4);
  dirLight.position.set(25, 50, 15);
  scene.add(dirLight);
  scene.add(new THREE.HemisphereLight(0x80a0ff, 0x202030, 0.5));

  // Ground + grid + NED axes
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(400, 400),
    new THREE.MeshStandardMaterial({ color: 0x0c1228, roughness: 0.92, metalness: 0.05 })
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -0.02;
  scene.add(ground);

  const grid = new THREE.GridHelper(400, 80, 0x4a5a8a, 0x1c2444);
  grid.material.transparent = true;
  grid.material.opacity = 0.6;
  scene.add(grid);

  const grid5 = new THREE.GridHelper(400, 16, 0x6a7aa8, 0x6a7aa8);
  grid5.material.transparent = true;
  grid5.material.opacity = 0.3;
  scene.add(grid5);

  const axisN = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(), 4, 0xff5050, 0.6, 0.3);
  const axisE = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 4, 0x50ff50, 0.6, 0.3);
  const axisD = new THREE.ArrowHelper(new THREE.Vector3(0, -1, 0), new THREE.Vector3(0, 0.05, 0), 2, 0x5080ff, 0.4, 0.25);
  scene.add(axisN, axisE, axisD);

  // Home marker (cross at origin)
  const homeMat = new THREE.LineBasicMaterial({ color: 0x6a8cd5 });
  const homeGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-0.3, 0.02, 0), new THREE.Vector3(0.3, 0.02, 0),
    new THREE.Vector3(0, 0.02, -0.3), new THREE.Vector3(0, 0.02, 0.3),
  ]);
  const home = new THREE.LineSegments(homeGeo, homeMat);
  scene.add(home);

  // Drone mesh
  const droneGroup = new THREE.Group();

  const bodyMatArmed = new THREE.MeshStandardMaterial({
    color: 0x4dd07e, metalness: 0.4, roughness: 0.35,
    emissive: 0x10401a, emissiveIntensity: 0.3,
  });
  const bodyMatDisarmed = new THREE.MeshStandardMaterial({
    color: 0x7a8caf, metalness: 0.4, roughness: 0.5,
  });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.12, 0.4), bodyMatDisarmed);
  droneGroup.add(body);

  const top = new THREE.Mesh(
    new THREE.BoxGeometry(0.25, 0.08, 0.25),
    new THREE.MeshStandardMaterial({ color: 0x222a40, metalness: 0.6, roughness: 0.3 })
  );
  top.position.y = 0.1;
  droneGroup.add(top);

  const armMat = new THREE.MeshStandardMaterial({ color: 0x3a4a6a, metalness: 0.5, roughness: 0.4 });
  for (const angle of [Math.PI / 4, -Math.PI / 4]) {
    const arm = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.05, 0.06), armMat);
    arm.rotation.y = angle;
    droneGroup.add(arm);
  }

  const rotorPositions = [
    new THREE.Vector3(0.5, 0.04, 0.5),
    new THREE.Vector3(0.5, 0.04, -0.5),
    new THREE.Vector3(-0.5, 0.04, 0.5),
    new THREE.Vector3(-0.5, 0.04, -0.5),
  ];
  const rotors = [];
  const rotorBladeMat = new THREE.MeshBasicMaterial({
    color: 0xffffff, transparent: true, opacity: 0.18, side: THREE.DoubleSide,
  });
  for (const pos of rotorPositions) {
    const cap = new THREE.Mesh(
      new THREE.CylinderGeometry(0.07, 0.07, 0.04, 12),
      new THREE.MeshStandardMaterial({ color: 0x222, metalness: 0.5, roughness: 0.5 })
    );
    cap.position.copy(pos);
    droneGroup.add(cap);

    const blade = new THREE.Mesh(new THREE.CircleGeometry(0.32, 12), rotorBladeMat);
    blade.position.copy(pos);
    blade.position.y += 0.04;
    blade.rotation.x = -Math.PI / 2;
    droneGroup.add(blade);
    rotors.push(blade);
  }

  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.09, 0.22, 12),
    new THREE.MeshStandardMaterial({ color: 0xff3030, emissive: 0xa00000, emissiveIntensity: 0.6 })
  );
  nose.position.set(0, 0, -0.4);
  nose.rotation.x = -Math.PI / 2;
  droneGroup.add(nose);

  scene.add(droneGroup);

  // Altitude line (drone → ground)
  const altLineGeo = new THREE.BufferGeometry();
  altLineGeo.setAttribute('position', new THREE.Float32BufferAttribute([0,0,0, 0,0,0], 3));
  const altLineMat = new THREE.LineDashedMaterial({
    color: 0x4dd0e1, dashSize: 0.25, gapSize: 0.2, transparent: true, opacity: 0.45,
  });
  const altLine = new THREE.Line(altLineGeo, altLineMat);
  scene.add(altLine);

  // Ground shadow (circle under drone)
  const shadow = new THREE.Mesh(
    new THREE.CircleGeometry(0.5, 24),
    new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.4 })
  );
  shadow.rotation.x = -Math.PI / 2;
  shadow.position.y = 0.005;
  scene.add(shadow);

  // Trail (fading line)
  const trailGeo = new THREE.BufferGeometry();
  const trailPositions = new Float32Array(TRAIL_MAX * 3);
  const trailColors = new Float32Array(TRAIL_MAX * 3);
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  trailGeo.setAttribute('color', new THREE.BufferAttribute(trailColors, 3));
  trailGeo.setDrawRange(0, 0);
  const trailMat = new THREE.LineBasicMaterial({
    vertexColors: true, transparent: true, linewidth: 2,
  });
  const trail = new THREE.Line(trailGeo, trailMat);
  scene.add(trail);

  // Target marker + path + marching arrows
  const targetMesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.3, 24, 24),
    new THREE.MeshStandardMaterial({
      color: 0xff8b3a, emissive: 0xff5500, emissiveIntensity: 0.7,
    })
  );
  targetMesh.visible = false;
  scene.add(targetMesh);

  const targetPillar = new THREE.Mesh(
    new THREE.CylinderGeometry(0.04, 0.04, 1, 8),
    new THREE.MeshBasicMaterial({ color: 0xff8b3a, transparent: true, opacity: 0.35 })
  );
  targetPillar.visible = false;
  scene.add(targetPillar);

  const pathGeo = new THREE.BufferGeometry();
  pathGeo.setAttribute('position', new THREE.Float32BufferAttribute([0,0,0, 0,0,0], 3));
  const pathMat = new THREE.LineDashedMaterial({
    color: 0x4080ff, dashSize: 0.5, gapSize: 0.35, transparent: true, opacity: 0.3,
  });
  const pathLine = new THREE.Line(pathGeo, pathMat);
  pathLine.visible = false;
  scene.add(pathLine);

  const arrowMeshes = [];
  for (let i = 0; i < ARROW_POOL; i++) {
    const m = new THREE.Mesh(
      new THREE.ConeGeometry(0.18, 0.55, 8),
      new THREE.MeshStandardMaterial({
        color: 0x4dc0ff,
        emissive: 0x2080ff,
        emissiveIntensity: 0.6,
        metalness: 0.2,
        roughness: 0.4,
      })
    );
    m.visible = false;
    arrowMeshes.push(m);
    scene.add(m);
  }

  const _upVec = new THREE.Vector3(0, 1, 0);
  const _tmpQuat = new THREE.Quaternion();

  function updateTrail(trailData) {
    const positions = trail.geometry.attributes.position.array;
    const colors = trail.geometry.attributes.color.array;
    const now = Date.now();
    for (let i = 0; i < trailData.length; i++) {
      const p = nedToThree(trailData[i].x, trailData[i].y, trailData[i].z);
      positions[i*3] = p.x;
      positions[i*3+1] = p.y;
      positions[i*3+2] = p.z;
      const age = (now - trailData[i].t) / TRAIL_MS;
      const alpha = Math.max(0.08, 1.0 - age);
      colors[i*3] = 0.6 * alpha;
      colors[i*3+1] = 0.85 * alpha;
      colors[i*3+2] = 1.0 * alpha;
    }
    trail.geometry.attributes.position.needsUpdate = true;
    trail.geometry.attributes.color.needsUpdate = true;
    trail.geometry.setDrawRange(0, trailData.length);
  }

  function renderCommand(dronePos, activeCommand, marker) {
    targetMesh.visible = false;
    targetPillar.visible = false;
    pathLine.visible = false;
    for (const a of arrowMeshes) a.visible = false;
    if (!activeCommand) return;
    const cmd = activeCommand.command;
    const now = Date.now();

    if ((cmd === 'goto' || cmd === 'rtl') && activeCommand.target) {
      const t = activeCommand.target;
      const targetPos = nedToThree(t.x, t.y, t.z);

      const pulse = 1 + Math.sin(now / 200) * 0.18;
      targetMesh.position.copy(targetPos);
      targetMesh.scale.set(pulse, pulse, pulse);
      targetMesh.material.color.setHex(0xff8b3a);
      targetMesh.material.emissive.setHex(0xff5500);
      targetMesh.visible = true;

      const pillarH = Math.abs(targetPos.y);
      if (pillarH > 0.1) {
        targetPillar.position.set(targetPos.x, targetPos.y / 2, targetPos.z);
        targetPillar.scale.y = pillarH;
        targetPillar.visible = true;
      }

      const posAttr = pathGeo.attributes.position;
      posAttr.setXYZ(0, dronePos.x, dronePos.y, dronePos.z);
      posAttr.setXYZ(1, targetPos.x, targetPos.y, targetPos.z);
      posAttr.needsUpdate = true;
      pathLine.computeLineDistances();
      pathLine.visible = true;

      const delta = new THREE.Vector3().subVectors(targetPos, dronePos);
      const dist = delta.length();
      if (dist > 1.0) {
        const count = Math.min(ARROW_POOL, Math.max(3, Math.floor(dist / 2.5)));
        const phase = (now / 1000) % 1;
        const dir = delta.clone().normalize();
        _tmpQuat.setFromUnitVectors(_upVec, dir);

        for (let i = 0; i < count; i++) {
          const tt = ((i + phase) / count) % 1;
          const p = dronePos.clone().lerp(targetPos, tt);
          const arrow = arrowMeshes[i];
          arrow.position.copy(p);
          arrow.quaternion.copy(_tmpQuat);
          const r = 0.3 + 0.6 * tt;
          const g = 0.85 - 0.4 * tt;
          const b = 1.0 - 0.7 * tt;
          arrow.material.color.setRGB(r, g, b);
          arrow.material.emissive.setRGB(r * 0.4, g * 0.3, b * 0.5);
          const pulseI = 0.4 + 0.6 * Math.sin(Math.PI * tt);
          arrow.material.emissiveIntensity = pulseI * 0.7 + 0.2;
          arrow.visible = true;
        }
      }
    } else if (cmd === 'takeoff') {
      const altitude = activeCommand.altitude_m || 5;
      const phase = (now / 1000) % 1;
      const count = 5;
      for (let i = 0; i < count && i < ARROW_POOL; i++) {
        const tt = ((i + phase) / count) % 1;
        const arrow = arrowMeshes[i];
        arrow.position.set(dronePos.x, dronePos.y + altitude * (1 - tt) - 0.3, dronePos.z);
        arrow.rotation.set(0, 0, 0);
        arrow.material.color.setHex(0x50dc82);
        arrow.material.emissive.setHex(0x20a050);
        arrow.material.emissiveIntensity = 0.5 + 0.5 * Math.sin(Math.PI * tt);
        arrow.visible = true;
      }
    } else if (cmd === 'land' || cmd === 'emergency_land') {
      const phase = (now / 1000) % 1;
      const count = 5;
      const totalH = Math.max(2, dronePos.y);
      const baseColor = cmd === 'emergency_land' ? 0xff3030 : 0xffb04a;
      const emColor = cmd === 'emergency_land' ? 0xa01010 : 0xc06010;
      for (let i = 0; i < count && i < ARROW_POOL; i++) {
        const tt = ((i + phase) / count) % 1;
        const arrow = arrowMeshes[i];
        arrow.position.set(dronePos.x, dronePos.y - totalH * tt + 0.3, dronePos.z);
        arrow.rotation.set(Math.PI, 0, 0);
        arrow.material.color.setHex(baseColor);
        arrow.material.emissive.setHex(emColor);
        arrow.material.emissiveIntensity = 0.5 + 0.5 * Math.sin(Math.PI * tt);
        arrow.visible = true;
      }
    } else if (cmd === 'precision_land' && marker && (now - marker.ts < 2500)) {
      const m = nedToThree(marker.ned.x, marker.ned.y, 0);
      const pulse = 1.0 + Math.sin(now / 150) * 0.35;
      targetMesh.position.copy(m);
      targetMesh.scale.set(pulse, pulse, pulse);
      targetMesh.material.color.setHex(0x4dd0e1);
      targetMesh.material.emissive.setHex(0x208090);
      targetMesh.visible = true;

      const posAttr = pathGeo.attributes.position;
      posAttr.setXYZ(0, dronePos.x, dronePos.y, dronePos.z);
      posAttr.setXYZ(1, m.x, m.y, m.z);
      posAttr.needsUpdate = true;
      pathLine.computeLineDistances();
      pathLine.material.color.setHex(0x4dd0e1);
      pathLine.visible = true;
    } else if (cmd === 'hover') {
      targetMesh.position.copy(dronePos);
      targetMesh.position.y -= 0.05;
      const pulse = 1.2 + Math.sin(now / 250) * 0.25;
      targetMesh.scale.set(pulse * 1.5, 0.3, pulse * 1.5);
      targetMesh.material.color.setHex(0xb0d0ff);
      targetMesh.material.emissive.setHex(0x4060a0);
      targetMesh.visible = true;
    }
  }

  let lastFrame = performance.now();

  function frame(drone, activeCommand, marker, trailData) {
    const now = performance.now();
    const dt = (now - lastFrame) / 1000;
    lastFrame = now;

    controls.update();
    updateTrail(trailData);

    const dronePos = nedToThree(drone.x, drone.y, drone.z);
    droneGroup.position.copy(dronePos);
    droneGroup.rotation.y = -drone.yaw;
    body.material = drone.armed ? bodyMatArmed : bodyMatDisarmed;

    if (drone.armed) {
      for (const r of rotors) r.rotation.z += dt * 80;
    }

    const altPos = altLine.geometry.attributes.position;
    altPos.setXYZ(0, dronePos.x, 0, dronePos.z);
    altPos.setXYZ(1, dronePos.x, dronePos.y, dronePos.z);
    altPos.needsUpdate = true;
    altLine.computeLineDistances();
    altLine.visible = dronePos.y > 0.1;

    shadow.position.x = dronePos.x;
    shadow.position.z = dronePos.z;
    const altRatio = Math.min(1, dronePos.y / 10);
    shadow.scale.set(1 + altRatio * 0.8, 1, 1 + altRatio * 0.8);
    shadow.material.opacity = 0.4 * (1 - altRatio * 0.35);

    renderCommand(dronePos, activeCommand, marker);
    renderer.render(scene, camera);
  }

  return { frame };
}
