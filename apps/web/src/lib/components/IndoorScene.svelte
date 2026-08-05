<script lang="ts">
  import { onMount } from 'svelte';
  import type { Attitude, ControlInputs, MissionPlanState, Pose } from '@electrode/sdk';
  import * as three from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import type { BufferGeometry, Group, Line, Material, Object3D, PerspectiveCamera, Scene, Vector3, WebGLRenderer } from 'three';
  import { loadVehicleRig, type VehicleKind, type VehicleRig } from '$lib/vehicle/vehicleRig';
  import {
    createSourceHalo,
    createSourceLabel,
    createSourceMarker,
    disposeTree,
    type SourceMarker
  } from '$lib/vehicle/sourceMarker';

  export let pose: Pose | null = null;
  export let attitude: Attitude | null = null;
  export let controls: ControlInputs | null = null;
  export let motors: number[] | null = null;
  export let mission: MissionPlanState | null = null;
  export let localizationQuality = 0;
  /**
   * Second pose/attitude drawn alongside the vehicle as a wire marker, so
   * telemetry and mocap can be compared in one view instead of by flipping
   * between them. Only meaningful when both are expressed in the same frame.
   */
  export let secondaryPose: Pose | null = null;
  export let secondaryAttitude: Attitude | null = null;
  /**
   * Naming the two sources turns the whole overlay on: an empty primary label
   * means there is only one source and nothing to disambiguate.
   */
  export let primaryLabel = '';
  export let secondaryLabel = '';
  export let primaryColor = '#fd7719';
  export let secondaryColor = '#35d0ff';
  export let theme: 'light' | 'dark' = 'dark';
  export let vehicleType: VehicleKind = 'fixedwing';

  // Local mocap metres are compressed into scene units for the room view. Keep
  // the vehicle on the same scale: fixed wing is 2 ft wingspan in local metres.
  const LOCAL_METERS_TO_SCENE = 0.09;
  const VEHICLE_FIT = 0.6096 * LOCAL_METERS_TO_SCENE;
  const MIN_ALTITUDE_SCENE_Y = 0.006;
  const CAMERA_TARGET = new three.Vector3(0, 0.24, 0);
  const CAMERA_START = new three.Vector3(1.15, 0.78, 1.25);
  // ENU/body yaw convention: yaw 0 faces +X (east), yaw +90 faces +Y (north).
  // The rig is authored/displayed with its nose along scene -Z, so rotate it
  // -90 deg at zero yaw to make the visual nose line up with +X.
  const YAW_ZERO_EAST_OFFSET_RAD = -Math.PI / 2;
  const MAX_TRAIL = 3600;

  // Follow mode: the orbit target eases onto the vehicle every frame and the
  // camera translates with it, preserving the user's orbit angle and zoom.
  const FOLLOW_LERP = 0.16;
  const followDelta = new three.Vector3();

  /** Ring buffer behind one flown path. */
  type Trail = { line: Line; positions: Float32Array; count: number };

  let followMode = false;
  let rig: VehicleRig | null = null;
  let rigLoadToken = 0;
  let trail: Trail | null = null;
  let mounted = false;

  // Source overlay: halo + label under the vehicle for the primary source, a
  // wire airframe with its own halo, label and trail for the secondary, and a
  // line between the two showing how far apart they are.
  let sourceGroup: Group | null = null;
  let sourceSignature = '';
  let secondaryMarker: SourceMarker | null = null;
  let primaryHalo: three.Mesh | null = null;
  let secondaryHalo: three.Mesh | null = null;
  let primaryLabelSprite: three.Sprite | null = null;
  let secondaryLabelSprite: three.Sprite | null = null;
  let offsetLine: Line | null = null;
  let secondaryTrail: Trail | null = null;
  const LABEL_LIFT_SCENE_Y = 0.08;
  const LABEL_SCALE: [number, number] = [0.17, 0.043];
  const HALO_RADIUS_SCENE = 0.055;
  const FLOOR_MARK_SCENE_Y = 0.008;

  let container: HTMLDivElement;
  let canvas: HTMLCanvasElement;
  let renderer: WebGLRenderer | null = null;
  let scene: Scene | null = null;
  let camera: PerspectiveCamera | null = null;
  let orbit: OrbitControls | null = null;
  let vehicleGroup: Group | null = null;
  let frameGroup: Group | null = null;
  let missionGroup: Group | null = null;
  let missionSignature = '';
  let resizeObserver: ResizeObserver | null = null;
  let animationFrame = 0;

  type ScenePalette = {
    bg: number;
    clear: number;
    floor: number;
    floorOpacity: number;
    gridCenter: number;
    gridLine: number;
    fineA: number;
    fineB: number;
    ink: number;
    forward: number;
    xAxis: number;
    up: number;
    labelX: string;
    labelY: string;
    labelZ: string;
    labelShadow: string;
  };

  function paletteFor(name: 'light' | 'dark'): ScenePalette {
    if (name === 'light') {
      return {
        bg: 0xeef1f3,
        clear: 0xeef1f3,
        floor: 0xf2f5f7,
        floorOpacity: 0.5,
        gridCenter: 0xe35f0c,
        gridLine: 0xbcc6cc,
        fineA: 0xccd4d9,
        fineB: 0xdee3e6,
        ink: 0x141a1f,
        forward: 0x141a1f,
        xAxis: 0xe35f0c,
        up: 0xc4831c,
        labelX: '#e35f0c',
        labelY: '#141a1f',
        labelZ: '#b0761a',
        labelShadow: '#ffffff'
      };
    }
    return {
      bg: 0x0a1113,
      clear: 0x091012,
      floor: 0x0d1718,
      floorOpacity: 0.62,
      gridCenter: 0xfd7719,
      gridLine: 0x203a3a,
      fineA: 0x1d5b55,
      fineB: 0x142827,
      ink: 0xe9fff9,
      forward: 0xf4fbf7,
      xAxis: 0xfd7719,
      up: 0xffc35a,
      labelX: '#fd7719',
      labelY: '#f4fbf7',
      labelZ: '#ffc35a',
      labelShadow: '#000000'
    };
  }

  let pal: ScenePalette = paletteFor('dark');

  $: localX = pose?.xM ?? 0;
  $: localY = pose?.yM ?? 0;
  $: localAlt = pose?.altM ?? 0;
  $: showSources = primaryLabel.trim().length > 0;
  $: updateVehicle(pose, attitude);
  $: updateMission(mission);
  $: applyTheme(theme);
  $: updateSourceOverlay(primaryLabel, secondaryLabel, primaryColor, secondaryColor, theme);
  $: updateSecondary(secondaryPose, secondaryAttitude);
  // Straight-line disagreement between the two sources: the number that says
  // whether the vehicle has actually converged onto mocap.
  $: separationM =
    showSources && pose && secondaryPose
      ? Math.hypot(pose.xM - secondaryPose.xM, pose.yM - secondaryPose.yM, pose.altM - secondaryPose.altM)
      : null;
  $: if (mounted && scene) {
    void loadVehicle(vehicleType);
  }

  onMount(() => {
    let disposed = false;

    requestAnimationFrame(() => {
      if (disposed) {
        return;
      }

      initScene();
      resizeObserver = new ResizeObserver(resize);
      resizeObserver.observe(container);
      resize();
      animate();
      mounted = true;
      void loadVehicle(vehicleType);
    });

    return () => {
      disposed = true;
      disposeScene();
    };
  });

  function initScene(): void {
    pal = paletteFor(theme);
    scene = new three.Scene();
    scene.background = new three.Color(pal.bg);
    scene.fog = new three.Fog(pal.bg, 26, 54);

    camera = new three.PerspectiveCamera(42, 1, 0.03, 28);
    camera.position.copy(CAMERA_START);
    camera.lookAt(CAMERA_TARGET);

    renderer = new three.WebGLRenderer({ canvas, antialias: true, alpha: false, preserveDrawingBuffer: true });
    renderer.setClearColor(pal.clear, 1);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    orbit = new OrbitControls(camera, renderer.domElement);
    orbit.enableDamping = true;
    orbit.dampingFactor = 0.11;
    orbit.enableRotate = true;
    orbit.enablePan = true;
    orbit.enableZoom = true;
    orbit.screenSpacePanning = false;
    orbit.minPolarAngle = 0.18;
    orbit.maxPolarAngle = Math.PI * 0.48;
    orbit.minDistance = 0.18;
    orbit.maxDistance = 8;
    orbit.rotateSpeed = 0.58;
    orbit.zoomSpeed = 0.78;
    orbit.panSpeed = 0.82;
    orbit.target.copy(CAMERA_TARGET);
    orbit.mouseButtons = {
      LEFT: three.MOUSE.PAN,
      MIDDLE: three.MOUSE.DOLLY,
      RIGHT: three.MOUSE.ROTATE
    };
    orbit.touches = {
      ONE: three.TOUCH.PAN,
      TWO: three.TOUCH.DOLLY_PAN
    };
    orbit.addEventListener('change', clampCameraControls);
    orbit.update();

    const ambient = new three.AmbientLight(0xbffaf0, 0.78);
    scene.add(ambient);

    const keyLight = new three.DirectionalLight(0xdffbf6, 1.35);
    keyLight.position.set(7, 12, 8);
    scene.add(keyLight);

    const fillLight = new three.PointLight(0xfd7719, 3.8, 24);
    fillLight.position.set(-7, 4, -6);
    scene.add(fillLight);

    frameGroup = buildFrame();
    scene.add(frameGroup);

    vehicleGroup = createVehicleMarker();
    scene.add(vehicleGroup);

    trail = createTrail(pal.xAxis, 0.7);
    scene.add(trail.line);

    updateVehicle(pose, attitude);
    updateMission(mission);
    // Signature starts empty, so this builds the overlay when sources are named.
    updateSourceOverlay(primaryLabel, secondaryLabel, primaryColor, secondaryColor, theme);
  }

  function createTrail(color: three.ColorRepresentation, opacity: number): Trail {
    const positions = new Float32Array(MAX_TRAIL * 3);
    const geometry = new three.BufferGeometry();
    geometry.setAttribute('position', new three.BufferAttribute(positions, 3));
    geometry.setDrawRange(0, 0);
    return {
      line: new three.Line(
        geometry,
        new three.LineBasicMaterial({ color, transparent: true, opacity })
      ),
      positions,
      count: 0
    };
  }

  async function loadVehicle(kind: VehicleKind): Promise<void> {
    if (!scene || !vehicleGroup) {
      return;
    }
    if (rig && rig.kind === kind) {
      return;
    }
    const token = ++rigLoadToken;

    let nextRig: VehicleRig;
    try {
      nextRig = await loadVehicleRig(kind, VEHICLE_FIT);
    } catch (error) {
      console.error('IndoorScene: vehicle model load failed', error);
      return;
    }

    if (token !== rigLoadToken || !vehicleGroup) {
      nextRig.dispose();
      return;
    }

    // Replace the placeholder arrow marker (or the previous model) in place.
    while (vehicleGroup.children.length > 0) {
      const child = vehicleGroup.children[0];
      vehicleGroup.remove(child);
      disposeTree(child);
    }
    rig?.dispose();
    rig = nextRig;
    vehicleGroup.add(rig.root);
    // Reset the trail so it doesn't jump across a vehicle swap.
    resetTrail(trail);
  }

  function applyTheme(name: 'light' | 'dark'): void {
    if (!scene || !renderer) {
      return;
    }

    pal = paletteFor(name);
    if (scene.background instanceof three.Color) {
      scene.background.set(pal.bg);
    }
    if (scene.fog) {
      (scene.fog as three.Fog).color.set(pal.bg);
    }
    renderer.setClearColor(pal.clear, 1);

    if (frameGroup) {
      scene.remove(frameGroup);
      disposeTree(frameGroup);
    }
    frameGroup = buildFrame();
    scene.add(frameGroup);

    // The vehicle model keeps its own GLB materials across themes; only recolor
    // the flight trail to the theme accent.
    if (trail) {
      (trail.line.material as three.LineBasicMaterial).color.set(pal.xAxis);
    }

    updateVehicle(pose, attitude);
    // The mission signature includes the theme, so this rebuilds the markers
    // with the new palette.
    updateMission(mission);
  }

  function buildFrame(): Group {
    const group = new three.Group();

    const floor = new three.Mesh(
      new three.PlaneGeometry(22, 22),
      new three.MeshBasicMaterial({
        color: pal.floor,
        transparent: true,
        opacity: pal.floorOpacity,
        side: three.DoubleSide
      })
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.015;
    group.add(floor);

    const grid = new three.GridHelper(22, 22, pal.gridCenter, pal.gridLine);
    grid.position.y = 0;
    group.add(grid);

    const fineGrid = new three.GridHelper(22, 44, pal.fineA, pal.fineB);
    fineGrid.position.y = 0.006;
    group.add(fineGrid);

    const origin = new three.Mesh(
      new three.SphereGeometry(0.012, 18, 12),
      new three.MeshBasicMaterial({ color: pal.ink })
    );
    origin.position.y = 0.015;
    group.add(origin);

    const axisOrigin = new three.Vector3(0, 0.015, 0);
    group.add(new three.ArrowHelper(new three.Vector3(1, 0, 0), axisOrigin, 0.45, pal.xAxis, 0.07, 0.035));
    group.add(new three.ArrowHelper(new three.Vector3(0, 0, -1), axisOrigin, 0.45, pal.forward, 0.07, 0.035));
    group.add(new three.ArrowHelper(new three.Vector3(0, 1, 0), axisOrigin, 0.22, pal.up, 0.06, 0.03));

    addAxisLabel(group, 'X', pal.labelX, new three.Vector3(0.52, 0.06, 0));
    addAxisLabel(group, 'Y', pal.labelY, new three.Vector3(0, 0.06, -0.52));
    addAxisLabel(group, 'Z', pal.labelZ, new three.Vector3(0.05, 0.3, 0));

    return group;
  }

  function addAxisLabel(group: Group, text: string, color: string, position: Vector3): void {
    const labelCanvas = document.createElement('canvas');
    labelCanvas.width = 128;
    labelCanvas.height = 72;
    const context = labelCanvas.getContext('2d');
    if (!context) {
      return;
    }

    context.clearRect(0, 0, labelCanvas.width, labelCanvas.height);
    context.font = '700 44px Inter, system-ui, sans-serif';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.shadowColor = pal.labelShadow;
    context.shadowBlur = 10;
    context.fillStyle = color;
    context.fillText(text, labelCanvas.width / 2, labelCanvas.height / 2);

    const texture = new three.CanvasTexture(labelCanvas);
    const material = new three.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false
    });
    const sprite = new three.Sprite(material);
    sprite.position.copy(position);
    sprite.scale.set(0.16, 0.09, 1);
    group.add(sprite);
  }

  function createVehicleMarker(): Group {
    const group = new three.Group();
    group.rotation.order = 'YXZ';
    const bodyOrigin = new three.Vector3(0, 0, 0);
    const origin = new three.Mesh(
      new three.SphereGeometry(0.02, 18, 12),
      new three.MeshBasicMaterial({ color: pal.ink })
    );
    group.add(origin);
    group.add(new three.ArrowHelper(new three.Vector3(0, 0, -1), bodyOrigin, 0.1, pal.ink, 0.025, 0.012));
    group.add(new three.ArrowHelper(new three.Vector3(1, 0, 0), bodyOrigin, 0.08, pal.forward, 0.02, 0.01));
    group.add(new three.ArrowHelper(new three.Vector3(0, 1, 0), bodyOrigin, 0.065, pal.up, 0.018, 0.009));

    return group;
  }

  // Mission waypoints are near-static (a periodic broadcast of a fixed plan),
  // so the marker group is rebuilt only when the plan, active item, or theme
  // actually changes.
  function missionSig(plan: MissionPlanState | null): string {
    if (!plan) {
      return theme;
    }
    const points = plan.waypoints
      .map((wp) => (wp ? `${wp.east},${wp.north},${wp.up}` : 'x'))
      .join(';');
    return `${theme}|${plan.missionId}|${plan.currentSeq}|${points}`;
  }

  function updateMission(plan: MissionPlanState | null): void {
    if (!scene) {
      return;
    }
    const signature = missionSig(plan);
    if (signature === missionSignature) {
      return;
    }
    missionSignature = signature;

    if (missionGroup) {
      scene.remove(missionGroup);
      disposeTree(missionGroup);
      missionGroup = null;
    }
    if (!plan || plan.waypoints.every((wp) => wp === null)) {
      return;
    }
    missionGroup = buildMissionGroup(plan);
    scene.add(missionGroup);
  }

  function buildMissionGroup(plan: MissionPlanState): Group {
    const group = new three.Group();
    const known = plan.waypoints.filter((wp): wp is NonNullable<typeof wp> => wp !== null);

    // Planned path through the waypoints in sequence order.
    if (known.length >= 2) {
      const points = known.map((wp) => enuToScene(wp.east, wp.north, wp.up));
      const pathGeometry = new three.BufferGeometry().setFromPoints(points);
      group.add(
        new three.Line(
          pathGeometry,
          new three.LineBasicMaterial({ color: pal.up, transparent: true, opacity: 0.55 })
        )
      );
    }

    for (const wp of known) {
      const active = wp.seq === plan.currentSeq;
      const position = enuToScene(wp.east, wp.north, wp.up);

      const marker = new three.Mesh(
        new three.SphereGeometry(active ? 0.03 : 0.018, 18, 12),
        new three.MeshBasicMaterial({
          color: active ? pal.xAxis : pal.ink,
          transparent: !active,
          opacity: active ? 1 : 0.78
        })
      );
      marker.position.copy(position);
      group.add(marker);

      if (active) {
        // Halo ring so the active waypoint reads at a glance.
        const halo = new three.Mesh(
          new three.RingGeometry(0.045, 0.058, 32),
          new three.MeshBasicMaterial({
            color: pal.xAxis,
            transparent: true,
            opacity: 0.85,
            side: three.DoubleSide
          })
        );
        halo.rotation.x = -Math.PI / 2;
        halo.position.copy(position);
        group.add(halo);
      }

      // Drop line to the floor anchors the altitude visually.
      const dropGeometry = new three.BufferGeometry().setFromPoints([
        position,
        new three.Vector3(position.x, 0.002, position.z)
      ]);
      group.add(
        new three.Line(
          dropGeometry,
          new three.LineBasicMaterial({
            color: active ? pal.xAxis : pal.gridLine,
            transparent: true,
            opacity: active ? 0.55 : 0.4
          })
        )
      );

      addAxisLabel(
        group,
        String(wp.seq + 1),
        active ? pal.labelX : pal.labelY,
        position.clone().add(new three.Vector3(0, 0.07, 0))
      );
    }

    return group;
  }

  function enuToScene(east: number, north: number, up: number): Vector3 {
    return new three.Vector3(
      clamp(east * LOCAL_METERS_TO_SCENE, -9.8, 9.8),
      localAltitudeSceneY(up),
      clamp(-north * LOCAL_METERS_TO_SCENE, -9.8, 9.8)
    );
  }

  function updateVehicle(nextPose: Pose | null, nextAttitude: Attitude | null): void {
    if (!vehicleGroup || !nextPose) {
      return;
    }

    const vehiclePosition = localPositionToScene(nextPose);
    vehicleGroup.position.copy(vehiclePosition);
    vehicleGroup.quaternion.copy(attitudeToSceneQuaternion(nextAttitude));
    recordTrail(trail, vehiclePosition);
    refreshSourceOverlay();
  }

  /**
   * Draw the second source as a wire airframe at its own pose and attitude.
   * Not the vehicle model: two solid aircraft would read as two aircraft.
   */
  function updateSecondary(nextPose: Pose | null, nextAttitude: Attitude | null): void {
    if (secondaryMarker && nextPose) {
      const position = localPositionToScene(nextPose);
      secondaryMarker.root.position.copy(position);
      secondaryMarker.root.quaternion.copy(attitudeToSceneQuaternion(nextAttitude));
      recordTrail(secondaryTrail, position);
    }
    refreshSourceOverlay();
  }

  function attitudeToSceneQuaternion(nextAttitude: Attitude | null): three.Quaternion {
    if (hasSynapseQuaternion(nextAttitude)) {
      const bodyToEnu = new three.Quaternion(
        nextAttitude.qx,
        nextAttitude.qy,
        nextAttitude.qz,
        nextAttitude.qw
      ).normalize();
      const xAxis = synapseBodyVectorToScene(modelLocalVectorToSynapseBody(new three.Vector3(1, 0, 0)), bodyToEnu);
      const yAxis = synapseBodyVectorToScene(modelLocalVectorToSynapseBody(new three.Vector3(0, 1, 0)), bodyToEnu);
      const zAxis = synapseBodyVectorToScene(modelLocalVectorToSynapseBody(new three.Vector3(0, 0, 1)), bodyToEnu);
      return new three.Quaternion().setFromRotationMatrix(new three.Matrix4().makeBasis(xAxis, yAxis, zAxis));
    }

    return new three.Quaternion().setFromEuler(
      new three.Euler(
        three.MathUtils.degToRad(nextAttitude?.pitchDeg ?? 0),
        three.MathUtils.degToRad(nextAttitude?.yawDeg ?? 0) + YAW_ZERO_EAST_OFFSET_RAD,
        -three.MathUtils.degToRad(nextAttitude?.rollDeg ?? 0),
        'YXZ'
      )
    );
  }

  function hasSynapseQuaternion(
    nextAttitude: Attitude | null
  ): nextAttitude is Attitude & { qx: number; qy: number; qz: number; qw: number } {
    return (
      nextAttitude !== null &&
      Number.isFinite(nextAttitude.qx) &&
      Number.isFinite(nextAttitude.qy) &&
      Number.isFinite(nextAttitude.qz) &&
      Number.isFinite(nextAttitude.qw)
    );
  }

  function modelLocalVectorToSynapseBody(vector: three.Vector3): three.Vector3 {
    return new three.Vector3(-vector.z, -vector.x, vector.y);
  }

  function synapseBodyVectorToScene(vector: three.Vector3, bodyToEnu: three.Quaternion): three.Vector3 {
    const enu = vector.applyQuaternion(bodyToEnu);
    return new three.Vector3(enu.x, enu.z, -enu.y).normalize();
  }

  function recordTrail(target: Trail | null, position: Vector3): void {
    if (!target) {
      return;
    }
    if (target.count >= MAX_TRAIL) {
      target.positions.copyWithin(0, 3);
      target.count = MAX_TRAIL - 1;
    }
    const index = target.count;
    target.positions[index * 3] = position.x;
    target.positions[index * 3 + 1] = position.y;
    target.positions[index * 3 + 2] = position.z;
    target.count++;
    const attribute = target.line.geometry.getAttribute('position') as three.BufferAttribute;
    attribute.needsUpdate = true;
    target.line.geometry.setDrawRange(0, Math.min(target.count, MAX_TRAIL));
  }

  function resetTrail(target: Trail | null): void {
    if (!target) {
      return;
    }
    target.count = 0;
    target.line.geometry.setDrawRange(0, 0);
  }

  /**
   * Rebuild the two-source overlay when the sources or the palette change.
   * An empty `primaryLabel` means there is nothing to disambiguate, so the
   * overlay is torn down entirely and the view is the plain single-vehicle one.
   */
  function updateSourceOverlay(
    nextPrimary: string,
    nextSecondary: string,
    nextPrimaryColor: string,
    nextSecondaryColor: string,
    nextTheme: 'light' | 'dark'
  ): void {
    if (!scene) {
      return;
    }
    const signature = `${nextPrimary}|${nextSecondary}|${nextPrimaryColor}|${nextSecondaryColor}|${nextTheme}`;
    if (signature === sourceSignature) {
      return;
    }
    sourceSignature = signature;
    clearSourceOverlay();
    if (!nextPrimary.trim()) {
      return;
    }
    buildSourceOverlay(nextPrimary, nextSecondary, nextPrimaryColor, nextSecondaryColor);
    updateSecondary(secondaryPose, secondaryAttitude);
  }

  function buildSourceOverlay(
    labelPrimary: string,
    labelSecondary: string,
    colorPrimary: string,
    colorSecondary: string
  ): void {
    const group = new three.Group();

    primaryHalo = createSourceHalo(HALO_RADIUS_SCENE, colorPrimary);
    group.add(primaryHalo);
    primaryLabelSprite = createSourceLabel(labelPrimary, colorPrimary, pal.labelShadow);
    primaryLabelSprite.scale.set(LABEL_SCALE[0], LABEL_SCALE[1], 1);
    group.add(primaryLabelSprite);

    if (labelSecondary.trim()) {
      secondaryMarker = createSourceMarker(VEHICLE_FIT * 1.7, colorSecondary);
      group.add(secondaryMarker.root);
      secondaryHalo = createSourceHalo(HALO_RADIUS_SCENE, colorSecondary);
      group.add(secondaryHalo);
      secondaryLabelSprite = createSourceLabel(labelSecondary, colorSecondary, pal.labelShadow);
      secondaryLabelSprite.scale.set(LABEL_SCALE[0], LABEL_SCALE[1], 1);
      group.add(secondaryLabelSprite);

      offsetLine = new three.Line(
        new three.BufferGeometry().setFromPoints([new three.Vector3(), new three.Vector3()]),
        new three.LineBasicMaterial({ color: colorSecondary, transparent: true, opacity: 0.45 })
      );
      group.add(offsetLine);

      secondaryTrail = createTrail(colorSecondary, 0.5);
      group.add(secondaryTrail.line);
    }

    sourceGroup = group;
    scene?.add(group);
  }

  function clearSourceOverlay(): void {
    if (vehicleGroup) {
      // Only the overlay hides the model; without it there is one source and
      // the view is always drawn.
      vehicleGroup.visible = true;
    }
    if (sourceGroup) {
      scene?.remove(sourceGroup);
      disposeTree(sourceGroup);
    }
    sourceGroup = null;
    secondaryMarker = null;
    primaryHalo = null;
    secondaryHalo = null;
    primaryLabelSprite = null;
    secondaryLabelSprite = null;
    offsetLine = null;
    secondaryTrail = null;
  }

  /** Park the halos, labels and offset line on the poses they annotate. */
  function refreshSourceOverlay(): void {
    if (!sourceGroup || !vehicleGroup) {
      return;
    }
    // With the overlay on, the model carries a source's name, so it must not be
    // left parked at a stale position when that source stops reporting.
    const hasPrimary = pose !== null;
    vehicleGroup.visible = hasPrimary;
    if (primaryHalo) {
      primaryHalo.visible = hasPrimary;
    }
    if (primaryLabelSprite) {
      primaryLabelSprite.visible = hasPrimary;
    }

    const primaryPosition = vehicleGroup.position;
    primaryHalo?.position.set(primaryPosition.x, FLOOR_MARK_SCENE_Y, primaryPosition.z);
    primaryLabelSprite?.position.set(
      primaryPosition.x,
      primaryPosition.y + LABEL_LIFT_SCENE_Y,
      primaryPosition.z
    );

    const hasSecondary = secondaryPose !== null && secondaryMarker !== null;
    if (secondaryMarker) {
      secondaryMarker.root.visible = hasSecondary;
    }
    if (secondaryHalo) {
      secondaryHalo.visible = hasSecondary;
    }
    if (secondaryLabelSprite) {
      secondaryLabelSprite.visible = hasSecondary;
    }
    if (secondaryTrail) {
      secondaryTrail.line.visible = hasSecondary;
    }
    if (offsetLine) {
      offsetLine.visible = hasSecondary && hasPrimary;
    }
    if (!hasSecondary || !secondaryMarker) {
      return;
    }

    const secondaryPosition = secondaryMarker.root.position;
    secondaryHalo?.position.set(secondaryPosition.x, FLOOR_MARK_SCENE_Y, secondaryPosition.z);
    secondaryLabelSprite?.position.set(
      secondaryPosition.x,
      secondaryPosition.y + LABEL_LIFT_SCENE_Y,
      secondaryPosition.z
    );
    if (offsetLine) {
      const points = offsetLine.geometry.getAttribute('position') as three.BufferAttribute;
      points.setXYZ(0, primaryPosition.x, primaryPosition.y, primaryPosition.z);
      points.setXYZ(1, secondaryPosition.x, secondaryPosition.y, secondaryPosition.z);
      points.needsUpdate = true;
      offsetLine.geometry.computeBoundingSphere();
    }
  }

  function localPositionToScene(nextPose: Pose): Vector3 {
    return new three.Vector3(
      clamp(nextPose.xM * LOCAL_METERS_TO_SCENE, -9.8, 9.8),
      localAltitudeSceneY(nextPose.altM),
      clamp(-nextPose.yM * LOCAL_METERS_TO_SCENE, -9.8, 9.8)
    );
  }

  function localAltitudeSceneY(altM: number): number {
    return clamp(altM * LOCAL_METERS_TO_SCENE, MIN_ALTITUDE_SCENE_Y, 3.2);
  }

  function resize(): void {
    if (!container || !renderer || !camera) {
      return;
    }

    const width = container.clientWidth;
    const height = container.clientHeight;
    renderer.setSize(width, height, false);
    camera.aspect = width / Math.max(1, height);
    camera.updateProjectionMatrix();
  }

  function animate(): void {
    animationFrame = requestAnimationFrame(animate);
    if (!renderer || !scene || !camera) {
      return;
    }
    rig?.update(controls, motors);
    if (followMode && orbit && vehicleGroup) {
      followDelta.copy(vehicleGroup.position).sub(orbit.target).multiplyScalar(FOLLOW_LERP);
      orbit.target.add(followDelta);
      camera.position.add(followDelta);
    }
    orbit?.update();
    renderer.render(scene, camera);
  }

  function toggleFollow(): void {
    followMode = !followMode;
    if (orbit) {
      // Panning would fight the follow target every frame; hand the target
      // back to the user only when follow is off.
      orbit.enablePan = !followMode;
    }
  }

  function disposeScene(): void {
    cancelAnimationFrame(animationFrame);
    resizeObserver?.disconnect();
    orbit?.removeEventListener('change', clampCameraControls);
    orbit?.dispose();
    rig?.dispose();
    rig = null;
    renderer?.dispose();
    scene?.traverse((object: Object3D) => {
      const maybeRenderable = object as Object3D & {
        geometry?: BufferGeometry;
        material?: Material | Material[];
      };
      maybeRenderable.geometry?.dispose();
      if (maybeRenderable.material) {
        const materials = Array.isArray(maybeRenderable.material) ? maybeRenderable.material : [maybeRenderable.material];
        for (const material of materials) {
          const materialWithMap = material as Material & { map?: { dispose: () => void } };
          materialWithMap.map?.dispose();
          material.dispose();
        }
      }
    });
    renderer = null;
    scene = null;
    camera = null;
    orbit = null;
    vehicleGroup = null;
    frameGroup = null;
    missionGroup = null;
    missionSignature = '';
    trail = null;
    sourceGroup = null;
    sourceSignature = '';
    secondaryMarker = null;
    primaryHalo = null;
    secondaryHalo = null;
    primaryLabelSprite = null;
    secondaryLabelSprite = null;
    offsetLine = null;
    secondaryTrail = null;
    resizeObserver = null;
  }

  function clampCameraControls(): void {
    if (!orbit) {
      return;
    }

    const min = new three.Vector3(-13, 0.05, -13);
    const max = new three.Vector3(13, 4.6, 13);
    orbit.target.clamp(min, max);
  }

  function clamp(value: number, min: number, max: number): number {
    return Math.min(max, Math.max(min, value));
  }

  function containSceneWheel(event: WheelEvent): void {
    event.preventDefault();
  }

  function containSceneContextMenu(event: MouseEvent): void {
    event.preventDefault();
  }
</script>

<div
  class="indoor-scene"
  class:light={theme === 'light'}
  bind:this={container}
  role="application"
  aria-label="Indoor local navigation map"
  onwheel={containSceneWheel}
  oncontextmenu={containSceneContextMenu}
>
  <canvas bind:this={canvas} aria-label="Indoor 3D local navigation view"></canvas>
  <button
    type="button"
    class="follow-toggle"
    class:active={followMode}
    aria-pressed={followMode}
    onclick={toggleFollow}
  >
    {followMode ? 'Following' : 'Follow'}
  </button>
  {#if showSources}
    <div class="source-legend">
      <div class="source-row">
        <span class="swatch" style={`background:${primaryColor};`}></span>
        <span class="source-name">{primaryLabel}</span>
        <strong>
          {pose
            ? `${pose.xM.toFixed(2)}, ${pose.yM.toFixed(2)}, ${pose.altM.toFixed(2)} m`
            : 'no pose'}
        </strong>
      </div>
      {#if secondaryLabel}
        <div class="source-row">
          <span class="swatch outline" style={`border-color:${secondaryColor};`}></span>
          <span class="source-name">{secondaryLabel}</span>
          <strong>
            {secondaryPose
              ? `${secondaryPose.xM.toFixed(2)}, ${secondaryPose.yM.toFixed(2)}, ${secondaryPose.altM.toFixed(2)} m`
              : 'no pose'}
          </strong>
        </div>
        <div class="source-row separation">
          <span class="swatch spacer"></span>
          <span class="source-name">Separation</span>
          <strong>{separationM !== null ? `${separationM.toFixed(2)} m` : '--'}</strong>
        </div>
      {/if}
    </div>
  {/if}
  <div class="indoor-readout">
    <div>
      <span>Local X</span>
      <strong>{localX.toFixed(1)} m</strong>
    </div>
    <div>
      <span>Local Y</span>
      <strong>{localY.toFixed(1)} m</strong>
    </div>
    <div>
      <span>Alt</span>
      <strong>{localAlt.toFixed(1)} m</strong>
    </div>
    <div>
      <span>Quality</span>
      <strong>{(localizationQuality * 100).toFixed(0)}%</strong>
    </div>
  </div>
</div>

<style>
  .indoor-scene {
    position: relative;
    overflow: hidden;
    width: 100%;
    height: 420px;
    border: 1px solid #2a383f;
    border-radius: 8px;
    background: #091012;
    overscroll-behavior: contain;
    touch-action: none;
  }

  .indoor-scene.light {
    border-color: #cdd5dc;
    background: #eef1f3;
  }

  canvas {
    display: block;
    width: 100%;
    height: 100%;
    cursor: grab;
    touch-action: none;
  }

  canvas:active {
    cursor: grabbing;
  }

  .follow-toggle {
    position: absolute;
    top: 12px;
    right: 12px;
    padding: 6px 12px;
    border: 1px solid rgba(253, 119, 25, 0.35);
    border-radius: 8px;
    background: rgba(5, 8, 8, 0.74);
    backdrop-filter: blur(5px);
    color: #edf6f1;
    font-size: 0.68rem;
    font-weight: 760;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    cursor: pointer;
  }

  .follow-toggle:hover {
    border-color: rgba(253, 119, 25, 0.7);
  }

  .follow-toggle.active {
    border-color: #fd7719;
    background: rgba(253, 119, 25, 0.22);
    color: #ffd9b8;
  }

  .indoor-scene.light .follow-toggle {
    border-color: rgba(227, 95, 12, 0.35);
    background: rgba(255, 255, 255, 0.82);
    color: #12171b;
  }

  .indoor-scene.light .follow-toggle:hover {
    border-color: rgba(227, 95, 12, 0.75);
  }

  .indoor-scene.light .follow-toggle.active {
    border-color: #e35f0c;
    background: rgba(227, 95, 12, 0.16);
    color: #a04208;
  }

  .source-legend {
    position: absolute;
    top: 12px;
    left: 12px;
    display: grid;
    gap: 4px;
    padding: 8px 10px;
    border: 1px solid rgba(253, 119, 25, 0.2);
    border-radius: 8px;
    background: rgba(5, 8, 8, 0.74);
    backdrop-filter: blur(5px);
    pointer-events: none;
  }

  .source-row {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .source-row .swatch {
    width: 11px;
    height: 11px;
    border-radius: 3px;
    flex: none;
  }

  .source-row .swatch.outline {
    background: transparent;
    border: 2px solid;
    border-radius: 50%;
  }

  .source-row .swatch.spacer {
    background: transparent;
  }

  .source-row .source-name {
    min-width: 74px;
    color: #91a39c;
    font-size: 0.62rem;
    font-weight: 760;
    text-transform: uppercase;
  }

  .source-row strong {
    color: #edf6f1;
    font-size: 0.72rem;
    font-weight: 760;
    font-variant-numeric: tabular-nums;
  }

  .source-row.separation strong {
    color: #ffc35a;
  }

  .indoor-scene.light .source-legend {
    border-color: rgba(227, 95, 12, 0.28);
    background: rgba(255, 255, 255, 0.86);
  }

  .indoor-scene.light .source-row .source-name {
    color: #5c6873;
  }

  .indoor-scene.light .source-row strong {
    color: #12171b;
  }

  .indoor-scene.light .source-row.separation strong {
    color: #a04208;
  }

  .indoor-readout {
    position: absolute;
    right: 12px;
    bottom: 12px;
    left: 12px;
    display: grid;
    grid-template-columns: repeat(4, minmax(0, 1fr));
    gap: 6px;
    pointer-events: none;
  }

  .indoor-readout > div {
    display: grid;
    gap: 3px;
    min-width: 0;
    padding: 7px 8px;
    border: 1px solid rgba(253, 119, 25, 0.2);
    border-radius: 8px;
    background: rgba(5, 8, 8, 0.74);
    backdrop-filter: blur(5px);
  }

  .indoor-readout span {
    overflow: hidden;
    color: #91a39c;
    font-size: 0.62rem;
    font-weight: 760;
    text-overflow: ellipsis;
    text-transform: uppercase;
    white-space: nowrap;
  }

  .indoor-readout strong {
    overflow: hidden;
    color: #edf6f1;
    font-size: 0.83rem;
    font-weight: 760;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .indoor-scene.light .indoor-readout > div {
    border-color: rgba(227, 95, 12, 0.28);
    background: rgba(255, 255, 255, 0.82);
  }

  .indoor-scene.light .indoor-readout span {
    color: #5c6873;
  }

  .indoor-scene.light .indoor-readout strong {
    color: #12171b;
  }

  @media (max-width: 820px) {
    .indoor-scene {
      height: 300px;
    }

    .indoor-readout {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }
</style>
