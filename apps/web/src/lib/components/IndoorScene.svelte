<script lang="ts">
  import { onMount } from 'svelte';
  import type { Attitude, Pose } from '@electrode/sdk';
  import * as three from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import type { BufferGeometry, Group, Material, Object3D, PerspectiveCamera, Scene, Vector3, WebGLRenderer } from 'three';

  export let pose: Pose | null = null;
  export let attitude: Attitude | null = null;
  export let localizationQuality = 0;

  let container: HTMLDivElement;
  let canvas: HTMLCanvasElement;
  let renderer: WebGLRenderer | null = null;
  let scene: Scene | null = null;
  let camera: PerspectiveCamera | null = null;
  let controls: OrbitControls | null = null;
  let vehicleGroup: Group | null = null;
  let resizeObserver: ResizeObserver | null = null;
  let animationFrame = 0;

  $: localX = pose?.xM ?? 0;
  $: localY = pose?.yM ?? 0;
  $: localAlt = pose?.altM ?? 0;
  $: updateVehicle(pose, attitude);

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
    });

    return () => {
      disposed = true;
      disposeScene();
    };
  });

  function initScene(): void {
    scene = new three.Scene();
    scene.background = new three.Color(0x0a1113);
    scene.fog = new three.Fog(0x0a1113, 26, 54);

    camera = new three.PerspectiveCamera(46, 1, 0.1, 80);
    camera.position.set(8.8, 6.4, 9.6);
    camera.lookAt(0, 0.45, 0);

    renderer = new three.WebGLRenderer({ canvas, antialias: true, alpha: false, preserveDrawingBuffer: true });
    renderer.setClearColor(0x091012, 1);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.11;
    controls.enableRotate = true;
    controls.enablePan = true;
    controls.enableZoom = true;
    controls.screenSpacePanning = false;
    controls.minPolarAngle = 0.18;
    controls.maxPolarAngle = Math.PI * 0.48;
    controls.minDistance = 4.8;
    controls.maxDistance = 34;
    controls.rotateSpeed = 0.58;
    controls.zoomSpeed = 0.78;
    controls.panSpeed = 0.82;
    controls.target.set(0, 0.45, 0);
    controls.mouseButtons = {
      LEFT: three.MOUSE.PAN,
      MIDDLE: three.MOUSE.DOLLY,
      RIGHT: three.MOUSE.ROTATE
    };
    controls.touches = {
      ONE: three.TOUCH.PAN,
      TWO: three.TOUCH.DOLLY_PAN
    };
    controls.addEventListener('change', clampCameraControls);
    controls.update();

    const ambient = new three.AmbientLight(0xbffaf0, 0.78);
    scene.add(ambient);

    const keyLight = new three.DirectionalLight(0xdffbf6, 1.35);
    keyLight.position.set(7, 12, 8);
    scene.add(keyLight);

    const fillLight = new three.PointLight(0x42e8c4, 3.8, 24);
    fillLight.position.set(-7, 4, -6);
    scene.add(fillLight);

    addLocalFrame();

    vehicleGroup = createVehicleMarker();
    scene.add(vehicleGroup);

    updateVehicle(pose, attitude);
  }

  function addLocalFrame(): void {
    if (!scene) {
      return;
    }

    const floor = new three.Mesh(
      new three.PlaneGeometry(22, 22),
      new three.MeshBasicMaterial({
        color: 0x0d1718,
        transparent: true,
        opacity: 0.62,
        side: three.DoubleSide
      })
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.015;
    scene.add(floor);

    const grid = new three.GridHelper(22, 22, 0x42e8c4, 0x203a3a);
    grid.position.y = 0;
    scene.add(grid);

    const fineGrid = new three.GridHelper(22, 44, 0x1d5b55, 0x142827);
    fineGrid.position.y = 0.006;
    scene.add(fineGrid);

    const origin = new three.Mesh(
      new three.SphereGeometry(0.13, 18, 12),
      new three.MeshBasicMaterial({ color: 0xe9fff9 })
    );
    origin.position.y = 0.08;
    scene.add(origin);

    const axisOrigin = new three.Vector3(0, 0.08, 0);
    scene.add(new three.ArrowHelper(new three.Vector3(1, 0, 0), axisOrigin, 6.2, 0x42e8c4, 0.38, 0.19));
    scene.add(new three.ArrowHelper(new three.Vector3(0, 0, -1), axisOrigin, 6.2, 0x61a8ff, 0.38, 0.19));
    scene.add(new three.ArrowHelper(new three.Vector3(0, 1, 0), axisOrigin, 2.9, 0xffc35a, 0.34, 0.17));

    addAxisLabel('X', '#42e8c4', new three.Vector3(6.85, 0.34, 0));
    addAxisLabel('Y', '#61a8ff', new three.Vector3(0, 0.34, -6.85));
    addAxisLabel('Z', '#ffc35a', new three.Vector3(0.35, 3.35, 0));
  }

  function addAxisLabel(text: string, color: string, position: Vector3): void {
    if (!scene) {
      return;
    }
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
    context.shadowColor = '#000000';
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
    sprite.scale.set(0.72, 0.4, 1);
    scene.add(sprite);
  }

  function createVehicleMarker(): Group {
    const group = new three.Group();
    group.rotation.order = 'YXZ';
    const bodyOrigin = new three.Vector3(0, 0, 0);
    const origin = new three.Mesh(
      new three.SphereGeometry(0.12, 18, 12),
      new three.MeshBasicMaterial({ color: 0xe9fff9 })
    );
    group.add(origin);
    group.add(new three.ArrowHelper(new three.Vector3(0, 0, -1), bodyOrigin, 1.28, 0xe9fff9, 0.24, 0.12));
    group.add(new three.ArrowHelper(new three.Vector3(1, 0, 0), bodyOrigin, 1.02, 0x61a8ff, 0.2, 0.1));
    group.add(new three.ArrowHelper(new three.Vector3(0, 1, 0), bodyOrigin, 0.86, 0xffc35a, 0.18, 0.09));

    return group;
  }

  function updateVehicle(nextPose: Pose | null, nextAttitude: Attitude | null): void {
    if (!vehicleGroup || !nextPose) {
      return;
    }

    const vehiclePosition = localPositionToScene(nextPose);
    vehicleGroup.position.copy(vehiclePosition);
    vehicleGroup.rotation.set(
      three.MathUtils.degToRad(nextAttitude?.pitchDeg ?? 0),
      -three.MathUtils.degToRad(nextAttitude?.yawDeg ?? 0),
      -three.MathUtils.degToRad(nextAttitude?.rollDeg ?? 0)
    );
  }

  function localPositionToScene(nextPose: Pose): Vector3 {
    return new three.Vector3(
      clamp(nextPose.xM * 0.09, -9.8, 9.8),
      localAltitudeSceneY(nextPose.altM),
      clamp(-nextPose.yM * 0.09, -9.8, 9.8)
    );
  }

  function localAltitudeSceneY(altM: number): number {
    return clamp(altM * 0.08, 0.25, 3.2);
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
    controls?.update();
    renderer.render(scene, camera);
  }

  function disposeScene(): void {
    cancelAnimationFrame(animationFrame);
    resizeObserver?.disconnect();
    controls?.removeEventListener('change', clampCameraControls);
    controls?.dispose();
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
    controls = null;
    vehicleGroup = null;
    resizeObserver = null;
  }

  function clampCameraControls(): void {
    if (!controls) {
      return;
    }

    const min = new three.Vector3(-13, 0.05, -13);
    const max = new three.Vector3(13, 4.6, 13);
    controls.target.clamp(min, max);
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
  bind:this={container}
  role="application"
  aria-label="Indoor local navigation map"
  onwheel={containSceneWheel}
  oncontextmenu={containSceneContextMenu}
>
  <canvas bind:this={canvas} aria-label="Indoor 3D local navigation view"></canvas>
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
    border: 1px solid rgba(66, 232, 196, 0.2);
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

  @media (max-width: 820px) {
    .indoor-scene {
      height: 300px;
    }

    .indoor-readout {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }
</style>
