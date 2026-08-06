// Overlay pieces for drawing two pose/attitude sources — telemetry and mocap —
// in a single view.
//
// The second source is deliberately *not* a copy of the vehicle model: two
// solid aircraft in one scene read as two aircraft, where an open wire outline
// reads as "the same vehicle, according to the other source". Colour is carried
// by the halo and the label rather than the model, because the GLB rigs bring
// their own materials and tinting them would fight the lighting.
import * as three from 'three';
import type { BufferGeometry, Group, Material, Mesh, Object3D, Sprite } from 'three';

export type SourceMarker = {
  root: Group;
  dispose(): void;
};

/**
 * Wire airframe in model-local axes: nose along -Z, right wing +X, up +Y —
 * the same convention the GLB rigs are authored in, so it can be driven by the
 * same attitude quaternion.
 */
export function createSourceMarker(span: number, color: three.ColorRepresentation): SourceMarker {
  const root = new three.Group();
  root.rotation.order = 'YXZ';

  const half = span / 2;
  const nose = half * 1.25;
  const material = new three.LineBasicMaterial({ color, transparent: true, opacity: 0.9 });
  const segments: Array<[number, number, number]> = [
    // Fuselage.
    [0, 0, half * 0.9],
    [0, 0, -nose],
    // Wing.
    [-half, 0, 0],
    [half, 0, 0],
    // Fin, so roll and pitch are readable from any angle.
    [0, 0, half * 0.75],
    [0, half * 0.5, half * 0.75],
    // Nose chevron.
    [-half * 0.3, 0, -half * 0.72],
    [0, 0, -nose],
    [half * 0.3, 0, -half * 0.72],
    [0, 0, -nose]
  ];
  const geometry = new three.BufferGeometry().setFromPoints(
    segments.map(([x, y, z]) => new three.Vector3(x, y, z))
  );
  root.add(new three.LineSegments(geometry, material));
  root.add(new three.LineLoop(circleGeometry(half * 0.95, 40), material));

  return {
    root,
    dispose: () => disposeTree(root)
  };
}

/** Flat ring drawn on the floor under a source, in that source's colour. */
export function createSourceHalo(radius: number, color: three.ColorRepresentation): Mesh {
  const halo = new three.Mesh(
    new three.RingGeometry(radius * 0.78, radius, 40),
    new three.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.75,
      side: three.DoubleSide,
      depthWrite: false
    })
  );
  halo.rotation.x = -Math.PI / 2;
  return halo;
}

/** Billboard text naming a source, so the colours never have to be memorised. */
export function createSourceLabel(text: string, color: string, shadow: string): Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 64;
  const context = canvas.getContext('2d');
  if (context) {
    context.clearRect(0, 0, canvas.width, canvas.height);
    context.font = '800 34px Inter, system-ui, sans-serif';
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.shadowColor = shadow;
    context.shadowBlur = 12;
    context.fillStyle = color;
    context.fillText(text.toUpperCase(), canvas.width / 2, canvas.height / 2);
  }

  const sprite = new three.Sprite(
    new three.SpriteMaterial({
      map: new three.CanvasTexture(canvas),
      transparent: true,
      depthTest: false,
      depthWrite: false
    })
  );
  sprite.scale.set(0.26, 0.065, 1);
  return sprite;
}

function circleGeometry(radius: number, segments: number): BufferGeometry {
  const points: three.Vector3[] = [];
  for (let index = 0; index < segments; index += 1) {
    const angle = (index / segments) * Math.PI * 2;
    points.push(new three.Vector3(Math.cos(angle) * radius, 0, Math.sin(angle) * radius));
  }
  return new three.BufferGeometry().setFromPoints(points);
}

export function disposeTree(root: Object3D): void {
  root.traverse((object: Object3D) => {
    const renderable = object as Object3D & {
      geometry?: BufferGeometry;
      material?: Material | Material[];
    };
    renderable.geometry?.dispose();
    if (!renderable.material) {
      return;
    }
    const materials = Array.isArray(renderable.material) ? renderable.material : [renderable.material];
    for (const material of materials) {
      (material as Material & { map?: { dispose: () => void } }).map?.dispose();
      material.dispose();
    }
  });
}
