import * as THREE from 'three';
import type { VenueId } from '../core/game/venues';
import { buildGym } from './venues/gym';
import { buildMoon } from './venues/moon';
import { buildSun } from './venues/sun';
import type { VenueLook } from './venues/common';

export interface World {
  scene: THREE.Scene;
  /** Key light whose shadow camera follows the plane. */
  sun: THREE.DirectionalLight;
  /** Move the shadow-casting light so it covers `p`. */
  followLight(p: THREE.Vector3): void;
  /** Show another place to throw (scenery, sky and light). */
  setVenue(id: VenueId): void;
  venue: VenueId;
}

const BUILDERS: Record<VenueId, () => VenueLook> = { gym: buildGym, moon: buildMoon, sun: buildSun };

/** The world around the throw: one of the venues, lit by a fill light and a shadow-casting key light. */
export function createWorld(initial: VenueId = 'gym'): World {
  const scene = new THREE.Scene();
  const fill = new THREE.HemisphereLight('#f2f6ff', '#8a7f6e', 1.1);
  scene.add(fill);
  const sun = new THREE.DirectionalLight('#fff3df', 2.0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  const r = 2.5;
  Object.assign(sun.shadow.camera, { left: -r, right: r, top: r, bottom: -r, near: 0.5, far: 30 });
  sun.shadow.bias = -0.0005;
  sun.shadow.normalBias = 0.01;
  scene.add(sun, sun.target);
  const offset = new THREE.Vector3(-4, 10, 3);
  const followLight = (p: THREE.Vector3) => {
    sun.target.position.copy(p);
    sun.position.copy(p).add(offset);
    sun.target.updateMatrixWorld();
  };

  // scenery is built once per venue and kept (switching back is instant)
  const built = new Map<VenueId, VenueLook>();
  let current: VenueLook | null = null;
  const world: World = {
    scene,
    sun,
    followLight,
    venue: initial,
    setVenue(id: VenueId) {
      if (current) scene.remove(current.group);
      let look = built.get(id);
      if (!look) {
        look = BUILDERS[id]();
        built.set(id, look);
      }
      current = look;
      world.venue = id;
      scene.add(look.group);
      scene.background = look.background;
      scene.fog = look.fog;
      sun.color.set(look.key.color);
      sun.intensity = look.key.intensity;
      offset.copy(look.key.offset).normalize().multiplyScalar(10);
      fill.color.set(look.fill.sky);
      fill.groundColor.set(look.fill.ground);
      fill.intensity = look.fill.intensity;
      followLight(sun.target.position.clone());
    },
  };
  world.setVenue(initial);
  followLight(new THREE.Vector3());
  return world;
}
