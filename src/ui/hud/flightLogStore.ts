import type { LaunchParams } from '../../core/physics/launch';

/** One finished throw, as kept in the browser (per viewer, best effort). */
export interface StoredFlight {
  date: string;
  design: string;
  launch: LaunchParams;
  /** Official result: horizontal distance and time to the first floor contact. */
  distance: number;
  time: number;
  impactSpeed: number;
  /** Where the plane finally came to rest (after sliding/bouncing), if it did. */
  restDistance: number | null;
  maxHeight: number;
}

const KEY = 'paperplanes.flights.v1';
const MAX = 30;

export function loadFlights(): StoredFlight[] {
  try {
    const raw = localStorage.getItem(KEY);
    const arr = raw ? (JSON.parse(raw) as unknown) : [];
    return Array.isArray(arr) ? (arr as StoredFlight[]) : [];
  } catch {
    return [];
  }
}

export function saveFlight(f: StoredFlight): StoredFlight[] {
  const all = [f, ...loadFlights()].slice(0, MAX);
  try {
    localStorage.setItem(KEY, JSON.stringify(all));
  } catch {
    /* storage unavailable (private mode etc.) – the list just isn't kept */
  }
  return all;
}

export function clearFlights() {
  try {
    localStorage.removeItem(KEY);
  } catch {
    /* ignore */
  }
}
