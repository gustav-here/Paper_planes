import type { GameMode, LeaderEntry } from '../../core/game/competition';
import { insertEntry } from '../../core/game/competition';
import type { VenueId } from '../../core/game/venues';

const KEY = 'paperplanes.leaderboard.v1';

/** One top list per game mode and venue (the gym's lists keep their original keys). */
type Boards = Partial<Record<string, LeaderEntry[]>>;
const boardKey = (mode: GameMode, venue: VenueId) => (venue === 'gym' ? mode : `${venue}:${mode}`);

function load(): Boards {
  try {
    const raw = localStorage.getItem(KEY);
    const b = raw ? (JSON.parse(raw) as unknown) : {};
    return b && typeof b === 'object' ? (b as Boards) : {};
  } catch {
    return {};
  }
}

export function leaderboard(mode: GameMode, venue: VenueId = 'gym'): LeaderEntry[] {
  return load()[boardKey(mode, venue)] ?? [];
}

/** Add a result; returns its rank (0-based) or −1 if it did not make the top ten. */
export function submit(entry: LeaderEntry): number {
  const boards = load();
  const key = boardKey(entry.mode, entry.venue ?? 'gym');
  const { list, rank } = insertEntry(boards[key] ?? [], entry);
  boards[key] = list;
  try {
    localStorage.setItem(KEY, JSON.stringify(boards));
  } catch {
    /* storage unavailable – the board just isn't kept */
  }
  return rank;
}
