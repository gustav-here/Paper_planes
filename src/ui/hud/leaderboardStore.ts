import type { GameMode, LeaderEntry } from '../../core/game/competition';
import { insertEntry } from '../../core/game/competition';

const KEY = 'paperplanes.leaderboard.v1';

type Boards = Partial<Record<GameMode, LeaderEntry[]>>;

function load(): Boards {
  try {
    const raw = localStorage.getItem(KEY);
    const b = raw ? (JSON.parse(raw) as unknown) : {};
    return b && typeof b === 'object' ? (b as Boards) : {};
  } catch {
    return {};
  }
}

export function leaderboard(mode: GameMode): LeaderEntry[] {
  return load()[mode] ?? [];
}

/** Add a result; returns its rank (0-based) or −1 if it did not make the top ten. */
export function submit(entry: LeaderEntry): number {
  const boards = load();
  const { list, rank } = insertEntry(boards[entry.mode] ?? [], entry);
  boards[entry.mode] = list;
  try {
    localStorage.setItem(KEY, JSON.stringify(boards));
  } catch {
    /* storage unavailable – the board just isn't kept */
  }
  return rank;
}
