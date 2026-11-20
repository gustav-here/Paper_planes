export interface PaperSpec {
  /** Short side (m). Lies along model Z. */
  widthM: number;
  /** Long side (m). Lies along model X. */
  heightM: number;
  /** Grammage (g/m²). */
  gsm: number;
}

export const A4_80GSM: PaperSpec = { widthM: 0.21, heightM: 0.297, gsm: 80 };

/** Typical office paper: ~1.25 µm per g/m² → 80 g/m² ≈ 0.1 mm. Used for rendering/stacking only. */
export const thicknessM = (spec: PaperSpec): number => spec.gsm * 1.25e-6;

/** Areal density (kg/m²). */
export const arealDensity = (spec: PaperSpec): number => spec.gsm / 1000;
