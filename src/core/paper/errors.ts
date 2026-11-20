/** Machine-readable error codes; the UI maps them to Swedish text. */
export type PaperErrorCode =
  | 'degenerateLine'
  | 'nothingToFold'
  | 'noLayerAtLine'
  | 'pickNotOnMovingSide'
  | 'tear'
  | 'nothingToCut'
  | 'clipNotOnEdge'
  | 'unknownClip'
  | 'invalidDesign';

export class PaperError extends Error {
  constructor(
    public readonly code: PaperErrorCode,
    message?: string,
  ) {
    super(message ?? code);
    this.name = 'PaperError';
  }
}

export type PaperWarningCode =
  | 'penetration'
  | 'mirrorUnavailable'
  | 'mirrorOnAxis'
  | 'piecesDiscarded'
  | 'clipRemoved';

export interface PaperWarning {
  code: PaperWarningCode;
  detail?: string;
}
