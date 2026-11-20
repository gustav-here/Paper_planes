import type { PaperErrorCode, PaperWarning } from '../../core/paper/errors';

export const ERROR_TEXT: Record<PaperErrorCode, string> = {
  degenerateLine: 'Linjen är för kort eller sidan går inte att avgöra.',
  nothingToFold: 'Det finns inget papper på den sidan av linjen.',
  noLayerAtLine: 'Inget lager ligger längs linjen – rita linjen över pappret.',
  pickNotOnMovingSide: 'Tryck på själva fliken som ska vikas.',
  tear: 'Vecket skulle riva papperet: delen sitter fast på annat ställe. Klipp loss den först eller välj ”Alla lager”.',
  nothingToCut: 'Snittet träffar inget papper.',
  clipNotOnEdge: 'Gemet måste sitta på en kant.',
  unknownClip: 'Gemet finns inte längre.',
  invalidDesign: 'Ogiltig designfil.',
};

export function warningText(w: PaperWarning): string {
  switch (w.code) {
    case 'penetration':
      return 'Varning: ett veck i filen går igenom annat papper (fysiskt omöjligt).';
    case 'mirrorUnavailable':
      return 'Spegling gick inte: modellen är inte längre symmetrisk.';
    case 'mirrorOnAxis':
      return 'Linjen ligger på symmetriaxeln – inget speglat veck behövs.';
    case 'piecesDiscarded':
      return `Snittet klippte loss ${w.detail ?? 'en'} bit(ar) som togs bort.`;
    case 'clipRemoved':
      return 'Ett gem satt på en bortklippt bit och togs bort.';
  }
}
