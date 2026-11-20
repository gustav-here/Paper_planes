# Pappersflygplan

Vik ett pappersflygplan av ett A4 och kasta det. Hur planet flyger uppstår ur fysiken: veck, snitt,
gem och tejp ändrar geometri och massa, och samma aerodynamik och samma stelkropps- och
flexmodell flyger alla plan. Ingenting är hårdkodat per flygplanstyp.

**Spela:** <https://gustav-here.github.io/Paper_planes/>

## Köra lokalt

```bash
npm install
npm run dev
```

## Tester

```bash
npm test
```

`npm run lint` och `npm run typecheck` kör de övriga kontrollerna. Alla kontroller körs på GitHub
vid varje push, och spelet publiceras då på GitHub Pages.

Arkitektur, fysikmodell och beslut finns i [PLAN.md](PLAN.md), och projektets konventioner i
[CLAUDE.md](CLAUDE.md).
