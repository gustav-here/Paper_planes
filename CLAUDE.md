# Pappersflygplan – projektkonventioner

Webbläsarspel: vik ett pappersflygplan av ett A4 och kasta det. Flygbeteendet ska
**uppstå ur fysiken**. Se `PLAN.md` för arkitektur, datamodell och formler.

## Den viktigaste regeln
- **Inget beteende får hårdkodas per flygplanstyp.** Fysikkoden vet inte vad en dart eller
  ett glidplan är. Skillnader mellan plan kommer enbart från vikningsoperationer och gem.
- Alla aerodynamiska/fysikaliska konstanter ligger i `src/core/physics/config.ts`,
  är globala och har en kommentar med källa eller motivering.
- Ändra aldrig fysiken för att få ett enskilt preset eller test att fungera. Om ett preset
  inte flyger: ändra dess geometri (veck), som man gör med riktigt papper, och redovisa det.
- När realism står mot spelbarhet: **fråga användaren**, gissa inte.

## Arkitektur
- `src/core/` är ren TypeScript: **inga** importer av `three`, DOM, `window`, `performance`.
  Ingen `Math.random()` eller `Date` i core ⇒ deterministisk simulering.
- `src/render/` (three.js) och `src/ui/` (DOM) läser från core, aldrig tvärtom.
- Fysiken stegar med fast `dt` (standard 1/240 s) via ackumulator i `app/GameLoop.ts`,
  oberoende av bildfrekvens.
- Kastläget (`src/ui/flight/FlightMode.ts`) stegar `FlightSim` via `GameLoop.stepper` och renderar
  i en egen värld-scen (`render/World.ts`, `render/FlightView.ts`). Editorn och kastläget delar renderer
  via `Stage.setView`.
- Designer är listor av operationer (`ops`) i JSON (`format: "paperplane-design"`,
  `version`). Höj `version` och skriv migrering vid formatändringar.

## Enheter och koordinater
- SI i core (m, kg, s, rad). UI visar mm och grader – konvertera bara i UI-lagret.
- Värld: Y upp, golv y = 0. Modellrum: ovikt A4 i y = 0, centrerat i origo,
  +X = långsidan/standardnos (297 mm), +Z = höger (210 mm).
- Arkrum `(u, v)`: koordinater på det ovikta arket; varje facett minns sin polygon där.
- Rotationer med kvaternioner (kropp → värld); vinkelhastighet i kroppsram.

## Fysikantaganden (sammanfattning – detaljer i PLAN.md §5–7)
- Papper: 80 g/m², A4 ⇒ 4,99 g. Gem 1 g (punktmassa). Lufttäthet 1,225 kg/m³, g = 9,81 m/s².
- Papper har nolltjocklek i geometri och fysik; tjocklek (~0,1 mm) används bara för
  lagerordning och rendering.
- Stelt papper i etapp 1–3: veck är låsta, ingen deformation.
- Staplade koplanära lager är **en** aerodynamisk yta (union) men varje lager räknas i massan.
- Kvasistationär tunnplattemodell per ytelement: lyft enligt Helmbold (finit AR beräknad ur
  geometri och flöde), stall via mjuk blandning mot separerad platta, kordvis lastfördelning
  (tryckcentrum ¼c fäst flöde → ½c vid 90°) där framkanten bestäms av aktuellt flöde.
- Tillskottsmassa (6×6, kroppsram, strip-teori) ingår. Munk-momentet är AV som standard: det
  ingår redan i lastfördelningen med tryckcentrum vid ¼ korda (annars dubbelräknas det, se PLAN.md §15).
- Sidoförhållandet räknas per lyftsystem (ytor inom 45° från varandra delar spännvidd).
- Ingen skuggning mellan icke-koplanära ytor, ingen markeffekt, ingen vind.
- Stel kropp 6 DOF i kroppsram (ν = v_b, ω_b), RK4.
- Markkontakt: impulsbaserad med restitution och Coulomb-friktion. Officiellt resultat
  (distans, flygtid) räknas vid **första nedslaget**; studs/glid simuleras och visas separat.
- Papperskanter har typ `crease` / `seam` / `cut`. Konnektivitet går aldrig över `cut`.
  Snitt och snittändar sparas för framtida hållbarhets-/styvhetsmodell.
- Omöjliga veck (papper genom papper): varning i etapp 1, blockeras från etapp 2.
- Kaststyrka 1–15 m/s utan förstärkning.
- Klaffar (`FoldOp.control`) vrider sina ytor runt sitt veck. Effekten uppstår ur aeromodellen,
  och läget i förhållande till tyngdpunkten avgör om klaffen fungerar som höjd-, skev- eller
  canardroder. ±25° med 125°/s.
- Flexibelt papper (`DesignFile.flexible`): gångjärnsförsedda stela paneler i reducerade
  koordinater med linjärt implicit Euler. Stelt papper är standard och används av valideringen.
- Skador blir vanliga operationer (`dmg-…`) efter spelarens egna och gäller tills planet viks om.
- Simulatorer implementerar `Simulator` (`physics/simulator.ts`). Använd `prepare`/`createSimulator`
  (`physics/factory.ts`) i stället för att skapa dem direkt.

## Kod
- TypeScript `strict`. Identifierare och kodkommentarer på engelska; UI-text och
  dokumentation på svenska.
- Små rena funktioner i core; muterande "hot loops" (aero per element) får återanvända
  buffertar för prestanda men ska kapslas in.
- Inga fysikbibliotek. Små geometrihjälpare (`polygon-clipping`, `earcut`) är tillåtna.

## Tester
- Vitest i Node-miljö, `tests/core/**` speglar `src/core/**`, valideringstester i
  `tests/validation/`.
- Varje ny core-funktion får tester. Fysiktester kontrollerar invarianter (bevarad area,
  massa, rörelsemängdsmoment, determinism) snarare än magiska siffror.
- Valideringstesterna (platt ark instabilt, glidplan sjunker långsammare än dart,
  gem framåt ⇒ stabilare/nosigare) måste vara gröna efter etapp 2.

## Kommandon
- `npm run dev` – Vite-server
- `npm test` – Vitest (en gång), `npm run test:watch`
- `npm run lint`, `npm run typecheck`
- Valideringstesterna (`tests/validation`) och de flexibla testerna kör riktiga flygningar.
  Hela sviten tar ungefär en minut, och testtimeout är 60 s.

## Arbetssätt
- Bygg en etapp i taget. Varje etapp avslutas med fungerande `npm run dev`, gröna tester och
  en kort sammanfattning av vad användaren ska prova.
