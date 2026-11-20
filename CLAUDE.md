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
- Stelt papper (valbart): veck är låsta, ingen deformation. Valideringstesterna använder det.
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
- Klaffar (`FoldOp.control`) vrider sina ytor runt sitt veck. Effekten uppstår ur aeromodellen.
  Varje klaff blandar höjd och skevning (elevon). Läget avgör tecknen: bakom eller i höjd med
  tyngdpunkten fungerar den som höjdroder, tydligt framför som canard, och sidan avgör skevningen.
  ±25° med 125°/s.
- Flexibelt papper (`DesignFile.flexible`): gångjärnsförsedda stela paneler i reducerade
  koordinater med linjärt implicit Euler. Stelt papper är standard och används av valideringen.
- Fold-scope `all` = alla lager vid den *dragna* linjen och det som sitter fast i dem (snitt begränsar).
- Snittflikar (`physics/slits.ts`): plant papper mellan två snittändar blir ett gångjärn med plan
  pappersböjstyvhet i flexibelt papper (fasetterna delas i `prepare`).
- Återfjädring (`paper/springback.ts`): ett släppt veck öppnar sig med 5 % av vikvinkeln. Gem och
  tejp håller vecket stängt där de sitter, och glipan växer linjärt till papperets ände (ett
  smalt V). Båda sidorna av ett veck öppnar sig från varandra. Det gäller alla kast via `prepare` (`Prepared.model` är relaxerad,
  `Prepared.folded` är exakt vikt). Validering och editor använder den vikta geometrin.
- Flexibelt papper är standard i spelet. Gångjärnen har plan pappersböjning i serie med vecket och
  ett kontaktstopp vid 180°.
- Böjformer (`physics/bendModes.ts`): stora plana paneler böjer sig själva med några släta former
  (extra frihetsgrader). Luften verkar på den böjda ytan, och kastläget ritar den böjd. Bortom
  35° lutning styvnar formerna, eftersom papper inte kan töjas. Se PLAN.md §26.
- Editorn visar modellen med återfjädring och skador. Verktygen arbetar i den vikta modellen
  via `ToolContext.hitPaper` (som returnerar en frame) och `setFrame`. Rita aldrig verktygsgrafik
  direkt i visningskoordinater.
- Platser (`core/game/venues.ts`: gymnastiksal, Månen, Solen) sätter g, ρ och kontaktytor via
  `venueConfig`. Gymnastiksalen är standard. Inget i fysiken får bero på vilken plats det är,
  utöver de parametrarna. Det gäller också markens friktion, studs och plogning
  (`ploughKgPerM`: lös regolit på Månen bromsar med ρ·A·v_t², oberoende av g).
- Procedurtexturer i `render/venues` ritas på 2D-canvas utan `filter` (det kraschade
  grafikprocessen).
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

## CI och publicering
- `.github/workflows/ci.yml` kör lint, typkontroll, tester och bygge vid varje push och pull request.
  Varje push till `main` publiceras på GitHub Pages: <https://gustav-here.github.io/Paper_planes/>.
- Vite bygger med relativa sökvägar (`base: './'`). Prova bygget under en undermapp med
  launch-konfigurationen `pages-preview` (`/Paper_planes/` på port 5174).

## Arbetssätt
- Bygg en etapp i taget. Varje etapp avslutas med fungerande `npm run dev`, gröna tester och
  en kort sammanfattning av vad användaren ska prova.
