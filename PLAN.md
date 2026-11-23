# Pappersflygplan – plan

Status: **Etapp 1–6 klara** (se §14–19). Planens senare etapper (§11) är byggda.

## 0. Grundprincip

Flygbeteendet ska **uppstå** ur geometri + massfördelning + en generell aerodynamisk modell
för tunna plattor. Fysikmotorn vet inte vad en "dart" eller ett "glidplan" är. Det enda som
skiljer två flygplan är listan av vikningsoperationer (och gem). Alla aerodynamiska konstanter
är globala och ligger i en dokumenterad config.

Förinställda modeller är *bara* JSON-listor med veck – precis det en spelare kan vika själv.

---

## 1. Arkitektur

```
            ┌───────────────────────────────────────────────┐
            │ src/core  (ren TypeScript, inga DOM/three-     │
            │            beroenden, testas i Node m. Vitest) │
            │                                               │
  Design ──▶│ paper/   vikningsmotor: ark → facetter+lager  │
  (JSON)    │    │                                          │
            │    ▼                                          │
            │ physics/airframe  massa, tyngdpunkt, tröghet, │
            │    │              aero-ytelement              │
            │    ▼                                          │
            │ physics/sim  6-DOF stel kropp, RK4, 240 Hz    │──▶ FlightLog
            └───────────────┬───────────────────────────────┘
                            │ läser state (aldrig tvärtom)
            ┌───────────────▼───────────────────────────────┐
            │ src/render (three.js)   src/ui (DOM/HUD)      │
            │ bord, papper, värld,    editorverktyg, kast,  │
            │ debug-overlays          grafer, historik      │
            └───────────────────────────────────────────────┘
```

- **Envägsberoende:** `render`/`ui` → `core`. `core` importerar aldrig three.js eller DOM.
  Upprätthålls med ESLint `no-restricted-imports` + ett test som skannar importerna.
- **Determinism:** samma design + samma kastparametrar ⇒ bit-identisk flygning (fast dt,
  ingen `Math.random`/`Date` i core). Det gör topplistor verifierbara senare (spela om).
- **Fast tidssteg:** spel-loopen har en ackumulator; fysiken stegar alltid `dt = 1/240 s`
  (konfigurerbart), med valfria interna substeg. Max antal steg per bildruta är begränsat
  (undviker "spiral of death"). Tidsskala (slow-motion) för felsökning.

### Teknik
- Vite + TypeScript (strict) + Three.js (OrbitControls från `three/examples`).
- Vitest (Node-miljö) för core.
- Små rena hjälpbibliotek i core (inte fysik): `polygon-clipping` (union av polygoner) och
  `earcut` (triangulering). Inga fysikbibliotek.
- `lil-gui` för debugpanel (justera config live).
- Inga externa bild-/ljudresurser: träbordets textur genereras procedurellt (canvas).

---

## 2. Mappstruktur

```
/
├─ index.html
├─ package.json  tsconfig.json  vite.config.ts  vitest.config.ts  eslint.config.js
├─ PLAN.md  CLAUDE.md
├─ src/
│  ├─ main.ts                       # startar App
│  ├─ app/
│  │  ├─ App.ts                     # tillståndsmaskin: Editor → Kast → Flykt → Resultat
│  │  └─ GameLoop.ts                # ackumulator, fast dt, tidsskala
│  ├─ core/                         # ← ingen three/DOM här
│  │  ├─ math/
│  │  │  ├─ vec2.ts vec3.ts mat3.ts quat.ts
│  │  │  ├─ rigidTransform.ts       # rotation (quat) + translation
│  │  │  ├─ polygon2d.ts            # area, centroid, split med linje, clip mot konvex
│  │  │  └─ linalg.ts               # 3×3 / 6×6 lösare
│  │  ├─ paper/
│  │  │  ├─ paperSpec.ts            # A4, 80 g/m², tjocklek
│  │  │  ├─ PaperModel.ts           # facetter, veckkanter, lager
│  │  │  ├─ ops.ts                  # FoldOp, ClipOp, DesignFile (+ validering av JSON)
│  │  │  ├─ applyFold.ts            # dela, rotera, lagerordning
│  │  │  ├─ applyCut.ts             # saxsnitt, lossnade bitar
│  │  │  ├─ connectivity.ts         # komponenter över crease/seam (inte cut)
│  │  │  ├─ foldCollision.ts        # (etapp 2) papper genom papper
│  │  │  ├─ movingSet.ts            # vilka facetter följer med (scope + konnektivitet)
│  │  │  ├─ symmetry.ts             # spegelveck (tvillingoperation)
│  │  │  ├─ clips.ts                # gem: kantsökning, fastklämda lager
│  │  │  ├─ edges.ts                # synliga/klämbara kanter
│  │  │  └─ replay.ts               # ops → modell, cache för ångra/gör om
│  │  ├─ physics/
│  │  │  ├─ config.ts               # ALLA fysikkonstanter, dokumenterade
│  │  │  ├─ massProperties.ts       # massa, tyngdpunkt, tröghetstensor
│  │  │  ├─ aeroMesh.ts             # koplanära grupper → union → ytelement
│  │  │  ├─ aeroModel.ts            # koefficienter, kordvis lastfördelning
│  │  │  ├─ airframe.ts             # PaperModel+gem → Airframe (kroppsfrihet)
│  │  │  ├─ addedMass.ts            # 6×6 tillskottsmassa + Munk-moment
│  │  │  ├─ rigidBody.ts            # 6-DOF tillstånd + derivator
│  │  │  ├─ integrator.ts           # RK4
│  │  │  ├─ groundContact.ts        # impulsbaserad kontakt: studs + friktion
│  │  │  ├─ simulation.ts           # värld, steg, landningsfaser, FlightLog
│  │  │  ├─ launch.ts               # kastparametrar → initialtillstånd
│  │  │  └─ windTunnel.ts           # fastlåst kropp: krafter/moment vs α, β
│  │  └─ presets/
│  │     ├─ flatSheet.json  dart.json  glider.json
│  │     └─ index.ts
│  ├─ render/
│  │  ├─ SceneSetup.ts  Table.ts  WoodTexture.ts  Lighting.ts
│  │  ├─ PaperMesh.ts               # facetter → BufferGeometry, lager-offset, veckelinjer
│  │  ├─ XRay.ts                    # genomskinlighet för dolda veck
│  │  ├─ ClipMesh.ts  World.ts (golv, mätlinjer)  CameraRig.ts (orbit/följ)
│  │  └─ DebugOverlay.ts            # tyngdpunkt, hastighet, krafter per element
│  └─ ui/
│     ├─ editor/ FoldTool.ts  CutTool.ts  AngleDrag.ts  Snapping.ts  ClipTool.ts
│     │          ViewControls.ts  History.ts
│     ├─ throw/ ThrowControls.ts
│     └─ hud/ Toolbar.ts  TimeSeriesPlot.ts  FlightLogPanel.ts  styles.css
└─ tests/
   ├─ core/…                        # enhetstester speglar src/core
   └─ validation/                   # dart vs glidplan, platt ark, gem
```

---

## 3. Koordinatsystem och enheter

- **SI överallt i core** (m, kg, s, rad). UI visar mm och grader.
- **Världen:** Y upp (som three.js), golv `y = 0`.
- **Modellrum (editorn):** arket ligger plant i `y = 0`, centrerat i origo, stående A4:
  - `+X` = arkets långa riktning bort från spelaren (standardnos), x ∈ [−0.1485, 0.1485]
  - `+Z` = höger, z ∈ [−0.105, 0.105]
  - `+Y` = upp ur bordet (arkets framsida, "framsida" är spårbar efter vändning)
- **Arkrum (flat):** 2D-koordinater `(u, v)` på det ovikta arket; varje facett minns sin
  polygon här (behövs för gem, symmetri, framtida gångjärn).
- **Kroppsram (flygning):** modellrummet förskjutet till tyngdpunkten. Axlarna behålls
  (full tröghetstensor, ingen diagonalisering). Nos/upp anges i designfilen (standard +X/+Y)
  och kan vändas av spelaren.

---

## 4. Datamodell (vikning)

### 4.1 Pappersmodell

```ts
interface Facet {
  id: number;
  flat: Vec2[];            // konvex polygon i arkrum (CCW sett från framsidan)
  T: RigidTransform;       // arkrum (u,v,0) → modellrum
  layer: number;           // stapelordning bland koplanära, överlappande facetter
  foldPath: string[];      // id för veck-operationer som flyttat facetten
}
type EdgeKind =
  | "crease"               // veck: gångjärn, papperet sitter ihop
  | "seam"                 // platt hjälpkant (facett delad men opåverkad), sitter ihop
  | "cut";                 // saxsnitt: papperet sitter INTE ihop här
interface Edge {           // kant mellan två facetter (blir gångjärn i senare etapper)
  kind: EdgeKind;
  a: number; b: number;    // facett-id
  flatSegment: [Vec2, Vec2];
  opId: string;
  dihedral: number;        // nuvarande vinkel (rad), 0 = platt
}
interface PaperModel { spec: PaperSpec; facets: Facet[]; edges: Edge[]; clips: Clip[];
                       cutEnds: CutEnd[] }   // snittändar = framtida spänningskoncentration
```

All konnektivitet (vad som följer med i ett veck, vad som lossnar vid klipp) räknas via
kanter av typ `crease`/`seam`, aldrig över `cut`.

Facetterna hålls **konvexa** (ett konvext A4 delat av räta linjer förblir konvext) ⇒ enkel
och robust delning. Nollttjocklek i geometrin; renderingen förskjuter lager med
`layer × tjocklek` längs normalen (tjocklek 80 g/m² ≈ 0,1 mm, visuellt överdrivet).

### 4.2 Operationer (sparas som JSON, spelas upp deterministiskt)

```jsonc
{
  "format": "paperplane-design", "version": 1,
  "name": "Min dart",
  "paper": { "widthM": 0.210, "heightM": 0.297, "gsm": 80 },
  "orientation": { "nose": [1,0,0], "up": [0,1,0] },
  "ops": [
    {
      "type": "fold", "id": "f3",
      "line": { "p0": [x,y,z], "p1": [x,y,z] },   // modellrum, på arbetsplanet
      "planeNormal": [0,1,0],    // arbetsplanets normal = sidan spelaren ser
      "movingSide": [0,0,1],     // riktning (i arbetsplanet) mot den sida som viks
      "kind": "valley",          // valley | mountain, relativt planeNormal
      "angleDeg": 180,           // 5 … 180
      "scope": "all",            // all | top | bottom | flap  (se 4.3)
      "pick": [x,y,z],           // (scope flap) punkt på fliken som ska vikas
      "mirror": true             // skapar tvillingveck enligt symmetrin (se 4.5)
    },
    { "type": "cut", "id": "k1",
      "path": [[x,y,z], [x,y,z]],  // polylinje i modellrum på arbetsplanet
      "planeNormal": [0,1,0],
      "depth": "all",              // all | top  (genom hela stapeln eller översta lagret)
      "mirror": true,
      "discard": [{ "flat": [u,v] }] }, // bitar som lossnat och kastas (punkt i arkrum)
    { "type": "clip", "id": "c1", "at": [u,v], "edgeDir": [du,dv], "massKg": 0.001 },
    { "type": "unclip", "id": "u1", "target": "c1" }
  ]
}
```

- Ett veck lagras med all information som behövs för uppspelning; dal/berg är tolkat
  mot `planeNormal`, så vändning/rotation av vyn påverkar inte uppspelningen.
- **Ångra/gör om:** historiken är `ops[]` + markör. Modellen efter varje op cachas
  (små modeller), så ångra är O(1) och gör om spelar bara upp en op.
- JSON valideras vid inläsning (version, typer, intervall) med tydliga fel.

### 4.3 Vikningsalgoritmen (`applyFold`)

1. **Arbetsplan:** planet för den facett spelaren började dra linjen på (för ett plant ark =
   bordsplanet). Snittytan är planet som innehåller linjen och är vinkelrätt mot arbetsplanet.
2. **Dela:** varje facett som snittytan korsar delas i två konvexa facetter (i arkrum, via
   `T⁻¹`); en ny `CreaseEdge` skapas mellan halvorna.
3. **Rörlig mängd (`movingSet`)**:
   - `scope: "all"` (förvalt, enligt kravet): alla facetter på den rörliga sidan.
   - `scope: "top"`/`"bottom"`: lagret närmast spelaren (resp. längst bort) vid linjen, plus
     **sluten konnektivitet**: varje facett på rörliga sidan som är förbunden med en rörlig
     facett via en veckkant som *inte* ligger på vecklinjen måste också följa med.
     Behövs för klassiska modeller (vik ena vingen, sedan andra), inte bara för bonus.
   - `scope: "flap"`: den sammanhängande komponent (över `crease`/`seam`, inte över vecklinjen
     eller `cut`) som innehåller den klickade punkten. Det är så man viker en urklippt flik,
     t.ex. ett höjdroder mellan två snitt.
   - **Rivningskontroll:** om den rörliga mängden sitter ihop med den stillastående via en
     kant som *inte* ligger på vecklinjen skulle papperet rivas ⇒ vecket nekas med förklaring.
   - Nya kanter på vecklinjen mellan två facetter som *båda* står still blir `seam` (platta),
     så ett lokalt flikveck inte lämnar falska veck över resten av vingen.
4. **Rotera** rörliga facetter kring linjen med vinkeln θ (riktning ges av dal/berg).
5. **Lagerordning:** vid θ = 180° hamnar de rörliga facetterna koplanärt med de stillastående.
   Dal: nya lager = `max + 1 …` i **omvänd** ordning (det som var underst hamnar överst).
   Berg: motsvarande under `min`. Vid θ < 180° behåller de rörliga facetterna sin inbördes
   (omvända) ordning; lager jämförs bara mellan koplanära facetter.
6. Uppdatera `dihedral` för nya och berörda veckkanter.

Veck på en redan 3D-formad modell (efter ett veck < 180°) fungerar med samma algoritm
eftersom snittet är ett plan i 3D. Fysiskt omöjliga veck (papper genom papper) **varnas**
för i etapp 1 och **blockeras från etapp 2** (beslut, §13): kollisionskontroll mellan
facetter under hela rotationen (triangel–triangel i 3D, samt lagerordningskonsistens för
platta koplanära staplar). Under vinkel-draget stannar animationen vid första kontakt.

### 4.4 Interaktion i editorn

- Pappret ligger på ett träbord, orbit-kamera (höger-/mittknapp), vänsterknapp = verktyg.
- **Vikverktyg:** dra en linje över pappret → linjen snäpper mot hörn, kantmittpunkter,
  befintliga veck, mittlinjen och 15°-vinklar (Shift stänger av snäpp). Hjälpläge
  **"punkt till punkt"**: klicka punkt A och B ⇒ vecklinjen = mittnormalen (t.ex. "vik
  hörnet till mittlinjen") – nödvändigt för att vika exakt med mus.
- Liten verktygsrad vid linjen: **Dal / Berg**, **Spegla**, **Lager: alla/översta/understa**,
  vilken sida som viks (klicka på sidan eller pilen).
- **Vinkel:** håll nere musknappen och dra fram/tillbaka ⇒ vinkel 5°–180° med live-animation
  av pappret; snäpp vid 45/90/135/180 (Shift = fritt). Släpp ⇒ vecket sparas.
  Esc avbryter.
- **Vända/rotera:** knappar "Vänd (runt långsida/kortsida)" och "Rotera 90°", animeras.
  Ändrar bara vyn, inte modellen.
- **Ångra/gör om:** Ctrl/Cmd+Z, Shift+Ctrl/Cmd+Z, samt knappar.
- **Genomskinlighet:** när ett speglat veck hamnar på en dold sida (bakom lager) ritas det
  med streckad "röntgen"-linje som syns genom pappret, och pappret blir svagt genomskinligt
  i området. Global "röntgen"-knapp finns också.
- **Saxverktyg:** se 4.6.
- **Gemverktyg:** hovra nära en klämbar kant ⇒ förhandsvisat gem som glider längs kanten;
  klick placerar. Klämbara kanter = konturen av varje koplanär lagerstapel (pappersranden
  och 180°-veck, t.ex. kölens underkant eller nosen). Gemet klämmer alla lager vars facetter
  överlappar gemets fotavtryck (djup ~8 mm, config) och sparas som op. Gem kan tas bort.
- **Spara/ladda:** exportera/importera JSON, samt förinställda modeller i en meny.
- **Nos/upp:** pil som visar nosriktning; kan vändas.

### 4.5 Spegelveck (samma veck på båda sidor)

Arket har en naturlig symmetri σ: spegling i mittlinjen (z → −z i arkrum). Så länge modellen
är symmetrisk finns en isometri **R** i modellrummet som avbildar varje facett på sin
σ-tvilling (beräknas från valfritt facettpar: `R = T_f' ∘ σ ∘ T_f⁻¹`). Tvillingvecket får:
linje = R(linje), rörlig mängd = σ(rörlig mängd), vinkelriktning speglad.

- Efter vikning på mitten 180° blir R spegling *genom pappersplanet* – då blir tvillingen
  automatiskt "samma linje, understa lagren, motsatt riktning", dvs. den andra vingen.
- Efter mittvikning 90° blir R spegling i bisektrisplanet (den öppna kölen).
- Om modellen inte längre är symmetrisk (osymmetriska veck) är Spegla avstängd med förklaring.
- Spegling gäller även snitt (4.6), så ett höjdroder kan klippas på båda sidor i ett drag.

### 4.6 Saxverktyg (snitt)

Ett snitt är en rak linje eller polylinje som spelaren drar över pappret (samma snäpp som
vikverktyget). Typiskt: två korta parallella snitt in från bakkanten ⇒ en flik som sedan viks
med `scope: "flap"` till höjdroder/klaff.

Algoritm (`applyCut`), per facett som snittet korsar (genom hela stapeln, eller bara översta
lagret med `depth: "top"`):
1. Facetten delas längs snittsegmentets **förlängda** linje så att facetterna förblir konvexa.
2. Den del av den nya kanten som snittet faktiskt täcker blir `cut`; resten blir `seam`.
   (En kant kan alltså delas i flera kantstycken av olika typ.)
3. Snittets ändpunkter inne i papperet sparas som `CutEnd` (position i arkrum, riktning).
4. Om snittet gör att en bit lossnar helt (ingen `crease`/`seam`-väg till resten) frågar
   editorn vilken bit som ska behållas (förval: största); övriga kastas och sparas i
   `discard` så att uppspelningen blir deterministisk.

Konsekvenser:
- Massa/tröghet: bortklippta bitar försvinner automatiskt (massa räknas ur facetterna).
- Aero: en flik som viks ut ur planet blir en egen yta med egen fram-/bakkant; ett ovikt
  snitt ändrar nästan ingenting (unionen sluter det), vilket är fysiskt rimligt.
- Gem kan inte klämma över ett snitt som saknar papper (gemets fotavtryck klipps mot facetterna).
- **Senare (hållbarhet):** `cut`-kanter saknar styvhet; `CutEnd` blir spänningskoncentration
  där rivning kan starta vid hårda kast, och styvheten i `seam`/`crease` nära snitt reduceras.
  Datamodellen bär redan allt som behövs för det.

---

## 5. Massa, tyngdpunkt, tröghet (etapp 2)

- Ytdensitet `ρ_A = 0.080 kg/m²`. A4: `A = 0.06237 m²` ⇒ **m_papper = 4.99 g**.
  Vikning bevarar area ⇒ massan är oberoende av vikningen (test). Saxsnitt som kastar
  bitar minskar massan med exakt den bortklippta arean (test).
- Varje facett trianguleras. För en tunn triangel med hörn a, b, c (relativt origo), massa m:

  `C = (m/12)·(aaᵀ + bbᵀ + ccᵀ + ssᵀ)`, `s = a + b + c`  (andra momentet ∫x xᵀ dm)

  `I_o = tr(C)·𝟙 − C`.
- Gem: punktmassa 1 g placerad i gemets masscentrum (kant + halva klämdjupet inåt).
- Tyngdpunkt: `r_cm = Σ mᵢ rᵢ / Σ mᵢ`; tröghet flyttas till tyngdpunkten med
  Steiners sats: `I_cm = I_o − m(|r_cm|²𝟙 − r_cm r_cmᵀ)`.

## 6. Aerodynamik (etapp 2)

### 6.1 Aero-ytor och element
1. Gruppera facetter som är **koplanära** (samma plan inom tolerans, normal ±).
2. Ta **unionen** av deras polygoner i planet ⇒ en aero-yta. (Staplade lager är *en* platta
   för luften men räknas i massan för varje lager.)
3. Dela ytan i element genom att klippa unionen mot ett rutnät (standard 10 mm, config).
   Varje element: centrum `rₑ` (kroppsram, relativt CoM), normal `nₑ`, area `Aₑ`, ytans id.
4. Ytans kontur sparas för kordberäkning (6.3).
Skuggning mellan icke-koplanära ytor (t.ex. köl som skymmer vinge) ignoreras – dokumenterat
antagande.

### 6.2 Per element och utvärdering (stilla luft, vind = 0)

```
uₑ   = v + ω × rₑ                     (elementets hastighet mot luften)
q    = ½ ρ |uₑ|²                       ρ = 1.225 kg/m³
α    = asin(|nₑ · ûₑ|)  ∈ [0, π/2]     anfallsvinkel mot plattan
ê_D  = −ûₑ
n_p  = −sign(nₑ·ûₑ)·nₑ                 (trycksidans normal)
ê_L  = normalize(n_p − (n_p·ûₑ) ûₑ)    lyftriktning ⟂ flödet
```
Plattan behandlas symmetriskt (båda sidor lika) – inget hårdkodat "upp".

**Effektivt sidoförhållande** per yta, varje steg: flödesriktningen projiceras in i ytans plan,
spännvidd `b` = ytans utsträckning vinkelrätt mot den, `AR = b²/S`. (Härleds ur geometri +
flöde, inte per flygplanstyp.)

**Koefficienter** (plattvingeteori + stall-blandning):
```
a        = a₀ · AR / (2 + √(AR² + 4))           Helmbold, a₀ = 2π·k_Re
C_L,att  = a · sin α
C_D,att  = C_D0 + C_L,att² / (π e AR)
C_L,sep  = C_N90 · sin α · cos α                  separerad platta (normalkraft)
C_D,sep  = C_D0 + C_N90 · sin² α
σ(α)     = 1 / (1 + exp(−M(α − α_s)))           mjuk övergång vid stall
C_L      = (1−σ) C_L,att + σ C_L,sep            (samma för C_D)
```

### 6.3 Kordvis lastfördelning (avgörande för att stabilitet ska uppstå)

Om varje element bara fick sin kraft i sitt eget centrum skulle tryckcentrum hamna i
ytans tyngdpunkt ⇒ ett platt ark vore neutralt stabilt, vilket är fel. Tunnplatteteori ger
lasten koncentrerad nära **framkanten** (tryckcentrum vid ¼ korda). Därför:

- För varje element: dra en linje genom centrum längs flödets riktning i ytans plan,
  skär med ytans kontur ⇒ lokal framkant/bakkant ⇒ kordkoordinat ξ ∈ [0,1] för elementets
  främre/bakre utsträckning [ξ₁, ξ₂]. **Framkanten bestäms av det aktuella flödet**, så samma
  kod ger rätt resultat för vinge, köl, sidvind och baklängesflygning.
- Fäst flöde (tunn platta), normaliserad fördelning med tryckcentrum vid ¼:
  `g_att(ξ) = (2/π)·√((1−ξ)/ξ)`, elementvikt
  `ḡ_att = (G(ξ₂) − G(ξ₁)) / (ξ₂ − ξ₁)`, `G(ξ) = (2/π)(asin√ξ + √(ξ(1−ξ)))`
  (integrerad, så singulariteten vid framkanten hanteras exakt).
- Separerat flöde: linjär fördelning `g_sep = 1 + k(α)(1 − 2ξ)`, `k = 6·Δ_cp·cos α` (ger
  `x_cp = 0,5 − Δ_cp·cos α`, och `k ≤ 1` håller fördelningen positiv), vilket
  flyttar tryckcentrum från ≈0,34c (strax efter stall) mot 0,5c vid 90° (Fage & Johansen-trend).
- `ḡ = (1−σ) ḡ_att + σ g_sep`.

**Elementkraft:**
`Fₑ = q Aₑ [ ḡ (C_L ê_L + (C_D − C_D0) ê_D) + C_D0 ê_D ]`
(friktionsmotståndet fördelas jämnt, tryckdelen enligt kordfördelningen)

Moment kring tyngdpunkten: `τ = Σ rₑ × Fₑ`. Dämpning i rotation uppstår av sig själv via
`ω × rₑ`. Cache: kordkoordinater räknas om bara när flödesriktningen i kroppsramen ändrats
mer än ~1° (prestanda).

### 6.4 Config (`src/core/physics/config.ts`, varje värde med källa/motivering)

| Parameter | Standard | Kommentar |
|---|---|---|
| `rho` | 1.225 kg/m³ | ISA havsnivå |
| `g` | 9.81 m/s² | |
| `paperGsm` | 80 g/m² | |
| `paperThicknessM` | 1.0e-4 | endast rendering/lagerordning |
| `clipMassKg` | 0.001 | |
| `CLIP_SPEC` | 28 × 8 mm | gemets fotavtryck (längd in från kanten × bredd); masscentrum halva längden in |
| `a0` | 2π · 0.9 | tunn platta vid Re ≈ 3·10⁴ ger något under 2π |
| `alphaStallDeg` | 10 | tunna plattor stallar tidigt vid låg Re |
| `stallBlendM` | 25 /rad | övergångens skärpa |
| `CN90` | 1.2 | platta med lågt AR (Hoerner: 1.17 kvadrat → 1.98 oändlig) |
| `CD0` | 0.02 | laminär friktion båda sidor, Re ~ 3·10⁴ |
| `oswaldE` | 0.8 | |
| `sepCopShift Δ_cp` | 0.16 | tryckcentrumsförskjutning i separerat flöde |
| `elementSizeM` | 0.010 | rutnät för ytelement |
| `dt` | 1/240 s | substeg konfigurerbart |
| `addedMass` | på | tillskottsmassa + Munk-moment (§7) |
| `groundRestitution` | 0.15 | papper studsar knappt |
| `groundFriction` | 0.4 | Coulomb μ, papper mot golv (ca) |

Alla värden kan justeras live i debugpanelen men är globala – aldrig per modell.

## 7. Stel kropp, 6 frihetsgrader (med tillskottsmassa)

Tillstånd: position `x` (värld), orientering `q` (kvaternion kropp→värld) och generaliserad
hastighet i kroppsram `ν = (v_b, ω_b)`. Kroppsramen behövs eftersom tillskottsmassan är
konstant där.

```
ẋ = R(q) v_b
q̇ = ½ q ⊗ (0, ω_b)
(M_RB + M_A) ν̇ + C_RB(ν) ν + C_A(ν) ν = τ_aero + τ_grav + τ_kontakt
```
- `M_RB = [m𝟙 0; 0 I_b]`, `C_RB` ger Eulers gyroterm `ω × Iω` och `ω × m v`.
- **Tillskottsmassa `M_A` (6×6, kroppsram, beräknas en gång per airframe):** per aero-yta s
  med normal n och tyngdpunkt r_s, strip-teori längs ytan:
  `m_a,s = κ(AR)·ρ·(π/4)·Σ cᵢ² Δbᵢ` (2D-platta: ρπc²/4 per spännviddsmeter), med
  3D-korrektion `κ(AR) = AR/√(1+AR²)`, samt rotationstillskott `ρπ Σ cᵢ⁴ Δbᵢ / 128` kring ytans
  spännviddsaxel. Bidraget läggs in med koppling via `r_s × n` så att även rotation kring
  tyngdpunkten accelererar luft. Endast normalriktningen får tillskott (tunn platta).
- `C_A(ν)` (Kirchhoff/Fossen-form) ger bl.a. **Munk-momentet**, som vill vrida en platta
  tvärs mot flödet – avgörande för att ett platt ark ska fladdra/tumla realistiskt.
- Varje deriveringsanrop löser ett 6×6-system (Cholesky; M är symmetrisk positiv definit).
- Kvasistationära aerokrafter (§6) och tillskottsmassa är separata termer; ingen dubbelräkning
  eftersom §6 bara beror på hastighet, `M_A` bara på acceleration.
- Flagga `addedMass: true/false` i config (standard **på**) för jämförelse och tester.
- Integrator: **RK4**, dt = 1/240 s, kvaternionen normaliseras varje steg.
  Uppskattad stelhet: tidskonstant för rotationsdämpning ≈ 10 ms ⇒ dt/τ ≈ 0,4, stabilt med RK4
  (tillskottsmassan ökar den effektiva trögheten och gör det snarare stabilare).
  Substeg (t.ex. 2× ⇒ 480 Hz) kan slås på om testerna visar behov.

### 7.1 Markkontakt: nedslag, studs och glid

- Kontaktpunkter = hörnen på modellens facetter (deduplicerade), i kroppsram.
- **Impulsbaserad kontakt** efter varje RK4-steg: för punkter med `y ≤ 0` och hastighet
  in i golvet löses normalimpuls med restitution `e` och friktionsimpuls med Coulomb-gräns
  `|J_t| ≤ μ J_n` (sekventiella impulser, några iterationer, full stelkroppsrespons med
  `M_RB + M_A`), plus liten positionskorrigering mot genomträngning.
  Impulsmetoden valdes framför fjäder/dämpare: ett 5 g-objekt mot en styv fjäder kräver
  orimligt små tidssteg.
- Aerokrafterna verkar hela tiden, även när planet glider på golvet.
- Faser i FlightLog: `flight` → `firstImpact` (tid, position, fart) → `bouncing/sliding`
  → `rest` (|v| < 2 cm/s och |ω| < 0,1 rad/s i 0,5 s) eller timeout.
- **Officiellt resultat = första nedslaget** (distans och flygtid). Glid-/studssträcka och
  vilopunkt loggas och visas separat.
- Config: `groundRestitution` 0,15, `groundFriction` μ 0,4 (papper mot trä-/linoleumgolv, ca).
- **Vindtunnel** (`windTunnel.ts`): lås kroppen, svep α/β, få ut krafter, moment, C_m(α),
  tryckcentrum och trimvinkel. Används i tester och debugläget.

## 8. Kast och värld (etapp 3)

- Kastparametrar: **riktning** (kursvinkel), **elevation** (−30°…+45°), **styrka** (fart,
  se öppen fråga 3), höjd vid släpp 1,6 m (config). Nosen pekar längs kastriktningen, vingarna
  vågräta (planets "upp" uppåt), ω₀ = 0. Valfritt reglage: anfallsvinkel vid släpp.
- UI: dra en "slangbella"-vektor från planet eller använd reglage; förhandsvisad bana (utan
  aero, bara som riktningsstöd – kan stängas av).
- Värld: stort golv, mätlinjer var 1 m med etiketter (var 5 m tydligare), kast-linje.
  Kamera följer planet mjukt (följ/fri/sidovy).
- **FlightLog:** flygtid och horisontell distans till **första nedslaget** (släppunkt →
  tyngdpunktens projektion), maxhöjd, därefter glid-/studssträcka och vilopunkt, tidsserie
  (t, x, v, q, ω, α_medel, fas). Senaste kast sparas i localStorage.
- **Visualisering av landning:** markör + mätetikett vid första nedslaget, spår (trail) under
  flykten i en färg och under studs/glid i en annan, markör vid vilopunkten, och
  "nedslagsfart". Kameran fortsätter följa tills planet ligger still.
- **Felsökningsläge:** tyngdpunkt (markör), hastighetsvektor, lyftkraft (grön) och
  motstånd (röd) per element (skalbara pilar), tryckcentrum, graf över höjd och fart mot tid
  (egen enkel canvas-plot), slow-motion/paus/stega.

## 9. Validering (tester)

Förinställda modeller som op-listor:
- `flatSheet.json` – inga veck.
- `dart.json` – klassisk dart: hörn till mittlinjen, igen till mittlinjen, vik på mitten
  (berg), vingar (spegelveck, `scope: top`) med vinkel så att vingarna blir vågräta.
- `glider.json` – bredvingat glidplan: nosen vikt ner flera gånger (tung framkant), låsflik,
  vik på mitten, låg köl, breda vingar.
- Om ett preset behöver ett litet **höjdroder** (bakkanten uppvikt 10–20°) så är det ett
  vanligt veck i op-listan – exakt som en människa trimmar ett riktigt plan. Det är geometri,
  inte hårdkodat beteende. Jag redovisar öppet om/var sådan trim behövs.

Testfall (Vitest, headless):
1. **Platt ark glider inte stabilt:** kastat vågrätt 4 m/s. Kriterium för "stabil glidning"
   (gemensam funktion för alla tester): över ett fönster på 1 s är stigningsvinkelns
   standardavvikelse < 3°, |ω| < 0,5 rad/s och glidtalet L/D > 2. Platt ark ska *inte*
   uppfylla det (det tumlar/fladdrar). Dart och glidplan *ska* uppfylla det (sanity).
2. **Glidplan har lägre sjunkhastighet än dart:** för varje modell sveps kastfarten; i stabil
   glidning mäts medelvärdet av −v_y. min(glidplan) < min(dart).
3. **Gem framåt:** gem på nosen vs. utan gem (och gem längre bak):
   - vindtunnel: `dC_m/dα` blir mer negativ (större statisk stabilitetsmarginal),
   - trimmad anfallsvinkel minskar ⇒ högre glidfart / brantare bana i simuleringen.
Övriga enhetstester: vecklinjer delar korrekt, area bevaras, vik + omvänd vik = identitet,
lagerantal efter mittvik = 2, JSON-rundresa + deterministisk uppspelning, spegelveck ger
symmetrisk modell, massa = 4,99 g oavsett vikning, tröghet för platt ark = m(a²+b²)/12 osv.,
fritt fall utan luft = ½gt², momentfri rotation bevarar rörelsemängdsmoment och energi,
platta i vindtunnel: tryckcentrum ≈ 0,25c vid litet α och → 0,5c vid 90°, rotationsinvarians.

## 10. Etapper och leverabler

Varje etapp avslutas med fungerande `npm run dev`, gröna `npm test` och en kort lista
över vad du ska prova.

**Etapp 1 – Vikningseditor**
1. Projektskelett (Vite/TS/Vitest/ESLint), `git init`, matematikbibliotek + tester.
2. Vikningsmotor i core: dela/rotera/lager, scope, spegling, gem, JSON, replay + tester.
3. Rendering: träbord, papper med lager-offset och veckelinjer, orbit-kamera, röntgen.
4. Editor-UI: linje med snäpp, punkt-till-punkt, dal/berg, vinkel-drag med animation,
   vända/rotera, ångra/gör om, spegling, **sax**, gem, spara/ladda, presets (dart, glidplan),
   varning för omöjliga veck.

**Etapp 2 – Massa och aerodynamik**
1. Massegenskaper + tester. 2. Aero-ytor/element + tester. 3. Aeromodell + vindtunnel.
4. Stel kropp + **tillskottsmassa** + RK4 + simulation headless.
5. **Markkontakt** (studs + glid) + tester (energi minskar, vila uppnås, friktionsgräns).
6. **Kollisionskontroll för veck** – omöjliga veck blockeras i editorn.
7. Valideringstester (sektion 9), körs med tillskottsmassa på.
8. Minimal visning i editorn: tyngdpunkt och aero-element som overlay.

**Etapp 3 – Kast och felsökning**
Kastläge (1–15 m/s), värld, följkamera, felsökningsläge med pilar och grafer, FlightLog,
visualisering av nedslag/studs/glid.

## 11. Förberett för senare etapper (byggs inte nu)

- **Styrning/klaffar:** aero-element bär `surfaceId` och vilken facett/veck-op de härstammar
  från. En klaff = ett veck vars vinkel blir en körtidsparameter; elementen transformeras med
  gruppens transform i stället för att nätet byggs om. `foldPath` på facetter gör det möjligt.
- **Flexibla veck / PBD:** `CreaseEdge` med `dihedral` finns redan och blir gångjärn med
  fjäderstyvhet. Aeromodellen tar bara emot `{position, hastighet, normal, area, yta}` per
  element och bryr sig inte om kroppen är stel eller deformerbar. Kroppen ligger bakom ett
  `Body`-gränssnitt (`elementStates()`, `applyForces()`, `step()`) så att en PBD-kropp kan
  ersätta `RigidBody`. Gem har redan sina fastklämda lager registrerade (blir constraints).
- **Hållbarhet och snitt:** `cut`-kanter har ingen styvhet, `CutEnd` markerar var rivning kan
  starta; styvheten i närliggande veck/sömmar sänks som funktion av avstånd till snittändar.
  Plastisk deformation/kollaps vid hårda nedslag använder kontaktimpulserna från §7.1.
- **Tävlingslägen:** `FlightLog` + deterministisk simulering ⇒ ett resultat kan verifieras
  genom att spela upp design-JSON + kastparametrar. `GameMode`-gränssnitt (poängsättning,
  måltavla) och topplista (localStorage först, backend senare).

## 12. Risker

- Kvasistationär plattmodell är en förenkling: lågt Reynoldstal, virvellyft på deltavingar
  och instationära effekter saknas. Målet är rätt **trender** och spelbar realism, inte CFD.
- Presets kan behöva justerad geometri (t.ex. vinkel på vingveck) för att flyga – samma sak
  som med riktiga pappersflygplan. Jag ändrar aldrig fysiken per modell för att få ett test grönt.
- Polygonunion av många lager kan ge numeriskt brus ⇒ toleranser och snäpp till arkrum.

## 13. Beslut (realism vs spelbarhet)

1. **Tillskottsmassa** byggs i etapp 2 (§7), standard på.
2. **Golvkontakt:** resultat räknas vid första nedslaget, men studs och glid simuleras och
   visualiseras (§7.1, §8).
3. **Kaststyrka:** 1–15 m/s, ingen förstärkning.
4. **Omöjliga veck:** varning i etapp 1, blockeras i etapp 2.
5. **Språk:** kod och kommentarer på engelska, UI och dokumentation på svenska.
6. **Saxverktyg** i etapp 1 (§4.6); snittens effekt på hållbarhet/styvhet tas med i en senare
   etapp (§11), datamodellen förbereds nu.

## 14. Avvikelser i genomförandet (etapp 1)

- **Kanter lagras inte** utan härleds ur geometrin (`computeEdges`): facetter som delar en
  kant i arkrummet är förbundna (`crease` om transformerna skiljer sig, annars `seam`), om inte
  ett lagrat snitt (`model.cuts`) täcker kanten (`cut`). Det tar bort en hel klass av
  synkfel. Snittändar räknas fram ur `model.cuts` när de behövs.
- **Lösa flikar som ligger inklämda** mellan lager som viks följer med (`scope: top/bottom/flap`).
  Det behövs för klassiska modeller, t.ex. dartens hörnflikar vid vingvecket.
- **Varning för omöjliga veck** i etapp 1: en flik som lämnar sin stapel åt ett håll där det
  ligger papper, samt ett samplat svep där facetter skär varandra under rotationen.
- Gem-op: `at` + `edgeDir` i arkrum (i stället för `facet.flat`). Gemets mått finns i
  `CLIP_SPEC` (flyttas till fysikkonfigen i etapp 2). Borttagning är en egen op (`unclip`).
- Presets är TypeScript-moduler (`src/core/presets/index.ts`) som bygger design-objekt med
  beräknade koordinater; de kan exporteras som JSON med Spara.
- Transformer heter `src/core/math/transform.ts` (rigid + affin/spegling).
- Designens `orientation` (nos/upp) används redan i etapp 1 för att visa laddade modeller
  upprätt; UI för att ändra den kommer i etapp 3.
- Efter etapp 1 (önskemål): startvyn är rakt ovanifrån så att arket syns som en oförvrängd
  A4-rektangel (kameran anpassas till fönstret och HUD:en), Vänd-knapparna vänder 90° per
  tryck, och ett gem tas bort med högerklick oavsett verktyg (osynlig träffyta runt gemet).

## 15. Avvikelser och resultat i etapp 2

- **Munk-momentet är avstängt som standard** (`munkMoment: false`). För en tunn platta är
  potentialströmningens moment kring mittkordan exakt lika med Munk-momentet, och det ingår
  redan via lastfördelningen med tryckcentrum vid ¼ korda. Med båda påslagna räknades samma
  destabiliserande moment två gånger: planen stegrade sig med cirka 22 rad/s på 0,1 s. Tröghet från
  tilläggsmassan (6×6, strip-teori) ingår fullt ut.
- **Sidoförhållande per lyftsystem:** ytor vars plan ligger inom 45° från varandra (t.ex.
  två vingar med V-form) delar spännvidd och area när AR beräknas. Annars skulle en
  V-form på 1° plötsligt halvera AR. Regeln är global och bygger enbart på geometrin.
- **Polygonunioner** (`polygon-clipping`) avrundas till ett fint rutnät (0,1–10 µm) vid
  numeriska problem i tjocka staplar.
- **Markkontakt:** genomträngning korrigeras med Baumgarte-hastighet med glapp, plus ett golv på 1 mm
  vid hårda nedslag. Vila detekteras på glidande medelvärden.
- **Presets fick höjdroder** (snitt 3 mm utanför vingroten + uppvikt flik 10°). Utan dem trimmar
  plana vingar vid α≈0 och dyker, precis som riktiga plan. Endast geometri ändrades.
- **Omöjliga veck blockeras:** en sökning med 1° steg ger vinkeln för första kontakt, och animationen
  stannar där. Kollisionstestet skär nu polygonerna längs plan–plan-linjen (fångar också
  korsningar där kanter sammanfaller).
- **Fysikpanel i editorn** (knappen ⚖ eller I): massa, tyngdpunkt, ytelement, medflyttad luft,
  trimvinkel, Cmα och glidtal vid trim, samt val av nos- och upp-riktning (sparas i designen).

Resultat med standardkonfigurationen (släpp från 10 m, olika kastfarter):

| Modell | Trim α | Cmα (1/rad) | Glidtal vid trim | Sjunkhastighet stationärt |
|---|---|---|---|---|
| Platt ark | ingen | positiv (instabil) | – | tumlar, ingen stabil glidning |
| Dart | 6,5° | −0,32 | 3,4 | ≈ 1,5 m/s |
| Glidplan | 9,1° | −0,29 | 4,6 | ≈ 0,9 m/s |
| Dart + gem på nosen | 4,6° | −0,48 | – | snabbare och brantare |

## 16. Etapp 3: kast och felsökning

- **Kastläge** (knappen ✈ Kasta eller T i editorn): reglage för styrka 1–15 m/s, vinkel uppåt,
  riktning, nosvinkel vid släpp och släpphöjd (sparas per webbläsare). Planet släpps utan rotation
  med vingarna vågräta. Mellanslag kastar.
- **Fri flykt** stegas med fast 240 Hz via `FixedStepper`. Renderingen interpolerar mellan stegen.
  Paus (P), stega ett tidssteg (.), tidsskala 1×/½×/¼×/⅒× (1–4).
- **Värld:** stor hall, mätlinjer varje meter längs +X (etikett var 5 m), röd släpplinje och en
  skuggande sol som följer planet.
- **Kamera:** följ (behåller användarens orbit-vinkel och flyttas med planet), från sidan, fri (C).
- **Felsökning** (D): tyngdpunkt, hastighetsvektor, lyft (grönt) och motstånd (rött) per ytelement,
  samt graf över höjd och fart mot tid.
- **Resultat:** distans och flygtid till första nedslaget (officiellt), nedslagsfart, maxhöjd och
  vilopunkt med glid- och studssträcka. Markörer och spår visas i två färger (luft blå, mark orange).
  De 30 senaste kasten sparas i localStorage, med bästa resultat per modell.
- Efter etapp 3 (önskemål): dal och berg är sammanslagna i vinkeldraget. Uppåt ger dalveck (mot
  betraktaren), nedåt ger bergsveck, och man kan passera noll i samma drag. Stoppvinkeln för omöjliga veck
  räknas separat för varje håll. Dal/Berg-knapparna och tangenterna D och B är borttagna.
- Fler önskemål efter etapp 3:
  - "Översta flik" är förvalt lager.
  - Vecklinjen är tjock och streckad (Line2) med mörk kontur och ändpunkter.
  - Vinkeldraget kräver nu ungefär 110 px för 180° (0,6 px/°).
  - Knappen **Spegla senaste** (N), under Spegla i vik- och saxpanelen, gör om det senaste vecket
    eller snittet spegelvänt på andra halvan. Det blir en egen operation, så den går att ångra separat.
  - **Automatisk riktning** (`core/paper/orientation.ts`): symmetriplanet ger vingaxeln, nosen är
    arkets överkant projicerad in i planet och upp väljs så att mittvecket (arkets mittlinje) hamnar
    nedåt. Riktningen räknas om när modellen ändras och vid kast, utom när spelaren har valt
    riktning för hand i fysikpanelen.
  - **360-boll** (`ui/editor/ViewBall.ts`) nere till vänster i vikläget. Dra i den så snurrar
    modellen fritt (arcball i skärmrymd) och följer musen. Bollen visar en miniatyr av modellen och
    nosen (orange prick). Dubbelklick ställer planet rättvänt igen.
    Bollen rastar in i 90°-lägen inom 3° (24 lägen relativt planets rättvända läge, så att
    pappret ligger exakt platt eller rakt). Ringarna lyser grönt när modellen är inrastad.
  - **Rutat papper** (kryssruta längst ned i Vik-rutan): rutor på bredd/10, alltså 21 mm på A4,
    linjerade efter mittlinjen och överkanten. Heldragna 1 px-linjer, och varje ruta är delad i fyra
    med streckade linjer. Rutnätet ritas i arkrum genom varje facett och följer därför alla veck och
    lager, även i kastläget. Valet sparas per webbläsare.

## 17. Etapp 4: tävlingslägen

- Lägen (`core/game/competition.ts`, ren och deterministisk): **Fritt kast**, **Längst flygtid**,
  **Längst distans** och **Precision**. Precision har ett mål 3–10 m bort, inom ±15° från rakt fram,
  och mäter avståndet från första nedslaget till målet i cm. Varje runda har tre kast och det bästa
  räknas.
- Topplista per läge, topp 10 i localStorage. Varje post sparar allt som behövs för att spela upp
  kastet exakt: designen inklusive skador, kastparametrar, styrinput per fysiksteg och målet.
  **▶ Spela upp** kör kastet igen och visar "✓ Verifierat" när resultatet blir bitidentiskt.
- Gränssnitt: lägesväljare i kastläget, mål på golvet, rundinformation i siktpanelen samt
  resultat och topplista i resultatkortet.

## 18. Etapp 5: klaffar

- Verktyget **Klaff** (K) i vikläget: klicka på en vikt flik så blir vecket som vek den en
  klaff (`FoldOp.control`). Speglade veck ger ett par. Klaffar visas orange.
- Fysiken lägger varken till krafter eller specialregler. Klaffytan vrids runt sitt eget veck,
  och samma aeromodell (`SurfacePose`) ger effekten. Gångjärnet räknas ut ur geometrin, och var
  klaffen sitter avgör vad den gör: bakom tyngdpunkten blir den höjdroder, framför blir den canard
  och vid sidan skevroder. Ett speglat par blir alltså elevoner.
- Styrning med ↑↓ (höjd) och ←→ (skevning), ±25° med fullt utslag på 0,2 s (spelarens val).
  Styrinputen registreras per fysiksteg (`ControlTrack`), så kast med styrning kan spelas upp exakt.
- Kastläget ritar planet som paneler som rör sig var för sig (`Simulator.panelPoses`).

## 19. Etapp 6: flexibla veck och hållfasthet

- Valet **Flexibelt papper** finns i vikläget och sparas i designen (`DesignFile.flexible`).
  Stelt är standard, och valideringstesterna körs med stel modell.
- **Paneler** (`physics/panels.ts`): papper som inte kan böjas i modellen, det vill säga facetter
  förenade av platta sömmar eller 180°-veck, lager som ligger på varandra och lager som ett gem
  klämmer. Övriga veck blir **gångjärn** med pappersveckets styvhet, 0,08 N·m/rad per meter veck
  och lager, ungefär motsvarande D ≈ Et³/12 för 80 g/m². Om gångjärnen bildar en loop slås
  panelerna ihop.
- **Flerkroppsmodell** (`physics/multibody.ts`) i reducerade koordinater: grundkroppen plus en
  vinkel per gångjärn. Massmatrisen är Σ Jᵀ M J, med pappersmassa och tilläggsmassa per panel,
  och den innehåller bias- och gyrotermer. Luftkrafterna kommer från samma elementmodell, med varje
  panels ytor positionerade. Papperet är så lätt att luftdämpningen (2–3 ms) är snabbare än
  tidssteget. Därför används **linjärt implicit Euler**: luftkraftens jakobian mot gångjärnens
  vinkelhastigheter (finita differenser) och veckens fjädrar och dämpning behandlas implicit. Med
  mycket styva veck blir resultatet detsamma som för den stela modellen (test).
- Klaffar i den flexibla modellen är gångjärn som drivs av en servofjäder mot kommandot.
- **Skador** (spelarens val: realistiskt, och de ligger kvar tills planet viks om):
  - Skrynkling: ett nedslag över 4,5 m/s, mätt som normalfarten i kontaktpunkten, viker papperet
    vid kontaktpunkten. Det gäller även stelt papper.
  - Plastisk böjning: ett veck som böjs mer än 15° från vilovinkeln får en bestående vinkeländring.
  - Rivning: ett snitts ände på ett hårt belastat veck, eller nära ett hårt nedslag, rivs vidare.
  - Skadorna blir vanliga operationer (`physics/damage.ts`), alltså veck och snitt efter
    spelarens egna. Ett skadat plan är fortfarande bara papper, och samma fysik flyger det.
  - Kastläget flyger det skadade planet från nästa kast. Vikläget visar "⚠ Skadat (n) · Reparera",
    och skadorna försvinner vid nästa vikning, ångra eller laddning.

## 20. Snittflikar, lagerval och kastläget (efter etapp 6)
- **"Alla lager" följer den dragna linjen.** Vecket tar alla lager som har en kant på den dragna
  delen av linjen och det som sitter fast i dem. Snitt begränsar alltså vecket: fliken mellan två
  snitt viks ensam, också i kölen där båda lagren klipps. Papper på den rörliga sidan som inte når
  linjen ligger kvar, utom lager som ligger inklämda mellan de rörliga lagren. En linje som når en
  hårsmån förbi snittändarna (mindre än 1 mm) räknas inte. Tidigare tog "Alla lager" allt på den
  rörliga sidan av den oändliga linjen.
- **Lagerval efter linjen.** Lagret kan bytas efter att linjen är dragen, i menyn eller med L.
  När man pekar på en sida markeras det papper som skulle vikas. Om vecket inte går att göra
  förklarar hjälptexten varför.
- **Linjens plan.** Linjen läggs i det pappersplan som störst del av den dras över (nio
  provpunkter längs linjen), inte i planet där dragningen började. Annars kunde linjen sväva i
  en uppvikt klaffs plan.
- **Snittflikar i flexibelt papper** (`physics/slits.ts`). Två snittändar som vetter mot
  varandra över plant, oveckat papper bildar ett gångjärn, om linjen mellan dem frigör en bit
  papper. Lager som klippts tillsammans räknas ihop. Fasetterna delas längs linjen i `prepare`.
  - Styvheten är plan pappersböjning, ett fast inspänt plattelement: k = 3·D·w/ℓ, med
    D = 2,5·10⁻⁴ N·m. ℓ är flikens längd, men högst remsans bredd och minst veckzonen (3 mm).
  - Gångjärnet ger inte plastiskt efter. Snittändarna ligger på gångjärnet, så stora moment
    river dem vidare, liksom hårda nedslag nära en snittände.
  - Ett ensamt snitt i plant papper ändrar inte styvheten i modellen. Panelerna är stela, och
    plattböjning av hela ytor modelleras inte. Snitt som korsar ett veck gör gångjärnet kortare
    och därmed vekare.
  - Stelt papper bortser från snittflikar.
- **Kastläget:**
  - Kameran följer med till nytt kast och behåller samma vinkel.
  - "🔧 Reparera" finns i listen och på resultatkortet.
  - "🐢 Slow time" växlar mellan ½× och 1×.
  - ↓ ger nosen upp och ↑ nosen ned, som en styrspak.

## 21. Återfjädring, plattböjning, tejp och flexibelt som standard
- **Återfjädring** (`paper/springback.ts`, `paper.creaseSpringback = 0,05`). Ett släppt veck öppnar
  sig med en andel av sin vikvinkel. Värdet motsvarar ett ordentligt tryckt veck: ett 180°-veck
  glipar ungefär 9°. Det är en uppskattning som ska kalibreras mot riktiga plan.
  - Ordning: per vikning, sista vikningen först. Det papper vikningen flyttade vrids tillbaka runt
    sitt veck som en enhet.
  - Låsta veck: ett veck som en senare vikning har böjt (inte längre en rak linje) öppnar sig inte.
    Det gäller till exempel nosflikarna på darten efter vingvikningen.
  - Lager som hålls fast: gem och tejp över vecket, eller papper ovanpå, hindrar vecket från att
    öppna sig. Delvis blockerade veck provar en mindre öppning.
  - Till sist passas modellen in med minsta kvadrat mot den vikta modellen, så att kroppsaxlarna
    stämmer. Ett glipande mittveck öppnar sig då symmetriskt (±4,5°).
  - Gäller alla kast, både stelt och flexibelt papper, via `prepare`. Editorn visar den ideala
    vikningen, och valideringstesterna använder den ideala geometrin.
  - Skador räknas om till den vikta geometrin (`damageOnFolded`) innan de blir operationer.
- **Aero för glipande lager.** Nästan plana lager (inom 25°) som överlappar och ligger nära
  varandra (inom 12 mm) räknas som en platta. Om de rör sig var för sig, som olika paneler, får den
  större ytan överlappet. Överlappande yta räknas alltså bara en gång.
- **Plattböjning i flexibelt papper.** Varje vecks gångjärn har det plana papperet bortom vecket
  (sett från kroppen) i serie med vecket.
  - Formel: k_p = r_c² / ∫₀^{r_c} (r_c − r)² / (D·w(r)) dr, där w(r) är papperets bredd över alla
    lager på avståndet r från vecket och r_c är lastens tyngdpunkt.
  - Bara veckets andel av rörelsen kan ge plastisk deformation.
  - Effekt: vingarna böjer sig 2–4° vid lugna kast och 20–40° vid hårda.
- **Paneler och kontakt.**
  - Panelträdets rot är kroppen, alltså mitten av panelgrafen.
  - Varje veck har ett stopp där sidorna ligger plant mot varandra (en kontaktfjäder,
    `contactStiffnessFactor`), så papper inte kan gå igenom papper.
  - Slingor genom glipande lager löses genom att lagren trycks ihop.
  - Gångjärnslinjer inom 3 mm från en rät linje räknas som ett gångjärn.
- **Tejp** (verktyg J). En 19 × 30 mm remsa vikt över en kant väger 0,03 g. Den håller ihop lagren
  som ett gem (`ClipOp.kind = 'tape'`).
- **Flexibelt papper är standard** i spelet. Designer som inte anger något flyger flexibelt.
- **Kastläget:**
  - Rutorna placeras under listen även när den bryts på flera rader.
  - "Flygdata" (förut Felsökning) är på från start.
  - Knappen Slow time är borttagen. Tidsskalan finns kvar i menyn och på tangenterna 1–4.

## 22. Synlig skrynkling och återfjädring i vikläget
- **Intryckt kant.** En hård krock mot en kant blir ett dragspel av fem omväxlande veck genom alla
  lager (`crumpleFolds = 5`) över en zon som är 2,5 gånger intryckningsdjupet.
  - Första och sista vecket viks α, de mellersta 2α, där cos α = 1 − k / ((k − 1)·2,5) ⇒ 60°.
    Spetsen fortsätter då rakt fram men blir djupet kortare.
  - Om papperet inte tillåter ett rakt dragspel, till exempel glidplanets trubbiga nos där vecket
    skulle gå in i vingarna, provas diagonala veck vid hörnet och därefter grundare intryckningar.
  - Skador fästs vid papperet (arkkoordinater), så att ordningen skrynkla → böj → riv håller även när
    de andra skadorna flyttar papper.
  - Skrynklat papper ritas något gråare och flammigt.
- **Vikläget visar planet som det vilar.** Editorn visar den vikta modellen med återfjädring och
  med eventuella skador. Verktygen räknar fortfarande på den exakt vikta modellen.
  - `hitPaper` översätter träffen via arkkoordinater till den vikta modellen och returnerar en
    *frame* (vikt → visad) för den papperbit man arbetar på.
  - Linjer, markeringar, spöken och skärmpositioner ritas i den framen, och strålar räknas tillbaka.
    Vik-linjen hamnar därför på det papper som syns.

## 23. Kortare intryckning, skada direkt i flykten, gem som håller lokalt
- **Bara spetsen viks ihop.** Intryckningsdjupet är 2 mm vid 4,5 m/s plus 1,2 mm per m/s över det,
  högst 8 mm. Dragspelet har tre veck över 1,6 gånger djupet (α ≈ 86°), så den yttersta biten
  viks tätt ihop.
- **Skadan syns direkt.** I kastläget ritas en skrynkling om i samma ögonblick som den uppstår.
  Den skadade modellens papper matchas mot de simulerade panelerna via arkkoordinater, så den rör
  sig med planet. Flygningen fortsätter med samma fysik. När planet ligger still visas hela
  skadan, även böjar och revor. Kameran kan zooma in till 8 cm.
- **Gem och tejp håller där de sitter, och glipan växer som ett smalt V.** Vid hållaren är vecket
  stängt. Glipan växer sedan linjärt med avståndet och når full återfjädring först vid papperets
  ände: θ(s) = θ · avstånd / (avstånd från hållaren till änden). Sidorna löper som raka linjer
  från gemet, så det ser ut som ett smalt V ovanifrån.
  - Fasetter längs vägen delas i remsor och trianglar. Remsbredden väljs så att vridhastighet ×
    remsbredd × avstånd från vecket ≈ 0,2 mm, och varje bit placeras stelt på den vridna ytan.
  - Vecklinjer ritas inte där papperet bara vrids lite (under 2°).
- **Båda sidorna öppnar sig.** Ett veck öppnar sig genom att båda sidorna vrids från varandra.
  Den lättare sidan vrids mer, i proportion till den andra sidans pappersyta. Ett mittveck öppnar
  sig därför symmetriskt kring mitten.
- I den flexibla modellen räknas papper som fortsätter nästan plant (under 3°) som samma panel.
    Med gem blir kölen därför en panel med vriden viloform. Den kan inte vridas isär ytterligare
    under flygningen; det skulle kräva vridleder.

## 24. Klaffar som kombinerade höjd- och skevroder
- **Varje klaff blandar höjd och skevning** (elevon): utslag = clip(höjd · pitchSign + skev ·
  rollSign) · 25°.
  - pitchSign är +1 bakom eller i höjd med tyngdpunkten och −1 tydligt framför den. En klaff i
    höjd med tyngdpunkten ignorerades förut för höjdled.
  - rollSign beror på vilken sida klaffen sitter på. Bara en klaff mitt på centerlinjen saknar
    skevning.
- **Piltangenterna styr alltid under flygningen**, även om en meny eller ett reglage har fokus.
  Fokus tas då bort från menyn. Ett kort tryck som släpps innan nästa fysiksteg räknas ändå för ett steg.
- **Verifierat att klaffens storlek påverkar effekten** (glidplan med höjdroder 10/20/30/40 mm,
  stelt och flexibelt papper):
  - Klaffytan är 17, 35, 52 och 70 cm².
  - Rollhastigheten efter 0,15 s är 1,6, 3,5, 6,0 och 8,5 rad/s, ungefär proportionell mot ytan.
  - Stigningen efter 0,4 s med fullt höjdroder är +17,6°, +23,8°, +26,3° och +28,0°. Den ökar men
    planar ut, eftersom planets egen dämpning i stigning tar över.
  - Darten ger samma bild: +16° till +31°.
  - Testerna finns i `tests/core/physics/controls.test.ts`.

## 25. Platser att kasta på
- **Platser** (`core/game/venues.ts`) bestämmer bara världen runt planet: tyngdkraft, luft och fasta
  ytor. Planet och fysiken är desamma överallt. Sporthallen är alltid vald när spelet startar.
  - **Sporthall** (hette tidigare gymnastiksal och var 30 × 16 × 7 m): 46 × 26 m med 9 m i
    takhöjd, som en fullstor kommunal hall med en handbollsplan på 40 × 20 m. g = 9,81, luft.
    Planet kastas 3 m från ena kortväggen, alltså på handbollsplanens ena mållinje.
    - Golv, väggar och tak är kontaktytor, så planet stannar mot väggen. Distansen räknas till
      första nedslaget, på golvet eller i en vägg.
    - Golvet är ett blått sportgolv med linjer för handboll (vitt), basket (rött), volleyboll
      (gult) och badminton (grönt), och distanslinjer från kastpunkten varje meter.
    - Väggarna har träpaneler upp till 2,6 m och ljus vägg ovanför. Salen har limträbalkar och
      lampor i taket, höga fönster, korgar, handbollsmål mot kortväggarna, ribbstolar, bänkar,
      dörrar och mattor.
  - **Månen:** g = 1,62 och ingen atmosfär (ρ = 0). Inget lyft, inget motstånd och ingen
    tillskottsmassa, så planet flyger en ren kastparabel. Testerna jämför med den analytiska
    parabeln.
    - Planet glider inte efter landningen. I den svaga tyngdkraften skulle vanlig friktion
      (μ·g) låta det glida tiotals meter. Lös regolit bromsar i stället genom plogning: varje
      punkt som rör marken trycker damm framför sig, med motståndet ρ·A·v_t²
      (`GroundConfig.ploughKgPerM` = 0,05 kg/m, ungefär 1500 kg/m³ × 3 mm × 1 cm). Det beror
      inte på g. Friktionen är μ = 1 och studsen 0.
    - Där planet landar kastas ett litet grått dammoln upp (`render/DustEffect.ts`). Kornen
      flyger i rena kastparabler med månens g och ligger kvar där de landar.
    - Hårda, branta kast kan fortfarande välta över nosen och slå kullerbyttor efter
      landningen, eftersom det inte finns någon luft som bromsar. Användaren valde att behålla
      det (realistiskt för en styv kropp utan luft) i stället för att dämpa det.
    - Det finns ett grått regolit med stenar, rundade kullar vid horisonten, svart himmel,
      hårt lågt solljus och distanslinjer varje 5 m.
  - **Solen:** g = 274, ingen yta att landa på. Papperet brinner upp direkt.
    - Ytan ligger i golvhöjd, så betraktaren står nära den. Den bortre kanten ligger då
      nästan i ögonhöjd, och horisonten blir ett rakt streck.
    - Planet blir svarta flagor i pixelstorlek som först faller isär och sedan stiger uppåt
      och försvinner inom 0,5 s. Därefter visas "Flygplanet brann upp".
    - I vikläget ligger en liten askhög på bordet tills man trycker på Reparera, som ger ett
      nytt ark vikt likadant. Att ladda en modell eller börja om tar också bort askan.
- **Kontaktytor:** `GroundConfig.planes` är en lista av plan (fritt där n·x ≥ d). Båda
  simulatorerna löser kontakt mot alla plan med samma impulsmodell.
- **Topplistor per plats och spelläge.** Sporthallens listor behåller gymnastiksalens gamla nycklar, och
  uppspelningar sker på den plats där kastet gjordes.
- **WebGL-fel:** om grafiken inte kan startas, eller går förlorad, visar sidan ett meddelande i
  stället för en tom yta.
  - 2D-canvasens `filter: blur` per penseldrag i stora texturer fick grafikprocessen att startas
    om, och då spärrade Chrome WebGL för hela `localhost`. Använd inte canvasfilter i
    procedurtexturer.

## 26. Böjformer: plant papper som böjer sig i fartvinden
- **Varför:** ett ovikt ark, eller en bred vinge, var en stel platta mellan vecken. Riktigt papper
  böjer sig mellan vecken. Ett 10 cm utskjutande A4 sjunker flera centimeter av sin egen vikt
  (D = 2,5·10⁻⁴ N·m), och vid hårda kast böjer sig vingarna mer.
- **Modell** (`physics/bendModes.ts`, flexibelt papper): stora plana paneler (minst 25 cm², plana
  inom 12°, inte klaffar) får några släta böjformer φₖ. Nedböjningen är w = Σ qₖ φₖ längs
  panelens normal, och amplituderna qₖ är extra frihetsgrader i flerkroppsmodellen.
  - Paneler som hänger i ett veck är fast inspända där: d̃² (böjning ut från vecket), d̃²s̃
    (vridning) och d̃²(s̃²−⅓) (välvning). Den fria rotpanelen (kroppen eller ett helt ovikt ark)
    får x̃², ỹ² och x̃ỹ, ortogonala mot stel rörelse.
  - Styvheten är plattenergin ∫ D [(∇²w)² − (1−ν)(…)] dA med ν = 0,3. Former som svänger
    snabbare än 120 Hz (smala remsor) räknas som stela och tas bort.
  - Trögheten är panelens papper och medföljande luft från samma masspunkter, så att
    massmatrisen alltid är positivt definit.
  - Luftkrafterna räknas på den böjda ytan: varje element flyttas med w, lutas med ∇w och får
    hastigheten ẇ. Den generaliserade kraften är Qₖ = Σ (F·n) φₖ.
  - Styvhet och dämpning integreras implicit, som vecken. Luftens dämpning tas fram med
    finita differenser för de element varje frihetsgrad påverkar, och räknas om var tredje steg.
  - En platta som böjs i ett veck har inte längre plattböjningen i serie i gångjärnet. Den
    finns i formerna i stället.
- **Mjuk gräns:** formerna är linjära, så papperet rör sig rakt längs normalen och skulle
  töjas vid stora utslag. Riktigt papper kan inte töjas, utan böjer sig inåt och vrider sig ur
  vinden. Bortom 35° lutning (`bendingLimitSlopeDeg`) styvnar formen snabbt: 30 × sin egen
  styvhet vid gränsen, och den extra styvheten växer med amplituden upphöjd till sex.
  - Innan gränsen fanns böjde sig glidplanets vingar 168 mm vid 12 m/s. Med gränsen blir det
    ungefär 35–45 mm.
- **Typiska värden:** ett kastat A4 böjer sig 25–35 mm. Glidplanets vingar böjer sig ungefär
  13 mm i glid och 40 mm direkt efter ett hårt kast. Simuleringen går 5–14 gånger snabbare än
  realtid i Node.
- **Golvkontakt:** en böjd panel ligger inte längre mellan sina hörn. Böjliga paneler får därför
  kontaktpunkter längs kanterna och över ytan med ungefär 4 cm mellanrum. Annars sjönk mitten av
  ett ark upp till 65 mm genom golvet.
- **Ritning vid golvet:** golven skjuts lite bakåt i djupled (`FLOOR_DEPTH`), och distanslinjerna
  ligger i golvets plan. Papperet ritas 1 mm ovanför sitt simulerade läge, så det aldrig
  försvinner in i golvet eller hamnar under linjerna.
- **Visning:** i kastläget ritas papperet med trianglar på högst 12 mm (`PaperView` med `maxEdge`).
  Varje bildruta flyttas hörnen och kantlinjerna längs böjningen
  (`Simulator.panelBending` → `FlightView.setPanelBending`), och ljusnormalerna lutas med ∇w.

## 27. Snabbare beräkning, gem som lutar bakåt och ny tejp
- **Snabbare beräkning.** Det flexibla dart-planet gick från ungefär 2,0 till 0,8 ms per steg i
  luften (Node). Ett steg är 1/240 s, så det är ungefär fem gånger snabbare än realtid.
  - **Golvkontakt:** varje panel har en omslutande sfär. Bara paneler som når en yta får sina
    punkter kontrollerade. Impulsmatriserna M⁻¹Jᵀ och JM⁻¹Jᵀ räknas en gång per punkt, så varje
    iteration blir linjär i antalet frihetsgrader.
  - **Böjning:** formerna summeras till ett polynom per panel (`bendField`) som räknas utan att
    skapa nya listor. Det används både i kontakten och i ritningen. Ritningen räknar varje unik
    punkt en gång och har fasta gränssfärer. För en dart tar det 1,3 ms per bildruta i stället
    för 4,8 ms.
  - **Luftens dämpning:** matrisen räknas om var sjätte steg (1/40 s). Flygningarna ändras mindre
    än 0,5 % jämfört med var tredje steg.
  - **Böjformerna:** integrationspunkterna ligger med 12 mm mellanrum i stället för 6 mm, så de
    byggs ungefär dubbelt så snabbt.
- **Gem som lutar bakåt** (`resolveClip`): ett gem kan inte gå genom papper som lämnar dess plan,
  till exempel vingen ovanför kölen. Gemmet vrids då i sitt plan, runt punkten på kanten, så lite
  som behövs för att inte korsa vecket där vingen börjar (högst 80°).
  - Editorn sparar riktningen bakåt från nosen i operationen (`lean`). Om gemmet aldrig får plats
    bakåt lutar det framåt. På en dart (kölen är 25 mm hög och gemmet 28 mm) lutar det ungefär 36°.
- **Tejp, alltid 2 cm lång** (19 mm bred, 0,02 g):
  - **Över skarven mellan vinghalvorna** (`bridge`): tejpen läggs ovanpå, 10 mm på varje halva,
    även när halvorna står i ett grunt V (högst 35° ur planet). Den håller ihop papper på båda
    sidor om ryggen, så mittvecket hålls stängt där (återfjädringen) och vingarna sitter ihop.
  - **Över en kant** (ryggen, nosen, en vinge): förhandsvisningen hänger hälften i luften. När man
    trycker fast tejpen viks den fria halvan över till andra sidan.
  - **Det som sticker ut** utanför pappret, till exempel framför nosen, klipps bort automatiskt
    (`tapeShape`: tejpen skärs med pappret under den).

## 28. Tunga modeller, rivning, kullerbyttor på Månen och Spara som
- **Varför vissa plan var tunga** (en egen modell med två klaffar mellan snitt):
  - Kölens två halvor böjde sig var för sig. Den ena var rotpanel med fria former, den andra var
    fastspänd vid ryggen. Halvorna ligger nästan an mot varandra, men böjde sig igenom varandra.
    Det gjorde planet osymmetriskt, kortade flygningen (3 m i stället för 7,4 m) och fick
    simuleringen att explodera vid 15 m/s.
  - Böjformer ges nu bara till ett ensamt ark som rot, och till paneler som inte ligger an mot
    en annan panel (gångjärn inom 25° från stängt). En rot som bär andra paneler böjer sig inte
    själv.
  - Styrklaffarnas servofjäder (20 × vecket, 0,04 g klaff) krävde 15 delsteg. Den löses implicit
    och styr inte längre antalet delsteg. Det blev 5 delsteg, och flygningarna är desamma.
  - Om ett steg ändå ger ogiltiga tal behålls det senaste giltiga läget och flygningen avslutas.
  - **Spelloopen** (`app/GameLoop.ts`) räknar fysik i högst 12 ms per bildruta. Det som inte
    hinns med släpps: spelet går långsammare i stället för att bildfrekvensen rasar.
    Tidigare kunde 32 tunga steg per bildruta ge en ond cirkel.
- **Rivning:** rivkriteriet räknade med servofjädern, så en klaff som trycktes mot golvet rev
  vidare från snittändarna. Nu används papprets eget veckmoment, och gränsen är 0,5 N·m/m per lager.
  - Böjning kan inte nå gränsen: ett helt vikt veck bär högst ungefär 0,25 N·m/m, och luften
    vid 15 m/s ger ungefär 0,007 N·m/m på en klaff. Snittändar rivs bara av hårda nedslag.
- **Sömmar vid böjliga vingar:** en stel klaff på en böjande vinge kan bara följa vingens lutning
  i en punkt. Längs resten av vecket blev det en glipa på upp till 1 cm vid 15 m/s, som såg ut
  som en rivning.
  - I ritningen flyttas klaffens papper närmast vecket till vingens böjda kant (`seamInto`).
    Flytten avtar kvadratiskt över klaffen, och glipan blir under 0,01 mm.
- **Månen utan kullerbyttor:** dammet tar upp papprets rotation medan det rör marken
  (`GroundConfig.spinDampingPerS` = 150 1/s). Användaren valde det framför realistiska
  kullerbyttor.
  - Planet glider högst ungefär 16 cm, i snitt 6 cm, och hoppar inte. Det gäller i alla prövade
    kast: tre plan, 4–14 m/s, 0–45°, stelt och flexibelt.
- **Spara som:** i Chrome och Edge öppnas webbläsarens eget Spara som-fönster med namn och mapp.
  Andra webbläsare frågar efter ett namn och laddar ner filen. Modellen får filens namn.

## 29. Ojämna kast, vingformer, fena i kölen och rättningar
- **Gem behåller lutningen:** lutningen räknades om på den återfjädrade modellen. Där har ryggen
  öppnats lite, så vingens veck låg inte längre i kölens plan, och gemmet blev rakt. Nu räknas
  lutningen när gemmet sätts på det vikta planet och sparas på gemmet (`Clip.tiltDeg`).
- **Det långsamma läget:** med två gem delade återfjädringen kölens inre lager i ett 30-tal remsor.
  Varje remsa blev en egen panel (0,009 g) med gångjärn, vilket gav 41 frihetsgrader, 24 delsteg och
  23 ms per steg. Remsor med samma vikhistorik har aldrig vikts mot varandra och hör nu alltid till
  samma panel. Det blev 11 frihetsgrader och 0,9 ms per steg.
- **Förinställningarna utan snitt:** Dart och Glidplan har inga klippta klaffar längre. De trimmas
  i stället av att vingens bakre yttre hörn viks upp (4 cm och 3 cm, 30°, `reflexOps`). Det ger
  ungefär samma trim som de gamla klaffarna. Klaffar finns kvar för tester (`ELEVATOR_TRIM`).
- **Lyftcentrum:** den blå pilen i fysikvyn var kroppens uppåtaxel. Nu visar den lyftcentrum
  (neutralpunkten, `centerOfLift`): x_np = x_cm + (Cmα/CLα)·c̄ vid en fast anfallsvinkel på 5°.
  Bakom tyngdpunkten betyder stabilt i tippled.
  - Vinkeln är fast, så punkten beror bara på formen. Ett gem flyttar tyngdpunkten men inte
    pilen.
  - Vid trimvinkeln skulle punkten också flytta sig, eftersom trimvinkeln ändras med gemmet och
    dartens neutralpunkt flyttar bakåt när vinkeln ökar: −30 mm vid 3° och −58 mm vid 10°.
- **Ojämna kast** (`LaunchParams.scatterSeed`, `PhysicsConfig.scatter`): spelet ger varje kast ett
  nytt slumpfrö, och uppspelningar behåller det. Ingen kastar exakt rakt eller viker exakt
  symmetriskt.
  - Avvikelserna är normalfördelade och kapas vid 2,5σ. Standardavvikelserna är: lutning åt sidan
    4°, nosen snett 3° åt sidan och 2° i höjd, rotation vid släppet 40/20/25 °/s (roll, gir,
    tipp), och i flexibelt papper varje vecks vila 1,5°, olika på vänster och höger sida.
  - Platta vingar blir då tydligt ostadiga: kursen ändras 55–76° från 3 m höjd. Hårda kast stiger
    och överstegrar ofta men inte alltid: darten vid 13 m/s i hälften av kasten, aldrig vid 5 m/s.
- **Vingformer** (vindtunnel vid α = 6°, valideringstester): allt som viks ur planet lyfter mindre
  än platta vingar men är stabilare. Clβ < 0 betyder att planet rätar upp sig i roll, och Cnβ > 0
  att det vrider nosen mot vinden.

  | Form | Lyft | Clβ | Cnβ |
  |---|---|---|---|
  | Platta vingar | 52 mN | −0,028 | −0,037 (instabil i gir) |
  | Dihedral 20° | −18 % | −0,088 | −0,018 |
  | Anhedral 20° | −18 % | +0,069 (kan tippa runt) | −0,021 |
  | Spetsar upp/ner 2 cm | −38 % | −0,033/−0,012 | +0,018/+0,013 (fenor) |
  | Djupt W | −18 % | −0,039 | −0,027 |
  | Grunt W | −2 % | nästan som platta vingar | nästan som platta vingar |

  - Spetsarna tar mer lyft än sin yta, eftersom lyftet för korta, breda vingar växer ungefär med
    spännvidden i kvadrat.
- **Fena av en flik i kölen** (`canPushThrough`, `FoldOp.through`): fliken mellan två snitt i kölen
  viks åt sidan och stoppas mot vingens undersida. Drar man 20° förbi stoppet hoppar den över
  till 180° och pekar rakt upp mellan vingarna.
  - Det tillåts bara om fliken inte går igenom något papper där den hamnar, och om den passerar
    en skarv där två fasta bitar i olika plan har en kant längs samma linje, som vingrötterna.
    Riktigt papper glider isär där ett ögonblick och sluter sig igen.
  - **Kollisionskontrollen:** papper som svänger förbi annat papper som sitter i samma
    gångjärnslinje räknas nu som papper genom papper. Tidigare passerade en flik som var klippt
    ända upp till vingroten rakt genom vingen.
  - Fenan gav något rakare flygning i provkasten. Den ger också mer luftmotstånd, se friktionen
    nedan.
- **Friktion från framkanten:** friktionsmotståndet räknas per element från hur långt luften har
  strömmat längs papperet sedan framkanten. Det är ett laminärt gränsskikt (Blasius) med lokalt
  Cf = 0,664/√Re_x per sida. Medelvärdet över elementet, (√x₂ − √x₁)/(x₂ − x₁), räknas fram när
  flödesriktningen ändras, och ett tillägg för kanterna (`CDedge` = 0,005) kommer till.
  - En flik nedåt är en fortsättning på kölen, där gränsskiktet redan har växt. Den ger exakt
    samma motstånd som en hel köl.
  - Uppåt får fenan en egen framkant och ger ungefär 4 % mer nollyftmotstånd för hela darten.
  - Totalt blev friktionen ungefär 12 % lägre än med det gamla fasta värdet (CD0 = 0,02), eftersom
    papperet längre bak på långa ytor har tunnare friktion. Dartens glidtal gick från 3,2 till 3,6.

## 30. Platt dart, pilvinkel, skadade plan, springor, kölflik och tävlingsläge
- **Darten utan extra veck:** standardmodellen har platta vingar och inga uppvikta hörn. Otrimmad
  trimmar den till ungefär 0° anfallsvinkel och flyger snabbt nosen nedåt, som en riktig otrimmad
  dart. Valideringen jämför med en trimmad dart (`trimmedDart`: 10° dihedral och uppvikta hörn).
- **Pilvinkel i lyftet:** lyftlutningen räknas per element med strimlans pilvinkel mot flödet, i
  DATCOM-formen av Helmbold: a = a₀A/(2 + √(A²(1 + tan²Λ) + 4)). Λ är mitt-kordans pilvinkel,
  medelvärdet av fram- och bakkantens lutning.
  - I sidglidning blir den vindvända vingen mindre pilad och lyfter mer, så planet rätar upp sig i
    roll. Clβ för platta vingar gick från −0,028 till −0,040, och anhedral 5° blev ungefär neutral.
  - Effekten är svag för korta, breda vingar: anhedral 10° tippar fortfarande runt nästan alltid
    i provkasten, eftersom modellen ger den en rollderivata åt fel håll (Clβ +0,017).
- **Skadade plan som exploderade:** skrynkelvecken från en krasch blev små egna paneler (2–12 mg),
  och en av dem kunde bli rotpanel med flera gram papper hängande på sig. Det gav 24 delsteg och
  ett nästan singulärt system, och fart på tusentals m/s vid nästa nedslag.
  - Papper på var sin sida om ett `dmg-crumple`-veck är nu en stel bit, eftersom ihopskrynklat
    papper är en tät klump.
  - Dessutom avslutas en flygning om farten överstiger 60 m/s (`MAX_PLAUSIBLE_SPEED`), som extra
    skydd. Det behövdes inte i något av proven.
- **Springor i vikläget:** lagren ritas lite isär längs sina normaler. Där två bitar möts i ett
  veck hamnade deras gemensamma kant på två ställen, med en springa emellan. En smal remsa längs
  varje sådant veck fyller nu ut den.
- **Kölflik med ett snitt:** när ett snitt slutar på vecklinjen och delar pappret under den, viks
  bara biten man tog tag i (`FoldOp.pick`, alla lagerval). Det gäller även om linjen ritas förbi
  snittet. Två platta vinghalvor som möts kant mot kant räknas nu som en skarv att trycka igenom.
- **Kölfliken i vikverktyget:** en vecklinje längs vingroten ligger både i vingen och i kölen. Om
  linjen drogs i vingens plan, låg kölen i det planet varken på den ena eller andra sidan av linjen,
  och den gick inte att vika. Nu viks pappret i planet för den bit man tar tag i, när linjen ligger i
  det planet. Ligger linjen upp till 3 mm därifrån flyttas den in i planet (`FoldTool.grab`).
  - Med två snitt ställer sig fliken mellan dem upp, och kölen bakom det bakre snittet hänger kvar.
  - Med ett snitt ställer sig hela kölen bakom snittet upp.
  - Båda varianterna testas genom verktyget i `tests/ui/foldTool.test.ts`.
- **Tävlingsläge:** en rullgardin längst till höger i vikningens topprad, med valen Längd, Tid och
  Precision. En tävling har tre kast.
  - Mellan kasten kan man reparera, ändra kastet eller trycka "Ändra vikning". Då kommer man till
    vikningen, där "Fortsätt" uppe till höger går vidare till nästa kast. Tävlingen fortsätter med
    det omvikta planet.
  - Varje nedslag får en gul flagga som står kvar under resten av tävlingen.
  - Precision: en vit prick med 10 cm radie, 10 m rakt fram. Avståndet från första nedslaget räknas.
  - Efter tre kast visar rutan "Resultat" kasten från bäst till sämst (`rankThrows`). Det bästa kastet
    sparas på topplistan, med den vikning det kastades med.
- **Köleffekt** (användarens val, `markKeelJunctions`, `aero.keelDihedral`): vingar som sitter
  ovanpå en köl beter sig som om de hade lite dihedral. I sidglidning får den vindvända vingen en
  uppvind som motsvarar dihedralvinkeln, och planet rollar bort från vinden.
  - Dihedralvinkeln är perDepth × kölens djup / halva spännvidden: 25 × djup/halvspann, högst 10°.
    Darten får 8,3°. Hela lyftsystemet får effekten, även ett W:s yttre partier.
  - Modellen är halvempirisk: tabellernas effekt för en vinge högt på en kropp, anpassad efter
    användarens observationer.
  - Utfall i 32 kast med spridning (flexibel, trimmad dart):

    | | Platta vingar | Anhedral 5° | Anhedral 10° | Anhedral 20° |
    |---|---|---|---|---|
    | Utan köleffekt | 8 tippar | 10 | 29 | 32 |
    | Med köleffekt | 15 tippar | 15 | 6 | 32 |

  - Utan rättningen nedan tippade platta vingar oftare med köleffekten, eftersom darten var
    riktningsinstabil: en sidglidning bestod och blev till en roll.
- **Riktningsstabilitet: kölen under vingen** (`AeroSurface.underWing`): modellens dart var
  riktningsinstabil (Cnβ ≈ −0,037). Kölens sidokraft angrep ungefär 38 mm framför tyngdpunkten,
  vid framkanten, som på en låg platta.
  - Slank kroppsteori: en kropps sidokraft uppstår där tvärsnittets medföljande luftmassa i sidled
    växer längs kroppen. För en ensam köl sker det bara vid nosen. Vingen ovanpå är en ändplatta som
    blir bredare hela vägen bakåt, så kölens medföljande massa, och därmed sidokraften, växer längs
    hela kölen.
  - En köl under en vinge får därför sin lastfördelning jämnt utspridd i fäst flöde.
  - Cnβ: platt dart +0,002, trimmad dart +0,033, glidplan +0,007.
  - Tippar runt i 32 kast med spridning: platta vingar 0, anhedral 5° och 10° 0, anhedral 20° 12.
  - Platta vingar vinglar lite, ungefär en rollväxling per kast, och kursen ändras i snitt 4°.
  - Hårda kast stiger och överstegrar: darten aldrig vid 5–9 m/s, i 4 av 16 kast vid 11 m/s och
    i alla från 13 m/s.

## 31. Större spridning, baktunga plan och resultatruta
- **Större spridning mellan kasten** (användarens val): spridningen är fördubblad, och rollen vid
  släppet tre gånger första gissningen. Standardavvikelserna är nu:

  | Lutning åt sidan | Nos snett i sidled | Nos snett i höjd | Roll | Gir | Tipp | Veck |
  |---|---|---|---|---|---|---|
  | 8° | 6° | 4° | 120 °/s | 40 °/s | 50 °/s | 3° |

  - Platt dart (24 kast, 6 m/s från 1,8 m): största lutning i snitt 15° → 33°, och distansen
    varierar ±0,1 → ±0,4 m. Glidplanet ±0,4 → ±0,7 m.
  - Rollväxlingarna blev inte fler (ungefär en per kast för den platta darten, två för den
    trimmade). Svängning i roll kräver att planet vrider nosen tillbaka mot vinden, och den platta
    darten är nästan neutral i gir (Cnβ ≈ 0).
  - En svagare köleffekt gjorde inte platta vingar ostadigare inom ett kort kast, men fick anhedral
    10° att tippa runt igen. Köleffekten är därför oförändrad.
- **Tyngdpunkt och lyftcentrum** (trimmad dart, flexibelt papper, 12 kast):

  | Gem | Lyftcentrum bakom tyngdpunkten | Cmα | Trimvinkel | Tippar | Flygtid |
  |---|---|---|---|---|---|
  | Ett framtill | 44 mm | −0,40 | 4° | 0 | 0,76 s |
  | Inga | 23 mm | −0,23 | 7° | 0 | 0,97 s |
  | Ett baktill | 1 mm | −0,16 | 15° | 3 | 0,90 s |
  | Två baktill | −10 mm | −0,12 | 38° | 0 | 1,64 s |

  - Framtung blir stabilare, som det ska.
  - Baktung blir inte oflygbar: planet hittar ett stabilt läge vid hög anfallsvinkel och sjunker
    som en fallskärm. Lyftcentrum flyttar bakåt när strömningen släpper (tryckcentrum mot ½ korda),
    så det hamnar bakom tyngdpunkten igen.
  - Riktiga baktunga plan tippar ofta runt. Det beror till stor del på effekter som uppstår när
    papperet roterar: rotationen ger ett eget lyft, som när fallande ark fladdrar och tumlar
    (Pesavento & Wang 2004). Den kvasistatiska modellen saknar det. (Det är tillagt i §32. Där
    visade det sig också att gemets remsor gav fel lyftcentrum, och det var den största orsaken.)
- **Resultatruta:** när tävlingens tredje kast är klart visas en ruta mitt på skärmen. Den visar
  kasten från bäst till sämst, eventuell placering på topplistan och knapparna Ny tävling, Till
  vikningen och Stäng.

## 32. Rotationslyft och tvinnat papper vid gem
- **Rotationslyft (Kramer-effekten)**, `aero.rotationalLift` (1 = teorivärdet):
  - Varje element får redan sin anfallsvinkel med ω × r. Ett plan som roterar kring spännvidden
    får då anfallsvinkeln vid tryckcentrum (¼ korda).
  - Enligt tunnplatteteorin bestäms cirkulationen i stället av anfallsvinkeln vid ¾ korda.
    Skillnaden är ΔC_L = a·ω c/(2U), där c är den lokala kordan längs flödet.
  - Hälften av tillskottet får fördelningen vid ¼ korda och hälften den elliptiska fördelningen
    kring ½ korda. Tillskottet står vinkelrätt mot flödet, så det ger inget motstånd.
  - Tillskottet finns kvar när strömningen släppt, då vid ½ korda. Det är rotationscirkulationen
    hos ett fallande, tumlande kort (Pesavento & Wang 2004, samma storlek).
  - Rotationen ger extra lyft som verkar ungefär vid ⅜ korda. Ligger lyftcentrum bakom
    tyngdpunkten dämpar tillskottet tippningen. Ligger det framför förstärker det tippningen.
  - Storleken är kontrollerad mot teorin. För den trimmade darten är ΔCLq ≈ 1,5, alltså ungefär
    CLα (1,28) gånger 4/3. Faktorn 4/3 kommer av att kordan vägs med area: en triangelvinges
    medelkorda är ⅔ av rotkordan, mot S/b = ½. Det är tvådimensionell teori. DATCOM:s
    uppskattning för ändliga vingar är ungefär hälften så stor.
- **Tvinnat papper är en yta för luften** (`mergeTwisted` i `aeroMesh.ts`):
  - Vid ett gem tvinnas papperet av återfjädringen och delas i smala remsor (`springback.ts`).
    Varje remsa var en egen yta med egen framkant, egen lastfördelning och eget gränsskikt.
  - Därför flyttade ett gem baktill lyftcentrum 23 mm bakåt, och baktunga plan såg mycket
    stabilare ut än de är.
  - Nu blir närliggande nästan plana bitar av samma panel en yta i sitt medelplan. Bitarna får
    skilja högst 3°, men upp till 15° om de har samma vikhistorik (`foldPath`), alltså samma
    ovikta papper. Spelarens egna veck slås därför aldrig ihop.
  - Hårfina glipor mellan bitarna räknas inte som kanter, och köleffekten hittar de
    sammanslagna vingarna.
  - Rutnätet klipps kolumn för kolumn, så det går inte långsammare att bygga.
- **Mätning** (trimmad dart, flexibelt papper, 12 kast, 6 m/s, 5° uppåt, från 1,8 m):

  | Gem | Lyftcentrum bakom tyngdpunkten | Trimvinkel | Flygtid | Distans | Stegrar sig (> 60°) |
  |---|---|---|---|---|---|
  | Ett framtill | 46 mm | 3° | 0,78 s | 4,1 ± 0,1 m | 0 |
  | Inga | 22 mm | 7° | 1,00 s | 5,3 ± 0,3 m | 0 |
  | Ett baktill | 1 mm | 15° | 1,87 s | 7,1 ± 1,3 m | 0 (högst 54°) |
  | Två baktill | −8 mm | 36° | 1,46 s | 2,6 ± 0,3 m | 12 |
  | Tre baktill | −12 mm | – | 1,31 s | 2,5 ± 0,3 m | 12 |

  - Med två eller tre gem baktill reser planet nosen nästan lodrätt, överstegrar och faller.
    Förut sjönk det med två gem som en fallskärm (6,1 m).
  - Med ett gem baktill är planet nära neutralt. Det glider långt men varierar mest mellan kasten.
  - Rotationslyften ensam ändrar distansen med några procent. Platt dart +3 %, trimmad dart
    +3 %, glidplan +11 % (mer dämpning i tipp). Ett gem baktill −6 % och dubbelt så stor
    spridning. Den stora skillnaden för baktunga plan kommer från de sammanslagna remsorna.
  - Ett gem framtill gör att planet lutar mer åt sidan än utan gem. Det gjorde det redan förut
    (högst 63°), och med de nya ytorna blir det högst 75°.

## 33. Flexibla vingar som håller trimmen, och snitt ända ut till kanten
- **Felet:** i flexibelt papper (spelets standard) dök alla plan som trimmats med uppvikta
  vinghörn. Glidplanet kastat i glidfart från 30 m gick från −14° till −67° bana på 12 m, och från
  6 m höjd landade det efter 11 m (stelt papper: 30 m). Vanliga kast från 1,6 m blev ungefär
  hälften så långa som i stelt papper. Det började när förinställningarna bytte klaffar mot
  uppvikta hörn (§29). Med klaffar glider glidplanet.
- **Orsak:** vingarna böjde sig med framkanten uppåt vid spetsen (wash-in, 5–9° vid
  glidplanets spets). Spetsarna sitter bakom tyngdpunkten, så mer lyft där trycker ner nosen.
  Mest var det formen som välver vingen längs kordan, d̃²(s̃²−⅓). Tre saker saknades:
  - **Raka veck:** ett veck som bär en annan panel är en vinkelprofil, ungefär tusen gånger
    styvare längs sin längd än plant papper, även vid 10° vikvinkel. Den stela flik som hänger i
    det är också rak. Förut kunde vingen kröka sig längs det uppvikta hörnets veck som om hörnet
    inte fanns. Nu begränsas böjformerna till kombinationer där nedböjningen är linjär längs varje
    sådant veck (`keepCreasesStraight`), och panelen får fyra extra former (d̃³, d̃³s̃, d̃²s̃³,
    d̃³s̃²) så att villkoret bara styvar upp vingen nära vecket.
  - **Lager som böjer sig tillsammans** (`paper.stackStiffnessExponent` = 3): en ihoppressad
    bunt av n lager böjer sig som nᵏ enkla ark. k = 1 om lagren glider fritt mot varandra,
    k = 3 om de är limmade. Vikta lager hålls ihop av sina vikta kanter och är pressade platta,
    så de räknas som limmade. Förut böjde sig förinställningarnas framkanter, med 3–4 lager, som
    tre eller fyra lösa ark.
  - Det räcker inte med en av delarna. Raka veck utan styva lager, eller tvärtom, ger fortfarande
    dykning för W-darten. Utan de extra formerna blir en vinge som bär ett veck nästan helt stel.
- **Kontroll:** fler böjformer (upp till sex extra) ändrade inte resultatet före rättningen, så
  felet var inte för få former. En kontroll av aerodynamiken visade att en böjd korda borde ge
  ännu mer nosen nedåt, inte mindre: modellen väger varje elements egen vinkel med
  lastfördelningen (tyngst vid framkanten), medan tunnplatteteorin väger bakkanten tyngst. Det är
  kvar som det är.
- **Utfall** (6 kast med spridning per fart, från 1,6 m, 0° uppåt, medeldistans):

  | Plan | Stelt 5 / 8 / 11 m/s | Flexibelt förut | Flexibelt nu |
  |---|---|---|---|
  | Dart | 3,6 / 5,7 / 8,0 | 3,5 / 5,5 / 7,7 | 3,6 / 5,7 / 8,0 |
  | W-dart (platta vingar, uppvikta hörn) | 5,7 / 10,3 / 11,1 | 3,9 / 5,3 / 5,9 | 5,0 / 10,7 / 15,8 |
  | Trimmad dart | 5,7 / 9,9 / 10,4 | 4,0 / 5,3 / 6,1 | 5,1 / 10,8 / 15,5 |
  | Glidplan | 8,8 / 12,2 / 12,5 | 5,1 / 6,0 / 5,9 | 9,0 / 12,7 / 20,1 |

  - Vid hårda kast flyger flexibla plan längre än stela. Det stela planet stiger, tappar fart och
    sjunker. Det flexibla vrider spetsarna lite uppåt i farten, vilket dämpar stigningen.
  - Glid från 30 m i glidfart: alla prövade plan glider stadigt, W-darten på −16°, glidplanet
    −11° (stelt −15° och −12°). En dart med klaffar längs hela bakkanten glider brant (−34°, stelt
    −15°): bakkanten hålls rak av klaffvecket, och bara den främre spetsen böjer sig upp.
  - Vingarna böjer sig mindre: glidplanet med klaffar 2,5 mm i ett lugnt kast och 8 mm i ett hårt
    (förut 15 och 38 mm).
  - Ett steg tar upp till 30 % längre tid för plan med uppvikta hörn (glidplanet 0,64 ms).
- **Månen:** en brant och hård landning (14 m/s, 45°) gör glidplanet i flexibelt papper
  osymmetriskt när nosen skrynklas, och det rullar över en vinge och glider 45–60 cm. Förut gjorde
  samma sak 1 kast av 15 närliggande. Nu sker det oftare, eftersom styvare vingar tar upp mindre
  av stöten. Testet för det går inte igenom. Det är inte rättat.
- **Snitt ända ut till kanten** (`reachEdges` i `cut.ts`, saxen): ett snitt delar bara papperet
  där det dras. Ett snitt i kölen som slutade en halv millimeter före mittvecket lämnade en remsa
  som höll ihop fliken med kölen framför, så hela kölen följde med när fliken viktes upp. Saxen
  drar nu ett snitt som slutar högst 2 mm före en papperskant (i snittets plan, längs snittet)
  0,5 mm förbi kanten. Ett snitt som slutar längre in i papperet lämnas som det är.

## 34. Köleffekten borttagen, och varför dihedral vinglar
- **Köleffekten** (§30, `aero.keelDihedral`) är borttagen på användarens begäran. Den gav vingar
  ovanpå en köl en halvempirisk ekvivalent dihedral på ungefär 8° i sidglidning, oberoende av
  fart. Riktiga plan med plana vingar eller några graders anhedral tappar stabiliteten inom
  5–8 m, och med köleffekten var de i stället lika stadiga som plan med dihedral. Kvar finns bara
  markeringen av kölen under vingen (`underWing`, kölens sidokraft fördelad längs den).
- **Rollmoment i sidglidning**, Clβ per radian vid trimvinkeln (stelt, W-dart med platta vingar):

  | Källa | Bidrag |
  |---|---|
  | Köleffekten (borttagen) | −0,035 |
  | Pilvinkeln: lasten flyttas mot den vindvända vingen, och lyftlutningen | −0,031 |
  | De uppvikta hörnen | −0,016 |

  - Pilvinkelns bidrag är i nivå med DATCOM för en deltavinge med dartens pilvinkel (ungefär
    −0,06 till −0,09 vid CL 0,13), så det ändras inte.
  - Plana vingar slogs ihop till en yta och fick större Clβ än vingar ½° ur planet (−0,047 mot
    −0,024). Det berodde på rutnätet: med finare element går de separata vingarna mot samma
    värde (−0,033 vid 5 mm, −0,038 vid 2,5 mm). Det är alltså inget fel i sammanslagningen.
- **Kast med spridning** (10 kast, flexibelt papper, från 1,6 m, 5° uppåt). "Tappar" = lutar mer
  än 35° (L) eller svänger mer än 25° (S) före landningen:

  | Plan | 8 m/s | 11 m/s |
  |---|---|---|
  | Dart (otrimmad) | 0 tappar, 6,5 m | 0, 9,4 m |
  | 3° anhedral, W | 3 tappar (L) efter 4,8 m | 5 (L) efter 6,4 m |
  | Plana vingar, W | 0, 10,7 m | 2 (S) efter 12 m |
  | 3° dihedral, W | 2 (S) efter 9,5 m | 7 (S) efter 12,6 m |
  | 10° dihedral, W | 4 (S) efter 8,5 m | 7 (S) efter 9,6 m |
  | Plana vingar, W, gem i nosen | 0, 6,2 m | 0, 9,0 m |
  | Glidplan | 7 (S) efter 7,9 m | 9 (S) efter 11 m |

  - Med köleffekten tappade 3° anhedral bara 1 av 10 vid 8 m/s, och plan med gem i nosen tappade
    lutningen redan efter 3 m.
- **Varför dihedral vinglar och svänger:**
  - I stadigt glid utan spridning dämpas en lutning på 10° fort för alla dartar (holländsk roll
    väl dämpad), och de glider nästan rakt. Med gem i nosen blir spiralen långsamt divergent
    (riktningsstabiliteten ökar), och glidplanet svänger stadigt.
  - I kasten är det veckspridningen (`scatter.creaseDeg`, 3°) som avgör: utan den tappar 10°
    dihedral 2 av 10 vid 11 m/s i stället för 7. Två vingar med olika vinkel lutar lyftkraften
    olika och ger en sidokraft. Plana vingar glider då lite snett. Med dihedral blir sidglidningen
    lutning och sväng, som när ett modellflygplan svänger med bara sidroder. Det är fysikaliskt
    riktigt och ändras inte.
  - Plana vingar med W glider fortfarande stadigt. Pilvinkeln rätar upp dem.
- **Spetsöverstegring prövad, inte införd:** användaren beskriver att plana vingar lutar, svänger
  eller ibland stegrar sig efter en kort sträcka och sedan faller. Det liknar spetsöverstegring
  hos pilade vingar, och W, gem i nosen och dihedral är de klassiska botemedlen. Modellen ger
  varje element samma lyftkoefficient, så lasten längs spännvidden följer kordan. En prövning
  med Schrenks fördelning (medelvärdet av kordan och en elliptisk fördelning, så att spetsarna får
  högre lokal lyftkoefficient och överstegras först) ändrade nästan ingenting. Planen glider vid
  6–8° anfallsvinkel och minst 4–5 m/s, så spetsarna når aldrig överstegring.

## 35. Månlandningar (rättat i §41)
- **Läget:** testet med ett perfekt symmetriskt kast (14 m/s, 45°) klarade sig på tur. Med spelets
  spridning hoppar planen upp till 2–5 m och glider upp till 20–33 m efter nedslaget, både före
  och efter §33–34. Det strider mot användarens val för Månen: dammet ska stoppa papperet, och
  inga kullerbyttor.
- **Orsak:** när en spets slår i vid ungefär 9 m/s stoppar friktionen och plogningen den tvärt.
  Spetsen har liten effektiv massa, så planets fart blir snurr kring den, 1 300–2 700 °/s. Samma
  sak händer i sporthallen. På Månen bromsar ingen luft snurren efteråt, och nästa hörn som slår
  i lyfter planet i den svaga tyngden. Varje kontakt varar bara ett steg, så dammets dämpning av
  rotationen hinner nästan inte verka.
- **Prövat utan att det räckte:**
  - Kontaktlösare som inte beror på punkternas ordning (Jacobi): landningen blev inte symmetrisk.
  - Dämpa snurren innan stöten räknas: värre.
  - Ett dammlager på 3 mm där papperet plogar åt alla håll: medianen för en stel dart gick från
    5,3 till 0,7 m, men enstaka kast hoppar fortfarande nästan 2 m.
  - Starkare dämpning (600/s): medianen 0,3–0,5 m, men kast på 5–15 m kvar. Vid 2 000/s kommer
    planet aldrig till vila.
- **Trolig väg framåt:** papperet är inte stelt där det slår i. Spetsen skrynklas och böjer sig,
  stöten tar längre tid och energin går åt till att deformera papperet. I modellen registreras
  skrynklingen bara och ritas efteråt, och kontakten är en stel punkt.

## 36. Kölen lastas som andra ytor: plana vingar viker av
- **Användarens iakttagelse:** en dart med helt plana vingar är instabil och viker oftast av åt
  något håll inom 4–8 m, oavsett hur den släpps. Vingar med uppvikta eller nedvikta spetsar, eller
  ett W, är stabilare men vinglar lite i roll. Att både upp- och nedvikta spetsar hjälper betyder
  att det är riktningsstabiliteten (fenor bakom tyngdpunkten), inte rollstabiliteten.
- **Tidigare (§30):** kölens sidokraft spreds jämnt längs hela kölen (`underWing`), med motiveringen
  att vingen ovanför är en ändplatta som breddas hela vägen bakåt. Det gjorde den plana darten svagt
  riktningsstabil (Cnβ +0,004).
- **Varför det var fel:** i slank kroppsteori uppstår sidokraften där tvärsnittets medföljande
  luftmassa växer: där kölen blir djupare (de första ~6 cm) och där vingen bredvid breddas tills den
  är ungefär dubbelt så bred som kölen är djup. På en dart händer båda ungefär 12 cm från nosen,
  framför tyngdpunkten (16 cm). En uppskattning av det (m ∝ h²(1 + r²/(1 + r²)), r = s/h) lade
  kraften ännu längre fram, och då blev även plan med fenor på spetsarna instabila, tvärtom mot
  användarens iakttagelse. Strömningen släpper vid kölens vassa kant, vilket brukar ge mindre kraft
  vid nosen än teorin.
- **Nu:** kölen lastas som vilken yta som helst, från sin framkant (tryckcentrum vid ¼ korda).
  Specialfallet `underWing` och `markKeelJunctions` är borttagna.
- **Utfall** (8 kast med spridning, 8 m/s, 5° uppåt, flexibelt papper; "viker av" = kursen ändras
  mer än 20°):

  | Plan | Cnβ | Viker av | Anmärkning |
  |---|---|---|---|
  | Plana vingar | −0,031 | 7 av 8 | slår ofta runt efter 3,5–6 m |
  | Spetsar uppvikta 80° (2 cm) | +0,014 | 1 | stadig |
  | Spetsar nedvikta 80° | +0,018 | 1 | stadig |
  | Ett gem i nosen | +0,013 | 0 | rak, landar efter 5 m (utan gem 6 m) |
  | Två gem i nosen | +0,020 | 0 | rak, 5 m |
  | 10° dihedral | −0,019 | 6 | vinglar i roll, ungefär 2 växlingar per kast |
  | Grunt W (14° vid 45 % av spännvidden) | −0,031 | 8 | för litet för att verka som fena |
  | Trimmad dart (små uppvikta hörn) | −0,015 | 7 | |
  | Glidplan | −0,002 | 4 | landar ändå efter 12–17 m |

  - De små uppvikta bakre hörnen hjälper nästan inte: vid flygvinkeln står de ungefär 23° mot
    luften och har släppt strömningen, så de är svaga fenor.
  - Valideringens trimmade dart viker nu av vid 11 m/s (14 av 16 kast) i stället för att stegra
    sig. Testet för överstegring använder därför 13 och 15 m/s (5 och 7 av 16 stegrar sig).

## 37. Plan som aldrig stannade på golvet, och genomsnitt i resultatet
- **Användarens iakttagelse:** efter flera kast med W-vingar fortsatte planet långsamt snurra på
  golvet långt efter landningen.
- **Orsak 1, studs från en fjäder:** ett uppvikt hörn som hamnat under vingen, klämt mot golvet,
  trycktes ner av sitt veck snabbare än studsgränsen (0,3 m/s) inom ett enda steg. Golvet studsade
  det, vecket tryckte ner det igen, och svängningen (120 Hz) fick planet att krypa och vrida sig
  i 30 s (1,7 cm/s, några grader på 10 s).
  - Nu studsar papper bara mot golvet om det var på väg mot det redan före steget
    (`MultiBodySim.contact`). Studsens storlek är oförändrad. En variant som också tog studsens
    storlek från farten före steget gjorde enstaka hårda landningar instabila.
- **Orsak 2, vila räknad på varje svängning:** en flik klämd mellan vingen och golvet vid sitt
  stopp vibrerade lite, och planets rotation låg precis över gränsen för vila (0,112 mot
  0,1 rad/s). Planet stod still men flygningen tog aldrig slut, så resultatet kom först efter 30 s.
  - Nu räknas vila på planets medelrörelse: hastighet och rotation medelvärdesbildas som vektorer
    (båda simulatorerna). En vibration fram och tillbaka tar ut sig själv, en verklig glidning
    eller snurr gör det inte.
- **Krasch:** "singular matrix" i den flexibla modellen avbröt spelet. Nu avslutas flygningen i
  senaste goda läge, som vid andra numeriska haverier.
- **Utfall** (120 kast med W-vingar, uppvikta spetsar, uppvikta hörn och glidplanet, flexibelt
  papper, 5–14 m/s): alla utom två kommer till vila 1,1–2,6 s efter landningen. Förut stannade
  två kast aldrig och ett kraschade. De två som återstår avbryts av skyddet mot orimliga värden
  strax efter ett hårt nedslag, och planet stannar där det är.
- **Genomsnitt i resultatet:** rutan efter tävlingens tredje kast, och kastkortet när tävlingen är
  slut, visar genomsnittet av kasten (`averageScore`). Kast utan giltig landning räknas inte, och
  då står det till exempel "(2 av 3 kast)".

## 38. Sneda veck (inbyggda, inte påslagna)
- **Varför:** ett handvikt plan är aldrig symmetriskt. Ett vingveck som ligger 1° snett ger den
  vingen ungefär 1° annan anfallsvinkel, och en dart med plana vingar har nästan inget som rätar upp
  den. Det kan vara det som får riktiga plana dartar att tippa åt sidan inom några meter.
- **Modell** (`paper/crooked.ts`): varje vikningslinje, och varje spegeltvilling för sig, vrids i
  sitt plan kring sin mitt (normalfördelat, `scatter.foldLineDeg` = 1°, avskuret vid 2,5σ). Det
  sneda planet viks sida vid sida med det exakta, och varje senare steg flyttas med papperet under
  det (stegen är givna i det exakt vikta planets koordinater). Spegelsteg görs som två separata
  steg. Snedheten bestäms av designens steg, så samma plan är lika snett varje kast och
  uppspelningar blir exakta. Kan planet inte vikas snett används det exakta.
- **Inte påslaget:** `prepare(design, cfg, { crooked: true })` används inte av spelet än. Tester och
  vindtunneln är exakta.
- **Prov** (10 kast, flexibelt papper): sneda veck ensamt ändrar inte att den plana darten vrider
  sig runt (Cnβ −0,031). Tillsammans med kölens sidokraft något längre bak (Cnβ ≈ −0,008) tippar den
  plana darten åt sidan i 6 av 10 kast och flyger 6,5 m, som användaren beskriver. Men plan med
  uppvikta spetsar och W tippar då också (6–7 av 10), vilket inte stämmer. Alla dessa dartar flyger
  nästan utan lyft (trimvinkel 0,2–0,6°), och utan lyft finns ingen rollstabilitet från
  pilvinkeln. En uppböjd bakkant som gav lyft (trim 6–8°) gjorde att den plana darten vred sig
  oftare. Arbetet är pausat.

## 39. Kölens sidokraft delvis utspridd (`aero.keelSideSpread` = 0,6)
- **Problem efter §36:** med hela kölens sidokraft från framkanten var den plana darten så
  riktningsinstabil (Cnβ −0,031) att nosen svängde 100–180° på under en sekund. Planet gick snett
  eller baklänges genom luften och tycktes stanna efter 3–3,5 m. Riktiga plana dartar tippar i
  stället åt sidan och sjunker medan de fortsätter framåt.
- **Provmatris** (10 kast per fall, 7–11 m/s, 5° uppåt, flexibelt papper; "tippar" = lutar mer än
  30° vid landningen, så lutning direkt efter släppet räknas inte): sidokraftens fördelning, sneda
  veck (§38) och lyft från en uppböjd bakkant prövades i kombination.
  - Lyft fick plan med uppvikta eller nedvikta spetsar att dras in i en spiral (lutar 30–80° vid
    landningen, kursen ändras 30–40°), tvärtom mot användarens iakttagelse att de bara vinglar lätt.
  - Sneda veck fick plana vingar att tippa (6 av 10), men även spets- och W-plan landade lutande.
  - Ingen kombination gav både plana vingar som tippar och spets- och W-plan som är stabila.
- **Valt (användarens val):** 60 % av sidokraften på en köl under en vinge fördelas jämnt längs
  kölen, resten lastas från framkanten. Halvempiriskt, mellan en ensam köl (allt framtill) och §30
  (allt utspritt).

  | Plan | Förut (0) | Nu (0,6) |
  |---|---|---|
  | Plana vingar | vrider sig 10/10, kursen 54°, 5,0 m | vrider sig 5/10, tippar 1/10, kursen 40°, 6,4 m |
  | Spetsar uppvikta 80° | stabil, 8° | stabil, 6° |
  | Spetsar nedvikta 80° | stabil, 3° | stabil, 5° |
  | Djupt W | tippar 5/10, 16° | stabilt, 4° |
  | Trimmad dart | vrider sig 4/10, tippar 5/10, 7,0 m | tippar 1/10, 11,7 m |
  | Glidplan | 16,3 m | 15,5 m |

  - Cnβ för den plana darten: −0,031 → −0,008. Med spetsar +0,041/+0,047.
- **Kvar:** den plana darten tippar fortfarande sällan och vrider sig i stället hälften av gångerna.
  Modellen saknar något som gör att spets- och W-plan rättar upp en sned vikning mycket bättre
  än plana vingar.
- **Rättat i förbigående:** `multibody.ts` hade formaterats om av Prettier med standardinställningar
  (dubbla citattecken, 793 ändrade rader) i §37. Filen har sin ursprungliga form igen, med samma
  ändringar som i §37. Steget är uppdelat i `substep` och `afterStep`.

## 40. Tejp över nosen gjorde vingarna stela
- **Användarens iakttagelse:** ett plan med en tejpbit som håller ihop nosen flög i regel flera
  meter kortare än samma plan utan tejp, både i hårda och medelhårda kast.
- **Orsak 1:** tejp över skarven mellan vinghalvorna pressar ihop lagren under sig. Även lagren
  på båda sidor om skarven slogs ihop till en panel, så båda vingarna och kölen blev en stel kropp.
  Den kroppen blev rot, eller var inte plan nog, och då fick ingen panel några böjformer. Vingarna
  kunde varken böja sig eller vrida sig var för sig i vingroten. Den flexibla modellen betedde sig
  som stelt papper, och i hårda kast flyger flexibla plan längre (§33).
  - Nu pressar tejpen ihop lagren på varje sida för sig. En 2 cm tejpbit vid nosen håller ihop
    halvorna där men hindrar inte vingarna från att böja sig längre ut (`buildPanels`).
- **Orsak 2:** tejpen håller vecken stängda mot återfjädringen, och längre bak öppnar de sig, så
  papperet vrids 1–2°. De lager som är vikta ovanpå vingen (glidplanets nostriangel och hörn) låg då
  inte längre exakt i vingens plan och blev en egen panel bredvid vingen, på samma vingrot.
  - Nu räknas lager inom 3° och 2 mm från varandra, som överlappar, som en ihoppressad bunt
    (`nearlyStacked`).
- **Utfall** (16 kast med spridning, flexibelt papper, 5° uppåt, medeldistans):

  | Plan | 6 m/s utan / med tejp förut | nu | 10 m/s utan / med tejp förut | nu |
  |---|---|---|---|---|
  | Trimmad dart | 6,6 / 7,0 m | 6,6 / 6,2 m | 12,4 / 9,7–13,1 m | 12,4 / 12,3 m |
  | Glidplan | 10,1 / 7,9 m | 10,1 / 9,7 m | 17,1 / 10,8 m | 17,1 / 15,6 m |

  - Det som är kvar för glidplanet (cirka 1,5 m vid 10 m/s) finns även i stelt papper. Tejpen
    håller nosens veck stängda, och planet får cirka 3,5 % mindre lyft. Vid ett hårt kast stiger
    glidplanet till drygt 4 m, överstegrar, dyker och tar upp dyket bara centimeter över golvet. Med
    lite mindre lyft når det golvet oftare i dyket. Utan återfjädring är skillnaden borta.

## 41. Månlandningar: papperet sjunker in i dammet
- **Felet (§35):** med spridning hoppade plan upp till 2–5 m och gled upp till 20–33 m efter
  landningen på Månen. Den stela golvytan stoppade spetsen som slog i på ett steg, planets fart
  blev snurr kring den (1 300–2 700 °/s), och nästa hörn lyfte planet i den svaga tyngden.
- **Modell** (`ground.dustDepthM`, Månen 3 cm): det översta lagret regolit är löst (fotspår är
  ungefär 3 cm djupa). Fast mark ligger 3 cm under ytan.
  - Papper i lagret som rör sig fortare än 1 m/s plogar genom det åt alla håll, inte bara i
    sidled (samma ρ·A·v² som `ploughKgPerM`, aldrig så att rörelsen vänds).
  - Papper som rör sig långsammare än 1 m/s vilar på dammet som på ett golv där det är
    (`DUST_SINK_SPEED`): löst damm bär ett papper som ligger still, bara ett nedslag sjunker in.
    Först villkoret på farten nedåt gav tillbaka de långa glidningarna, eftersom papper som rörde
    sig långsamt nedåt men fort i sidled fick golvets fulla friktion och spetsen grep igen.
  - Dammet tar upp snurren så länge papper är i lagret (`spinDampingPerS`, som förut).
  - Båda simulatorerna. Sporthallen och Solen har inget lager.
- **Utfall** (12 kast med spridning, 14 m/s, 45° uppåt; glider = från första nedslaget till vila):

  | | Typiskt | Längst | Hoppar högst (tyngdpunkt) |
  |---|---|---|---|
  | Dart, stelt, förut | 5,4 m | 20,5 m | 3,0 m |
  | Dart, stelt, nu | 0,22 m | 0,43 m | 0,15 m |
  | Glidplan, stelt, förut | 3,1 m | 32,8 m | 3,9 m |
  | Glidplan, stelt, nu | 0,19 m | 0,93 m | 0,28 m |
  | Dart, flexibelt, nu | 0,11 m | 0,61 m | 0,31 m |
  | Glidplan, flexibelt, nu | 0,11 m | 0,41 m | 0,29 m |

  - Planen vilar högst ungefär 2 cm nersjunkna i dammet.
  - Andra lager prövades: 1 cm gav fortfarande kast på 17 m, 2 cm på 5,5 m, 3 cm inga över 1 m.
- **Testet** för hårda landningar på Månen är påslaget igen och kontrollerar också två kast med
  spridning (där felet syntes), i stelt och flexibelt papper.

## 42. Plana dartar fortsätter framåt, och spelet viker snett
- **Användarens iakttagelse:** riktiga dartar med plana vingar tappar sällan helt stabiliteten och
  lyftkraften som i spelet. De är ostadiga i roll: efter några meter börjar de rolla mot 90°,
  tappar lyft och höjd men fortsätter framåt eller i en vid sväng. Uppvikta eller nedvikta spetsar,
  W, dihedral och lite anhedral gör dem stadigare i roll, så de flyger längre. För mycket anhedral
  vänder planet upp och ned, och för mycket dihedral tappar lyft men flyger rakt tills det tar mark.
- **Vad spelet gjorde** (16 kast per fall med spridning, 7 och 10 m/s, 10° uppåt, flexibelt
  papper; "vrider sig" = sidglidning över 40°): den plana darten rollade nästan aldrig. Den vred
  nosen runt i 5–9 av 16 kast, kölen kom på tvären mot luften och bromsade, och planet tycktes
  stanna. 15° dihedral vinglade med 60–90° i roll och slog runt i 10 av 16 kast vid 10 m/s.
- **Orsak:** med 60 % av kölens sidokraft utspridd (§39) var den plana darten fortfarande neutral
  eller instabil i girled (Cnβ ≈ 0).
- **Prövat:**
  - Hela sidokraften utspridd (`keelSideSpread` = 1): inga plan vrider sig runt, dihedral flyger
    rakt, men den exakt vikta plana darten flyger också rakt.
  - Sneda veck (§38) med 1°: alla plan rollar runt, med eller utan fenor. Med 0,3° (ungefär 1 mm
    på ett 20 cm långt veck) tippar plana dartar ibland.
  - Kölens sidokraft dubblad (vingen som spegelplan för kölen): Clβ ändras bara 0,002–0,004.
  - I modellen ger spetsarna mest riktningsstabilitet (Cnβ +0,05) men lite rollstabilitet
    (Clβ −0,012 uppvikta, +0,005 nedvikta, mot −0,004 för plana vingar). W (−0,036) och dihedral
    (−0,026 vid 5°) rätar upp planet i roll.
- **Valt (användarens val):** `keelSideSpread` = 1 och sneda veck i spelet (`scatter.foldLineDeg`
  = 0,3°, `prepare(…, { crooked: true })` i kastläget). Snedheten beror på designens steg, så
  samma plan är lika snett varje kast. Ändrar man vikningen blir det snett på ett nytt sätt.
  Uppspelningar från topplistan flygs också med det sneda planet.
- **Utfall** med båda ändringarna, 16 kast med olika snedhet per fall, 7 / 10 m/s:

  | Plan | Tippar (lutar över 60° eller slår runt) | Vrider sig runt |
  |---|---|---|
  | Plana vingar | 1 / 1 | 1 / 3 (förut 5 / 9) |
  | 5° dihedral | 1 / 2 | 0 |
  | 15° dihedral | 2 / 2 (förut 6 / 12) | 0 |
  | Spetsar uppvikta | 2 / 6 | 0 |
  | Spetsar nedvikta | 3 / 6 | 0 |
  | W | 0 / 3 | 0 |
  | 5° anhedral | 1 / 3 | 9 / 7 |
  | 15° anhedral | 15 / 16 (slår runt) | 1 / 0 |

- **Kvar:** plan med spetsar tippar lika ofta som plana vingar eller oftare, tvärtom mot
  användarens prov. Modellen saknar något som gör spetsarna till rollstöd. Förinställningen Dart
  råkar vara nästan rakt vikt (lutar högst 18° i 10 kast), så den tippar sällan.
- **Test:** den plana darten vrider inte nosen runt (sidglidning under 40°) i 12 kast med
  spridning (`tests/validation/flight.test.ts`). Testet för kölen kräver nu svag riktningsstabilitet.
