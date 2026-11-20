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
  ytor. Planet och fysiken är desamma överallt. Gymnastiksalen är alltid vald när spelet startar.
  - **Gymnastiksal:** 30 × 16 m med 7 m i takhöjd, g = 9,81, luft. Planet kastas 3 m från ena
    kortväggen.
    - Golv, väggar och tak är kontaktytor, så planet stannar mot väggen. Distansen räknas till
      första nedslaget, på golvet eller i en vägg.
    - Golvet har linjer för basket (rött), volleyboll (gult), badminton (blått) och
      handboll/innebandy (svart), och distanslinjer från kastpunkten varje meter.
    - Väggarna har träpaneler, och salen har korgar, handbollsmål, ribbstolar, bänkar,
      fönster, tak med lampor och mattor.
  - **Månen:** g = 1,62 och ingen atmosfär (ρ = 0). Inget lyft, inget motstånd och ingen
    tillskottsmassa, så planet flyger en ren kastparabel. Testerna jämför med den analytiska
    parabeln.
    - Det finns ett grått regolit med stenar, rundade kullar vid horisonten, svart himmel,
      hårt lågt solljus och distanslinjer varje 5 m.
  - **Solen:** g = 274, ingen yta att landa på. Efter 0,2 s brinner papperet upp.
    - Planet blir svarta flagor i pixelstorlek som först faller isär och sedan stiger uppåt
      och försvinner inom 0,5 s. Därefter visas "Flygplanet brann upp".
    - I vikläget ligger en liten askhög på bordet tills man trycker på Reparera, som ger ett
      nytt ark vikt likadant. Att ladda en modell eller börja om tar också bort askan.
- **Kontaktytor:** `GroundConfig.planes` är en lista av plan (fritt där n·x ≥ d). Båda
  simulatorerna löser kontakt mot alla plan med samma impulsmodell.
- **Topplistor per plats och spelläge.** Gymnastiksalens listor behåller de gamla nycklarna, och
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
- **Visning:** i kastläget ritas papperet med trianglar på högst 12 mm (`PaperView` med `maxEdge`).
  Varje bildruta flyttas hörnen och kantlinjerna längs böjningen
  (`Simulator.panelBending` → `FlightView.setPanelBending`), och ljusnormalerna lutas med ∇w.
