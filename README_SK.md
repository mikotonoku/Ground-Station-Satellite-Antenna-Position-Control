# Riadenie polohy antény s DC servomotorom 🛰️

![MATLAB](https://img.shields.io/badge/MATLAB-R2026b-0076A8?logo=Mathworks&logoColor=white)
![Simulink](https://img.shields.io/badge/Simulink-Model--Based%20Design-FF6F00?logo=Mathworks&logoColor=white)
![Control System Toolbox](https://img.shields.io/badge/Control%20System-Toolbox-00629B)
![Fuzzy Logic Toolbox](https://img.shields.io/badge/Fuzzy%20Logic-Toolbox-6A1B9A)
![PID Control](https://img.shields.io/badge/Control-PID-2E8B57)
![LQR](https://img.shields.io/badge/Control-LQR-8A2BE2)
![Fuzzy PI](https://img.shields.io/badge/Control-Fuzzy--PI-FF8C00)
![Academic Project](https://img.shields.io/badge/Type-Academic%20Project-success)
[![Research Basis](https://img.shields.io/badge/Research%20Basis-IEEE%20Access%202023-00629B)](https://doi.org/10.1109/ACCESS.2023.3313976)

## O projekte

Tento repozitár obsahuje môj projekt z predmetu **RMS**, ktorý sa zaoberá modelovaním a riadením uhlovej polohy pozemnej satelitnej antény poháňanej DC servomotorom. Práca vychádza z koncepcie riadiaceho systému uvedenej v článku IEEE Access *Enhanced Antenna Positioning Control System Using Adapted DC Servo Motor and Fuzzy-PI Controller*.

Cieľom je nastaviť anténu na požadovaný uhol tak, aby poloha motora sledovala referenciu presne, rýchlo a s prijateľným prekmitom a osciláciami. Skript MATLAB definuje parametre motora, vytvára prenosovú funkciu aj stavový model a pripravuje regulátory. Model Simulink realizuje uzavreté spätnoväzbové slučky a zobrazuje referenčnú aj skutočnú polohu motora.

V rámci rovnakého modelu sú porovnávané tri stratégie riadenia: automaticky naladený PID regulátor, LQR regulátor s úplnou stavovou spätnou väzbou a Mamdaniho Fuzzy-PI regulátor s adaptívnym proporcionálnym a integračným zosilnením. Repozitár slúži ako reprodukovateľná simulačná štúdia aj ako úvodný príklad, ako spolu súvisia matematické modelovanie, návrh regulátora a validácia v Simulinku pri úlohe polohovania antény.

> English version: [README.md](README.md)

---

<a id="obsah"></a>

## 📑 Obsah

- 🎯 [Cieľ projektu](#ciel-projektu)
- 📚 [Vedecký základ](#vedecky-zaklad)
- 🧭 [Mapa učenia: technológie a pojmy](#mapa-ucenia)
- 📐 [Matematický model](#matematicky-model)
- 🎛️ [Regulátory](#regulatory)
- 🛠️ [Implementácia repozitára](#implementacia)
- ⚙️ [Parametre a logika ladenia](#parametre-ladenie)
- 📈 [Výsledky a interpretácia](#vysledky)
- ▶️ [Začíname](#zaciatok)
- 📂 [Štruktúra repozitára](#struktura)
- ⚠️ [Obmedzenia](#obmedzenia)
- 🖼️ [Zoznam obrázkov](#zoznam-obrazkov)
- 📊 [Zoznam tabuliek](#zoznam-tabuliek)
- 📐 [Zoznam rovníc](#zoznam-rovnic)
- 📖 [Literatúra](#literatura)
- 👩‍🎓 [Autorka](#autorka)

---

## 🗂️ Zoznam obrázkov, tabuliek a rovníc

<a id="zoznam-obrazkov"></a>

### 🖼️ Zoznam obrázkov

1. [Obrázok 1. PID riadiaca slučka v Simulinku](#fig-pid-sk)
2. [Obrázok 2. LQR slučka so stavovou spätnou väzbou v Simulinku](#fig-lqr-sk)
3. [Obrázok 3. Mamdaniho Fuzzy-PI slučka v Simulinku](#fig-fuzzy-sk)
4. [Obrázok 4. Porovnanie simulovaných odoziev polohy antény](#fig-results-sk)

<a id="zoznam-tabuliek"></a>

### 📊 Zoznam tabuliek

1. [Tabuľka 1. Mapa učenia: technológie a pojmy](#tab-learning-sk)
2. [Tabuľka 2. Použité údaje motora](#tab-motor-sk)
3. [Tabuľka 3. Parametre a logika ladenia](#tab-tuning-sk)
4. [Tabuľka 4. Výsledky a interpretácia](#tab-results-sk)

<a id="zoznam-rovnic"></a>

### 📐 Zoznam rovníc

1. [Rovnica 1. Stavový vektor](#eq-state-vector-sk)
2. [Rovnica 2. Paralelný PID zákon](#eq-pid-law-sk)
3. [Rovnica 3. Prenosová funkcia filtrovaného PID](#eq-filtered-pid-sk)
4. [Rovnica 4. Kritérium kvality LQR](#eq-lqr-index-sk)
5. [Rovnica 5. Škálovanie zosilnení Fuzzy-PI](#eq-fuzzy-gains-sk)
6. [Rovnica 6. Elektrický a mechanický model DC motora](#eq-motor-dynamics-sk)
7. [Rovnica 7. Prenosová funkcia napätie–poloha](#eq-transfer-function-sk)
8. [Rovnica 8. Stavový opis](#eq-state-space-sk)

[Späť na obsah](#obsah)

---

<a id="ciel-projektu"></a>

## 🎯 Cieľ projektu

Repozitár je mojou implementáciou úlohy polohovania antény pomocou DC servomotora. Model prijíma požadovaný uhol antény a simuluje polohu motora. Použila som ho na preskúmanie troch prístupov k spätnoväzbovému riadeniu:

1. **PID** – automaticky naladený paralelný PID regulátor s filtrom derivácie;
2. **LQR** – úplná stavová spätná väzba navrhnutá z kontinuálneho stavového modelu;
3. **Fuzzy-PI** – Mamdaniho inferenčný systém, ktorý prispôsobuje proporcionálne a integračné zosilnenie podľa regulačnej odchýlky a jej zmeny.

Projekt je určený ako prvý praktický úvod do teórie riadenia: odvodiť model sústavy, vytvoriť vykonateľnú simuláciu, naladiť regulátory a porovnať sledovanie referencie namiesto voľby zosilnení ako náhodných konštánt.

[Späť na obsah](#obsah)

<a id="vedecky-zaklad"></a>

## 📚 Vedecký základ

Model vychádza z článku Fkirina a Khiru *Enhanced Antenna Positioning Control System Using Adapted DC Servo Motor and Fuzzy-PI Controller* (IEEE Access, 2023) [1]. Článok skúma anténu pozemnej prijímacej stanice poháňanú DC servomotorom T-36001 a porovnáva PID, LQR a Fuzzy-PI prístupy.

Tento repozitár realizuje myšlienku modelovania a rodiny regulátorov z článku ako vzdelávací experiment MATLAB/Simulink. **Nejde** o tvrdenie, že prebehla hardvérová validácia, ani o presnú reprodukciu každého nastavenia regulátorov a každého výsledku z článku. Hmotnosť LQR `R = 10`, medzná frekvencia PID a škálovania Fuzzy-PI sú nastavenia použité v tomto projekte a sú zdokumentované nižšie.

Článok je dostupný podľa vlastnej licencie Creative Commons Attribution-NonCommercial-NoDerivatives. Pri využívaní jeho myšlienok alebo nezmeneného článku citujte pôvodných autorov.

[Späť na obsah](#obsah)

<a id="mapa-ucenia"></a>

## 🧭 Mapa učenia: technológie a pojmy

<a id="tab-learning-sk"></a>

**Tabuľka 1. Mapa učenia: technológie a pojmy.**

| Téma | Význam | Kde ju nájsť v repozitári |
|---|---|---|
| MATLAB | Numerické prostredie na definovanie parametrov, prenosových funkcií, stavových modelov a regulátorov. | [`Zadanie_1.m`](Zadanie_1.m) |
| Simulink | Prostredie blokových schém na zostavenie spätnoväzbových slučiek a simuláciu v čase. [2] | [`Zadanie_1_sim.slx`](Zadanie_1_sim.slx) |
| Prenosová funkcia | Vstupno-výstupný opis v Laplaceovej oblasti: `G(s) = Theta(s)/Va(s)`. | `num`, `den` a `G = tf(num,den)` |
| Stavový model | Vektorový model prvého rádu `xdot = Ax + Bu`, `y = Cx + Du`; potrebný pre LQR. | `A`, `B`, `C`, `D` v `Zadanie_1.m` |
| PID riadenie | Spätnoväzbový zákon z proporcionálnej, integračnej a derivačnej zložky. [3][4] | PID vetva modelu |
| LQR | Metóda stavovej spätnej väzby minimalizujúca kvadratický kompromis medzi stavovou odchýlkou a riadiacim úsilím. [5] | LQR vetva a `lqr(A,B,Q,R)` |
| Fuzzy inferencia | Regulátor založený na pravidlách, jazykových premenných a Mamdaniho inferencii. [6] | [`Fuzzy_PI.fis`](Fuzzy_PI.fis) |
| Validácia simuláciou | Porovnanie sledovania referencie, prekmitu, oscilácií a ustálenia pred testovaním hardvéru. | Scope bloky Simulinku |

### MATLAB a Simulink

MATLAB poskytuje numerický model a príkazy na návrh regulátorov. Simulink premieňa rovnice na prepojené bloky, takže možno vizuálne sledovať signály a spätnoväzbové cesty. Ide o postup Model-Based Design: jeden model sa používa na formuláciu, simuláciu aj analýzu sústavy pred fyzickou implementáciou. [2]

### Prenosové funkcie a stavové premenné

Prenosová funkcia je užitočná, keď nás zaujíma, ako sa mení výstupná poloha pri zmene napätia kotvy. Stavový model je užitočný, keď sú dôležité aj vnútorné veličiny, napríklad prúd kotvy a uhlová rýchlosť. Stavový vektor je

<a id="eq-state-vector-sk"></a>

**Rovnica 1. Stavový vektor.**

$$x = \begin{bmatrix} i_a & \omega & \theta \end{bmatrix}^{T},$$

kde $i_a$ je prúd kotvy, $\omega$ je uhlová rýchlosť a $\theta$ je uhlová poloha v radiánoch.

### PID riadenie jednoducho

Pre odchýlku $e(t)=r(t)-\theta(t)$ je paralelný spojitý PID regulátor

<a id="eq-pid-law-sk"></a>

**Rovnica 2. Paralelný PID zákon.**

$$u(t)=K_p e(t)+K_i\int_0^t e(\tau)\,d\tau+K_d\frac{de(t)}{dt}.$$

- $K_p$ reaguje na aktuálnu odchýlku a zvyčajne zrýchľuje odozvu.
- $K_i$ akumuluje minulú odchýlku a môže odstrániť ustálenú chybu, ale príliš veľká integračná zložka môže spôsobiť prekmit alebo pomaly doznievajúce oscilácie.
- $K_d$ reaguje na trend odchýlky a môže zvýšiť tlmenie; v praxi sa derivácia filtruje, pretože zosilňuje vysokofrekvenčný šum.

Reprezentácia `pid` použitá funkciou `pidtune` obsahuje filter derivácie prvého rádu [4]:

<a id="eq-filtered-pid-sk"></a>

**Rovnica 3. Prenosová funkcia filtrovaného PID.**

$$C(s)=K_p+\frac{K_i}{s}+\frac{K_d s}{T_f s+1}.$$

### LQR jednoducho

LQR volí stavovú spätnoväzbovú akciu $u=-Kx$ tak, aby minimalizoval

<a id="eq-lqr-index-sk"></a>

**Rovnica 4. Kritérium kvality LQR.**

$$J=\int_0^\infty \left(x^TQx+u^TRu\right)dt.$$

`Q` vyjadruje, ako silno sú penalizované odchýlky stavov; `R` vyjadruje cenu veľkého riadiaceho vstupu. MATLAB vypočíta maticu zosilnenia `K` z príslušnej Riccatiho rovnice. [5] LQR predpokladá, že stavy použité pre spätnú väzbu sú dostupné alebo odhadnuté.

### Fuzzy-PI jednoducho

Fuzzy logika nahrádza jeden pevný vzorec zosilnení pravidlami typu „ak je odchýlka kladne veľká a rýchlo sa mení, použi vysoké/nízke zosilnenie“. Regulátor má dva vstupy – `Error` a `ChangeError` – a dva výstupy, normalizované `Kp` a `Ki`. Používa sedem gaussovských funkcií príslušnosti pre každý vstup a 49 Mamdaniho pravidiel. Skutočné zosilnenia vzniknú škálovaním výstupov:

<a id="eq-fuzzy-gains-sk"></a>

**Rovnica 5. Škálovanie zosilnení Fuzzy-PI.**

$$K_p(t)=G_{Kp}K_{p,\mathrm{fuzzy}}(t),\qquad
K_i(t)=G_{Ki}K_{i,\mathrm{fuzzy}}(t).$$

[Späť na obsah](#obsah)

<a id="matematicky-model"></a>

## 📐 Matematický model

Elektrický obvod kotvy a mechanická dynamika sú opísané vzťahmi

<a id="eq-motor-dynamics-sk"></a>

**Rovnica 6. Elektrický a mechanický model DC motora.**

$$V_a(t)=R_a i_a(t)+L_a\frac{di_a(t)}{dt}+K_b\omega(t),$$

$$J_m\frac{d\omega(t)}{dt}+B_m\omega(t)=K_t i_a(t),\qquad \dot{\theta}(t)=\omega(t).$$

Pri nulových počiatočných podmienkach má prenosová funkcia napätie–poloha tvar [1]

<a id="eq-transfer-function-sk"></a>

**Rovnica 7. Prenosová funkcia napätie–poloha.**

$$G(s)=\frac{\Theta(s)}{V_a(s)}=
\frac{K_t}{s\left[(R_a+L_a s)(J_m s+B_m)+K_tK_b\right]}.$$

Ekvivalentný stavový opis použitý pri LQR je

<a id="eq-state-space-sk"></a>

**Rovnica 8. Stavový opis.**

$$\dot{x}=Ax+Bu,\qquad y=Cx+Du,$$

$$A=\begin{bmatrix}
-R_a/L_a & -K_b/L_a & 0\\
K_t/J_m & -B_m/J_m & 0\\
0 & 1 & 0
\end{bmatrix},\quad
B=\begin{bmatrix}1/L_a\\0\\0\end{bmatrix},\quad
C=\begin{bmatrix}0&0&180/\pi\end{bmatrix}.$$

`C` prevádza simulovanú polohu z radiánov na stupne pre výstup stavového modelu. Fyzikálny model regulátora pracuje v radiánoch: `Deg` je používateľsky čitateľný cieľ v stupňoch, zatiaľ čo `Ref = Deg*pi/180` je príslušná referencia v radiánoch.

### Použité údaje motora

<a id="tab-motor-sk"></a>

**Tabuľka 2. Použité údaje motora.**

| Symbol | Význam | Hodnota použitá v kóde |
|---|---|---:|
| $R_a$ | Odpor kotvy | 13.3 Ohm |
| $L_a$ | Indukčnosť kotvy | 29 H |
| $J_m$ | Moment zotrvačnosti rotora | 51.386 |
| $J_a$ | Moment zotrvačnosti záťaže (deklarovaný pre štúdiu) | 3.5946 |
| $B_m$ | Koeficient viskózneho trenia | 13.558 |
| $K_t$ | Momentová konštanta | 138 |
| $K_b$ | Konštanta spätného elektromotorického napätia | 187 |
| `Deg` | Predvolený referenčný uhol | 150 deg |

Číselné hodnoty sú prevzaté zo zdrojového kódu projektu a vychádzajú z údajov motora použitých ako základ článku. Pôvodný kontext a jednotky sú uvedené v citovanom článku. [1]

[Späť na obsah](#obsah)

<a id="regulatory"></a>

## 🎛️ Regulátory

### Vetva PID

<a id="fig-pid-sk"></a>

**Obrázok 1. PID riadiaca slučka v Simulinku.** Referenčná poloha sa porovnáva s nameranou polohou motora; blok PID prevádza regulačnú odchýlku na riadiaci signál motora.

<p align="center">
  <img width="793" height="147" alt="Schéma PID riadenia" src="https://github.com/user-attachments/assets/23735932-f219-45c6-88d0-50e975695eab" />
</p>

Skript vytvorí model rastliny pomocou `tf` a regulátor získa programovo:

```matlab
wc = 0.5;                     % cieľová medzná frekvencia [rad/s]
C_pid = pidtune(G, 'PID', wc);

Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
```

Nejde o pokus–omyl: `pidtune` navrhne PID regulátor pre lineárny model a usiluje sa vyvážiť výkon a robustnosť. [3][7] Zosilnenia nie sú pevne zapísané; pri každom spustení skriptu sa prepočítajú z aktuálneho modelu rastliny.

### Vetva LQR

<a id="fig-lqr-sk"></a>

**Obrázok 2. LQR slučka so stavovou spätnou väzbou v Simulinku.** Stavový vektor sa násobí vypočítaným zosilnením `K_lqr`; výsledná riadiaca akcia budí stavový model motora.

<p align="center">
  <img width="1360" height="217" alt="Schéma LQR riadenia" src="https://github.com/user-attachments/assets/e7e5f1f7-9897-424f-a535-fc09efdf6fab" />
</p>

Projekt používa úplnú stavovú spätnú väzbu a vypočíta `K_lqr` pomocou funkcie MATLAB `lqr`.

```matlab
Q = 1e6 * eye(3);
R = 10;
K_lqr = lqr(A, B, Q, R);
```

Matica `Q` v tomto experimente rovnako váži prúd, rýchlosť a uhol. Skalár `R=10` je hmotnosť riadiaceho úsilia zvolená pre štúdium odozvy v tomto repozitári. Zmena ktorejkoľvek matice mení kompromis, zosilnenie aj odozvu.

### Vetva Fuzzy-PI

<a id="fig-fuzzy-sk"></a>

**Obrázok 3. Mamdaniho Fuzzy-PI slučka v Simulinku.** Odchýlka a zmena odchýlky vstupujú do FIS; jeho normalizované výstupy sa škálujú na časovo premenlivé proporcionálne a integračné zosilnenia.

<p align="center">
  <img width="1546" height="217" alt="Schéma Fuzzy-PI riadenia" src="https://github.com/user-attachments/assets/ea786de8-078d-4e58-b375-f1f0c505e5b9" />
</p>

`Fuzzy_PI.fis` definuje Mamdaniho FIS s implikáciou `min`, agregáciou `max` a defuzifikáciou centroidom. Skript ho načíta funkciou `readfis` a nastaví škály:

```matlab
Ge  = 1/Ref;
Gde = 0.8;
GKp = 130;
GKi = 20;
```

Dva vstupy sú normalizovaná odchýlka a zmena odchýlky. Výstup FIS sa pomocou `GKp` a `GKi` prevedie späť na časovo premenlivé PI zosilnenia.

[Späť na obsah](#obsah)

<a id="implementacia"></a>

## 🛠️ Implementácia repozitára

Hlavný postup je:

```text
Parametre motora --> G(s), A/B/C/D --> návrh regulátora --> model Simulink
                                                               |
Referenčný uhol (Deg --> Ref v rad) --------------------------+
                                                               |
                                                        Scope / grafy
```

1. `Zadanie_1.m` vyčistí pracovný priestor, deklaruje konštanty motora a prevedie požadovaný uhol zo stupňov na radiány.
2. Vytvorí prenosovú funkciu `G` aj stavový model `A,B,C,D`.
3. Vypočíta zosilnenia PID pomocou `pidtune`, zosilnenie LQR pomocou `lqr` a načíta Fuzzy-PI FIS, ak je dostupný.
4. `Zadanie_1_sim.slx` použije tieto premenné pracovného priestoru v blokoch regulátorov.
5. Scope bloky zobrazia referenciu a polohu motora.

Výsledky simulácie sa zobrazujú priamo v Scope blokoch Simulinku.

[Späť na obsah](#obsah)

<a id="parametre-ladenie"></a>

## ⚙️ Parametre a logika ladenia

<a id="tab-tuning-sk"></a>

**Tabuľka 3. Parametre a logika ladenia.**

| Regulátor | Parametre v tejto implementácii | Logika výberu | Poznámka k reprodukovateľnosti |
|---|---|---|---|
| PID | `type='PID'`, `wc=0.5 rad/s` | Automatické ladenie založené na modeli pomocou `pidtune(G,'PID',wc)`. | Spustite `Zadanie_1.m`; pozrite si vypísané `Kp`, `Ki`, `Kd`. |
| LQR | `Q=1e6*eye(3)`, `R=10` | `Q` penalizuje odchýlku stavu; `R` penalizuje riadiace úsilie. `lqr` rieši spojitý problém LQR. | Skript prepočíta `K_lqr`. |
| Fuzzy-PI | `Ge=1/Ref`, `Gde=0.8`, `GKp=130`, `GKi=20`; 49 pravidiel | Normalizácia vstupov, inferencia normalizovaných zosilnení a následné škálovanie. | Súbor `.fis` musí byť v aktuálnom priečinku MATLABu. |

Pri porovnávaní regulátorov používajte rovnakú referenciu a konfiguráciu simulácie. Sledujte:

- **Čas nábehu**: ako rýchlo sa odozva prvýkrát priblíži k cieľu;
- **Prekmit**: o koľko odozva prekročí cieľ;
- **Ustálenie**: ako rýchlo zostane odozva v blízkosti cieľa;
- **Riadiace úsilie**: či by požadované napätie alebo riadiaci signál bol praktický pre skutočný aktuátor.

[Späť na obsah](#obsah)

<a id="vysledky"></a>

## 📈 Výsledky a interpretácia

Pre predvolenú simuláciu platí `Deg = 150`, teda $150\pi/180 \approx 2.618$ rad. Spustite model Simulink a porovnajte priebehy v Scope blokoch pri rovnakej referencii.

<a id="fig-results-sk"></a>

**Obrázok 4. Porovnanie simulovaných odoziev polohy antény.** Žltá krivka je referenčný uhol; odozvy regulátorov ukazujú, ako rýchlo a plynulo poloha antény sleduje túto hodnotu.

<p align="center">
  <img width="2238" height="1170" alt="Simulované odozvy polohy antény" src="https://github.com/user-attachments/assets/2fff9ea7-b040-4d7c-9ff3-ae9a578ca18f" />
</p>

<a id="tab-results-sk"></a>

**Tabuľka 4. Výsledky a interpretácia.**

| Regulátor | Čo vyhodnotiť v simulácii | Interpretácia |
|---|---|---|
| PID | Čas nábehu, prekmit a oscilácie. | Rýchlejšia slučka nie je automaticky lepšia; záleží na prechodových požiadavkách. |
| LQR | Sledovanie referencie pri nakonfigurovaných hodnotách `Q` a `R`. | Kvadratické hmotnosti určujú kompromis medzi odchýlkou stavu a riadiacim úsilím. |
| Fuzzy-PI | Sledovanie s bázou pravidiel FIS a nakonfigurovanými škálovacími faktormi. | Adaptívne zosilnenia menia prechodový dej; výkon závisí od bázy pravidiel a škálovania. |

Konečné porovnanie robte z priebehov vytvorených vlastným spustením modelu. PID, LQR a Fuzzy-PI nemajú univerzálne poradie: výsledok závisí od rastliny, nastavení regulátora, referencie a kritéria výkonu. Zdrojový článok uvádza vlastné experimentálne porovnanie pri vlastných nastaveniach. [1]

[Späť na obsah](#obsah)

<a id="zaciatok"></a>

## ▶️ Začíname

### Požiadavky

- MATLAB s **Control System Toolbox** (`tf`, `ss`, `pidtune`, `lqr`);
- Simulink;
- **Fuzzy Logic Toolbox** na načítanie a simuláciu `Fuzzy_PI.fis` / vetvy fuzzy regulátora.

### Spustenie štúdie

1. Naklonujte alebo stiahnite tento repozitár.
2. V MATLABe nastavte Current Folder na koreňový priečinok repozitára, ktorý obsahuje `Zadanie_1.m`.
3. Spustite:

   ```matlab
   Zadanie_1
   ```

4. Otvorte `Zadanie_1_sim.slx`.
5. Spustite simuláciu a sledujte Scope bloky.
6. Ak chcete otestovať iný cieľový uhol, upravte iba:

   ```matlab
   Deg = 150;
   ```

   Skript automaticky nastaví `Ref = Deg*pi/180`, aby model dostal hodnotu v radiánoch.

### Dôležitá poznámka k PID Tuner

Tlačidlo **Tune...** v Simulinku môže vyžadovať licenciu Simulink Control Design. Tento repozitár nezávisí od tejto interaktívnej aplikácie: používa príkaz `pidtune` pre LTI model. Ak `pidtune` nie je v lokálnej inštalácii dostupný, vetvu PID nemožno automaticky nanovo vygenerovať; použite inštaláciu s Control System Toolbox.

[Späť na obsah](#obsah)

<a id="struktura"></a>

## 📂 Štruktúra repozitára

```text
Project root/
├── README.md              # Anglická dokumentácia
├── README_SK.md           # Slovenská dokumentácia
├── Zadanie_1.m            # Parametre, rastlina, návrh regulátorov
├── Zadanie_1_sim.slx      # Model Simulink
└── Fuzzy_PI.fis           # Mamdaniho Fuzzy-PI inferenčný systém
```

<a id="obmedzenia"></a>

## ⚠️ Obmedzenia

- Ide o simulačnú štúdiu, nie o regulátor v reálnom čase na hardvéri.
- Model rastliny je lineárny a neobsahuje saturáciu aktuátora, šum snímača, vôľu prevodov, neurčitosť parametrov ani poruchy záťaže, pokiaľ ich používateľ nepridá.
- Úplná stavová spätná väzba LQR predpokladá, že potrebné stavy sú merané alebo odhadnuté.
- Porovnanie regulátorov závisí od konfigurácie; pri vyvodzovaní záverov meňte vždy iba jednu premennú.

<a id="literatura"></a>

## 📖 Literatúra

1. M. A. Fkirin a M. A.-E. Khira, „Enhanced Antenna Positioning Control System Using Adapted DC Servo Motor and Fuzzy-PI Controller,“ *IEEE Access*, roč. 11, s. 102661–102668, 2023. [DOI: 10.1109/ACCESS.2023.3313976](https://doi.org/10.1109/ACCESS.2023.3313976).
2. MathWorks, „[Simulink Documentation](https://www.mathworks.com/help/simulink/).“
3. MathWorks, „[PID Controller Tuning](https://www.mathworks.com/help/control/pid-controller-design.html).“
4. MathWorks, „[`pid` – PID controller in parallel form](https://www.mathworks.com/help/control/ref/pid.html).“
5. MathWorks, „[`lqr` – Linear-Quadratic Regulator design](https://www.mathworks.com/help/control/ref/lti.lqr.html).“
6. MathWorks, „[Fuzzy Logic Toolbox Documentation](https://www.mathworks.com/help/fuzzy/).“
7. MathWorks, „[`pidtune` – PID tuning algorithm for linear plant model](https://www.mathworks.com/help/control/ref/dynamicsystem.pidtune.html).“

<a id="autorka"></a>

## 👩‍🎓 Autorka

**Ievgeniia Ievgrafova**

Projekt z predmetu RMS

Fakulta elektrotechniky a informatiky

Slovenská technická univerzita v Bratislave
