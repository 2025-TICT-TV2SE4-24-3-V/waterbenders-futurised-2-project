# Ontwikkeldocument Project Futurised-2

**Versie:** 1.1 \
**Team:** WaterBenders

## Inhoudsopgave

- [Ontwikkeldocument Project Futurised-2](#ontwikkeldocument-project-futurised-2)
  - [Inhoudsopgave](#inhoudsopgave)
  - [Inleiding](#inleiding)
  - [Leeswijzer](#leeswijzer)
  - [Uitgangspunten](#uitgangspunten)
    - [Systeem Context](#systeem-context)
    - [Minimum Viable Product (MVP)](#minimum-viable-product-mvp)
    - [Identificatie en prioritering van Key Drivers](#identificatie-en-prioritering-van-key-drivers)
  - [Requirements](#requirements)
    - [Functionele Requirements](#functionele-requirements)
    - [Niet-Functionele Requirements](#niet-functionele-requirements)
    - [Constraints](#constraints)
    - [Use Cases](#use-cases)
    - [Activity Diagrammen](#activity-diagrammen)
  - [Ontwerp](#ontwerp)
    - [Functionele decompositie / (sub)systems and interfaces](#functionele-decompositie--subsystems-and-interfaces)
    - [Objectmodellen](#objectmodellen)
      - [Lijst met Objecten](#lijst-met-objecten)
    - [Taakstructurering](#taakstructurering)
      - [Taaksoort en deadline](#taaksoort-en-deadline)
      - [Taken samenvoegen](#taken-samenvoegen)
    - [Klassediagrammen](#klassediagrammen)
    - [STD's](#stds)
  - [Realisatie](#realisatie)
    - [Fysieke View](#fysieke-view)
    - [Code](#code)
    - [Unit Tests](#unit-tests)
    - [Integratie Tests](#integratie-tests)
    - [Eindresultaat](#eindresultaat)
  - [Conclusie en Advies](#conclusie-en-advies)
  - [Appendices](#appendices)
    - [Appendix 1: Mindmaps](#appendix-1-mindmaps)
    - [Appendix 2: Gespreksverslagen](#appendix-2-gespreksverslagen)
      - [Notities bij Kickoff-Meeting](#notities-bij-kickoff-meeting)
    - [Appendix 3: Upgradeonderzoeksverslagen](#appendix-3-upgradeonderzoeksverslagen)
    - [Appendix 4: Referenties](#appendix-4-referenties)

## Inleiding

``Van wie komt de opdracht? Waar gaat de opdracht in hoofdlijnen over? Leg verder uit dat dit document bedoeld is om op heldere wijze overzicht en samenhang te geven voor het team tijdens het werken aan het project, en na afloop als overdrachts-document voor eventuele follow-ups.``

## Leeswijzer

``leg uit wat er in de hoogste-niveau-hoofdstukken wordt behandeld en hoe deze onderwerpen met elkaar in verband staan``

## Uitgangspunten

``Leg uit dat dit hoofdstuk de uitgangspunten voor de requirements inventariseert. Verwijs naar een appendix met genoteerde input (verslag van speech, gespreksverslagen) van de opdrachtgever - (de echte, tijdens de kickoff-meeting of diens vervanger (Marius,Bart) erna) ``

### Systeem Context

``Modelleer en beschrijf de Systeem Context – gebruik een systeem context diagram. Onderdelen van binnen je System of Interest horen er niet in thuis. Een decompositie van je systeem ook niet. Zijn er geen belangrijke actoren vergeten? Tip: maak eerst met je team een mindmap (zie appendix)``

### Minimum Viable Product (MVP)

#### Eisen

- De autonome besturing kan worden overgenomen door een persoon
- Herkennen van verschillende soorten objecten (bijv. vuur, obstakels, mensen, etc.)
- Autonoom rondrijden in een willekeurige ruimte
- Omgeving waarden uit sensoren verkrijgen
- Ruimte schetsen op een 2D kaart

#### Benodigdheden

Software:

- Gazebo
- OpenCV
- Python
- C++ compiler

Hardware sensors (op de robot):

- Camera
- Air Pressure sensors
- Altimeter
- Lidar
- IMU
- Thermal camera
- Wide angle camera?


### Identificatie en prioritering van Key Drivers

Stakeholders:
- Gebruiker 
- Opdrachtgever (product owner)
- Klant (Futurised)

| Key Driver          | Stakeholders         | Beschrijving                                                                                                                                                                                                                                |
| ------------------- | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Overdraagbaarheid   | Opdrachtgever        | Voor extra uitleg over het systeem is er documentatie in de vorm van comments in de code en diagrammen in het ontwikkel document. Daarnaast is de code overzichtelijk en goed leesbaar. Dit draagt bij aan de duurzaamheid van het product. |
| Onafhankelijkheid   | Opdrachtgever, klant | De robot kan uit zichzelf navigeren en bepaalde beslissing maken, waarbij geen hulp nodig is van een extra persoon.                                                                                                                         |
| Gebruiksvriendelijk | Gebruiker            | Het systeem moet ook voor niet-technische mensen goed bruikbaar zijn.                                                                                                                                                                       |


## Requirements

``leg uit hoe de requirements opgesteld worden door de samenhang te verwoorden van de onderwerpen uit de sub-hoofdstukken.``

### Functionele Requirements

| F01 | Manouvreren |
| --- | --- |
| **Beschrijving** | De robot manouvreert om objecten heen. |
| **Rationele** | De robot kan hierdoor autonoom rijden in een omgeving, zonder zelf te kunnen manouveren valt het autonome weg. |
| **Prioriteit** | Must-have |

---
---

| F02 | Objecten Herkennen |
| --- | --- |
| **Beschrijving** | Als opdrachtgever wil ik dat de robot objecten kan herkennen in de simulatie.  |
| **Rationele** | Zodat hij die data kan gebruiken voor manouvreren, blussen en redden bijvoorbeeld. |
| **Prioriteit** | Must-have |

---
---

| F03 | Plattegrond Genereren |
| --- | --- |
| **Beschrijving** | Er wordt een plattegrond gegeneerd van de ruimte waar de robot zich in bevindt.  |
| **Rationele** | Hierdoor krijgt de bestuurder een beter inzicht van de omgeving en situatie. |
| **Prioriteit** | Must-have |

---
---

| F04 | Autonoom Rijden |
| --- | --- |
| **Beschrijving** | De robot kan zonder aangestuurd te worden, manouvreren om objecten heen.  |
| **Rationele** | Hierdoor hoeft de bestuurder niet constant handmatig te controleren op de robot. |
| **Prioriteit** | Must-have |

---
---

| F05 | Handmatig Besturen |
| --- | --- |
| **Beschrijving** | De robot kan ook handmatig bestuurd worden op afstand. |
| **Rationele** | Hierdoor kan er ingegrepen worden door de bestuurder wanneer dit nodig is. |
| **Prioriteit** | Must-have |

---
---

| F06              | Afstanden detecteren                                                                                                           |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| **Beschrijving** | De robot meet afstanden tot objecten.                                                                                          |
| **Rationele**    | Door afstanden te detecteren heeft de robot genoeg tijd om richting en snelheid aan te passen om om het object heen te rijden. |
| **Prioriteit**   | Must-have                                                                                                                      |

---
---

| F07              | Thermal Sensor                                                                            |
| ---------------- | ----------------------------------------------------------------------------------------- |
| **Beschrijving** | Als product ontwikkelaar wil ik dat de robot gebruik maakt van een thermal sensor.        |
| **Rationele**    | Hierdoor kan de robot de heat signature van een mens of een brandend object te herkennen. |
| **Prioriteit**   | Could-have                                                                                |

---
---

| F08              | Ultra-Wide Camera                                                             |
| ---------------- | ----------------------------------------------------------------------------- |
| **Beschrijving** | Als product ontwikkelaar wil ik dat de robot een ultra-wide camera gebruikt.  |
| **Rationele**    | Hierdoor heeft de robot een groot overzicht van zijn omgeving.                |
| **Prioriteit**   | Could-have                                                                    |

---
---

| F09              | Audio Communicatie                                                                                                 |
| ---------------- | ------------------------------------------------------------------------------------------------------------------ |
| **Beschrijving** | Als product ontwikkelaar wil ik dat de robot een microfoon en speaker heeft in de Gazebo simulatie.                |
| **Rationele**    | Hierdoor kan de robot geluid ontvangen en versturen, zodat communicatie met slachtoffers of operators mogelijk is. |
| **Prioriteit**   | Could-have                                                                                                         |

### Niet-Functionele Requirements

| NF01             | Maximale Afstand                                                                                                                                |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **Beschrijving** | De robot meet afstanden met een lidar sensor en kan een minimale afstand houden tot objecten en obstakels van Xcm wanneer de gebruiker dit wil. |
| **Rationele**    | Met een afstand van Xcm wordt botsing met objecten voorkomen.                                                                                   |


---
---

| NF02             | Bochten Maken                                                                                                                       |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| **Beschrijving** | De robot kan een bocht maken van ? graden per seconde met rupsbanden. -> kan om eigen as draaien <br>Feedback: (bart: graden per s) |
| **Rationele**    | Door bochten te kunnen maken kan De robot om objecten heen bewegen zonder er tegenaan te botsen of snelheid te verliezen.           |

---
---

| NF03             | Visuele/Auditieve Feedback                                                                 |
| ---------------- | ------------------------------------------------------------------------------------------ |
| **Beschrijving** | De robot geeft een signaal naar bestuurder zodra een obstakel binnen 20cm is.              |
| **Rationele**    | Door visuele/auditieve feedback te geven, kan een mens ingrijpen wanneer nodig of gewenst. |

---
---

| NF04             | Dimensies Bepalen                                                                                                    |
| ---------------- | -------------------------------------------------------------------------------------------------------------------- |
| **Beschrijving** | Als product ontwikkelaar wil ik dat de dimensies van objecten bepaald kunnen worden met behulp van een lidar sensor. |
| **Rationele**    | Hierdoor kan het formaat van objecten ingeschat worden.                                                              |

---
---

| NF05             | Thermal Sensor                                                                                         |
| ---------------- | ------------------------------------------------------------------------------------------------------ |
| **Beschrijving** | De thermal camera heeft een beeldhoek van X graden en een resolutie van X pixels.                      |
| **Rationele**    | Hierdoor kan de robot de heat signature van een mens of een brandend object te herkennen en weergeven. |

---
---

| NF06             | Ultra-Wide Camera                                              |
| ---------------- | -------------------------------------------------------------- |
| **Beschrijving** | De ultra-wide camera heeft een beeldhoek van X graden          |
| **Rationele**    | Hierdoor heeft de robot een groot overzicht van zijn omgeving. |

---
---
**VRAGEN OPDRACHTGEVER**

| NF07             | Snelheid Plattegrond Genereren                                       |
| ---------------- | -------------------------------------------------------------------- |
| **Beschrijving** | De plattegrond moet binnen Xs worden gegenereerd.                    |
| **Rationele**    | Door dit snel te generen krijgt de bestuurder sneller een overzicht. |

---
---
**VRAGEN OPDRACHTGEVER**

| NF08             | Hoeveelheid Objecten Genereren                                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| **Beschrijving** | De plattegrond moet minimaal X objecten kunnen weergeven binnen een straal van Xm. -> filteren met een 'boundary'-box, zelf in kunnen stellen |
| **Rationele**    | Hierdoor weet de bestuurder door hoeveel objecten de robot moet manouvreren.                                                                  |

---
---

| NF09             | Objecten Vormgeven                                                      |
| ---------------- | ----------------------------------------------------------------------- |
| **Beschrijving** | Er worden vormen gebruikt om objecten op de plattegrond aan te tonen.   |
| **Rationele**    | Hierdoor kan de bestuurder en robot zien wat voor object is getecteerd. |

---
---

| NF10 | Data Opslaan |
| --- | --- |
| **Beschrijving** | De data van een object wordt opgeslagen in een database als het is gedetecteerd. |
| **Rationele** | Hierdoor hoeft de data in een keer maar eenmaal te generen en kan de situatie voor de toekomst worden geanalyseerd. |

---
---

| NF11             | Vooruit en Achteruit Rijden                                                                              |
| ---------------- | -------------------------------------------------------------------------------------------------------- |
| **Beschrijving** | De robot kan zowel vooruit als achteruit rijden met een snelheid van Xkm/u door middel van een rupsband. |
| **Rationele**    | Op deze manier kan de robot meerdere richtingen op manouvreren.                                          |

---
---

| NF12             | Hitte Bestendig                                                                   |
| ---------------- | --------------------------------------------------------------------------------- |
| **Beschrijving** | De sensoren op de robot werken tot een temperatuur van maximaal X graden Celsius. |
| **Rationele**    | Hierdoor kan de robot goed functioneren in erg hete ruimtes.                      |

---
---

| NF13             | Algoritmisch Rijden                                                    |
| ---------------- | ---------------------------------------------------------------------- |
| **Beschrijving** | De robot rijdt autonoom door middel van een geschreven algoritme.      |
| **Rationele**    | Het algoritme zorgt ervoor dat de robot werkelijk autonoom kan rijden. |

---
---

### Constraints

``Beschrijf de relevante Constraints``

| C01              | Hittebestendidheid                                                                                     |     |
| ---------------- | ------------------------------------------------------------------------------------------------------ | --- |
| **Beschrijving** | De robot functioneerd niet bij een temperatuur van 125 graden.                                         |     |
| **Rationele**    | De sensoren op de robot hebben een limiet, als deze limiet overschreden wordt, functioneren deze niet. |     |



### Use Cases

`` Een of meerdere use case diagram(men) met bijbehorende use case beschrijvingen ``

| Naam | UC01 Autonoom Manouvreren |
| --- | --- |
| Actor | Brandweer |
| Samenvatting   | De robot detecteerd objecten en manouvreert hier omheen met behulp van een algoritme. |
| Preconditie    | De robot staat aan. |
| Scenario       | 1. Gebruiker zet autonome-modus aan.<br>2. Gebruiker krijgt bevestiging dat de robot in autonome-modus staat.<br>3. Robot begint met scannen.<br>4. Robot detecteert een of meerdere obstakels<br>5a. Als het pad voor de robot vrij is of hij tussen obstakels past, rijdt hij die richting op.<br>5b. Als het pad niet vrij is zoekt de robot naar een manier om het obstakel heen te rijden.<br>6. De robot rijdt tussen het obstakel door of om het obstakel heen. |
| Invariant      | De robot is contstant aan het scannen. |
| Postconditie   | - |
| Uitzonderingen | De gebruiker schakelt autonome-modus uit.|

---
---
---

| Naam | UC02 Objecten Detecteren |
| --- | --- |
| Actor | Brandweer |
| Samenvatting   | De robot detecteerd verschillende soorten objecten. |
| Preconditie    | De robot staat aan. |
| Scenario       | 1. De robot scant omgeving<br>2. De robot detecteerd een object.<br>3. De robot bepaald aan de hand van de sensoren wat voor object dit is.<br>3a. Aan de hand van de thermo-sensor bepaalt de robot of het een levend wezen is.<br>3b. Aan de hand van de lidar sensor bepaalt de robot het formaat van het object.<br>3c. Aan de hand van de ultra-wide camera kan de robot bepalen wat voor object het is.<br>4. Het gescande obstakel wordt opgeslagen. |
| Invariant      | De robot is contstant aan het scannen. |
| Postconditie   | De robot slaat het object op in een plattegrond (UseCase 3). |
| Uitzonderingen | De robot kan het object niet detecteren als een van de sensoren niet goed werkt. |

---
---
---

| Naam | UC03 Plattegrond Genereren |
| --- | --- |
| Actor | Brandweer |
| Samenvatting   | De robot detecteerd objecten en slaat deze op in een plattegrond |
| Preconditie    | De robot staat aan en er wordt data ontvangen. |
| Scenario       | 1. De robot detecteerd object(en)(UseCase 2).<br>2. Aan de hand van het object koppelt de robot de correcte vorm aan het object.<br>3. De robot bepaalt de positie van het object.<br>4. De robot slaat de locatie en soort object op in de plattegrond.<br>5. De bestuurder ziet de plattegrond op de 'pc'.|
| Invariant      | De robot is contstant aan het scannen. |
| Postconditie   | - |
| Uitzonderingen | De robot kan de data van het object niet verzenden of opslaan. |

---
---
---

| Naam | UC04 - Handmatige besturing |
| --- | --- |
| Actor | Brandweer |
| Samenvatting   | De robot kan handmatig worden bestuurd wanneer dit gewenst is vanuit de bestuurder. |
| Preconditie    | De robot staan aan. |
| Scenario       | 1. Gebruiker zet manuele modus aan. <br> 2. De besturing wordt overgedragen aan de brandweer en de autonome-modus wordt uitgezet. <br> 3. Gebruik krijgt bevestiging dat de robot in manuele modus staat.<br>4. Gebruiker bestuurd de robot |
| Invariant      | Ten alle tijden kan de robot weer terug worden gezet naar autonome-modus. |
| Postconditie   | - |
| Uitzonderingen | Wanneer de robot in gevaar verkeert en inactief is. |

---
---
---

| Naam | UC05 - Data Beheren/Inzien |
| --- | --- |
| Actor | Bart, Futurised |
| Samenvatting   | Ten alle tijden slaat de robot al zijn verkregen data op, op een veilige plek. |
| Preconditie    | De sensoren krijgen metingen binnen. |
| Scenario       | 1. Data uit een sensor wordt constant gemeten.<br> 2. Deze data wordt meteen opgeslagen op de computer.<br>3. De data wordt op de computer weergegeven. |
| Invariant      | De robot is constant aan het scannen |
| Postconditie   | De data wordt doorgestuurd naar de juiste applicatie die deze vervolgens verwerkt en gebruikt. |
| Uitzonderingen | Er is geen data opslag meer of de sensoren riskeren overvehitting. |

---
---
---

| Naam | UC06 - Audio Communicatie |
| --- | --- |
| Actor | Brandweer |
| Samenvatting   | De robot kan audio ontvangen en afspelen via een microfoon en speaker in de simulatie. |
| Preconditie    | De robot staat aan en de audio componenten zijn actief in Gazebo. |
| Scenario       | 1. De robot activeert de microfoon.<br> 2. Geluid wordt opgenomen vanuit de omgeving.<br> 3. De audio wordt doorgestuurd naar de operator.<br> 4. De operator spreekt een bericht in.<br> 5. De robot ontvangt dit bericht.<br> 6. De speaker speelt het bericht af in de omgeving van de robot. |
| Invariant      | De audioverbinding blijft actief zolang de robot aanstaat. |
| Postconditie   | De operator en omgeving kunnen met elkaar communiceren via de robot |
| Uitzonderingen | De microfoon of speaker werkt niet correct of er is geen verbinding. |

### Use-case diagram

![use-case diagram img](/docs/images/ontwikkel_document_UC_diagram.png)

### Activity Diagrammen

![activity jdiagram img](/docs/images/ontwikkel_document_AC_UC01.png) \
*UC01 - Autonoom Manouvreren*

![activity diagram img](/docs/images/ontwikkel_document_AC_UC02.png) \
*UC02 - Objecten Decteren*

![activity diagram img](/docs/images/ontwikkel_document_AC_UC03.png) \
*UC03 - Plattegrond Generegen*

![activity diagram img](/docs/images/ontwikkel_document_AC_UC04.png) \
*UC04 - Handmatige Besturing*

![activity diagram img](/docs/images/ontwikkel_document_AC_UC05.png) \
*UC05 - Data Opslaan*


`` Voor belangrijke, wat complexere usecases kan het extra verduidelijking opleveren om er SysML - activity diagrammen bij te maken. ``

## Ontwerp

``leg uit hoe het ontwerp volgt door het volgen van de stappen die in de sub-hoofdstukken worden behandeld en hoe deze onderwerpen met elkaar in verband staan``

### Functionele decompositie / (sub)systems and interfaces

`` Geef met een functionele decompositie van het systeem grafisch weer hoe de verschillende functies (uit de functionele requirements) van het systeem met elkaar samenhangen. Dat geeft goede handvaten voor de decompositie van software en hardware verder op de rit. ``

### Objectmodellen

`` Ontwerp, uitgaande van use case beschrijvingen en activity diagrammen, de delen van het objectmodel die dat kunnen waarmaken. Beperk de interactie met het web-subsysteem tot een enkel object (een web proxy object oid). Een gedetailleerde uitwerking van het web-subsysteem is in een apart ontwikkeldocument te vinden.``

#### Lijst met Objecten

`` Voeg elk object uit de objectmodellen toe in de "lijst met objecten" Let op dat de beschrijvingen niet de relatie tussen de objecten duiden, maar louter wat objecten "los bekeken" doen. Dus niet: InstelControl stuurt een signaal naar .. Maar Instelcontrol is de "dirigent" van de usecase "Instellen" (meteen link toevoegen naar die usecase).``

| Object Naam   | Stereotype | Beschrijving                                                    |
| ------------- | ---------- | --------------------------------------------------------------- |
| InstelControl | Control    | "Dirigent" van de use case "Instellen" (zie use case Instellen) |
| Display       | Boundary   | Stuurt display hardware aan.                                    |
| etc..         |            |                                                                 |

### Taakstructurering

``leg uit wat het doel is van taakstructurering en hoe de deelstappen (sub-hoofdstukken) samen dat doel reliseren``

#### Taaksoort en deadline

`` Maak een tabel die per object taaksoort, deadline, periode en prioriteit weergeeft. Belangrijk: Deadline is zo lang mogelijk waarbij het nog net geen irritatie oplevert. Deadline <= Periode, Prioriteit is omgekeerd evenredig met deadline ``

| Object Naam   | Taaksoort     | Periode | Deadline | Prioriteit |
| ------------- | ------------- | ------- | -------- | ---------- |
| InstelControl | Demand Driven |         | 30ms     | 1          |
| PlusKnop      | Periodiek     | 60ms    | 60ms     | 2          |
| etc..         |               |         |          |            |

#### Taken samenvoegen

`` Maak een tabel waarin je laat zien welke objecten een eigen taak hebben en van welke de taken worden samenvoegd in een enkele "Taak". Noem in het laatste geval het object (bijvoorbeeld een handler) dat eigenaar wordt van die Taak als eerste.``

| Taak Naam  | Object Naam                            | Taaksoort     | Periode | Deadline | Prioriteit |
| ---------- | -------------------------------------- | ------------- | ------- | -------- | ---------- |
| InstelTaak | InstelControl                          | Demand Driven |         | 30ms     | 1          |
| ButtonTaak | <u>ButtonHandler</u> PlusKnop, MinKnop | Periodiek     | 60ms    | 60ms     | 2          |
| etc..      |                                        |               |         |          |            |

### Klassediagrammen

`` Ontwerp, uitgaande van de objectmodellen de bijbehorende klassediagrammen. Vergroot eventueel in latere verbeteringsronden de herbruikbaarheid en het gebruiksgemak van de klassen door het toepassen van geschikte Design Patterns of templating. Voeg de klassen ook toe in de Requirements Traceability diagrammen zodat duidelijk is welke requirements de klasse adresseert``

### STD's

``Ontwerp voor elke Taak de STD van de bijbehorende klasse(n), indien van toepassing vanuit activity diagram of usecase beschrijving, protocol of anderszins. Belangrijk: alle toestanden moeten gerepresenteerd worden in het diagram. Code zonder toestanden en zonder directe invloed op de flow tussen de toestanden kunnen gerepresenteerd worden door calls naar helper-functies. Vergeet niet bovenaan een geschikte STD-interface toe te voegen``

## Realisatie

``leg uit waarin de realisatie zich onderscheidt van het ontwerp, en noem kort de rollen van de subhoofdstukken``

### Fysieke View

``Ontwerp de fysieke decompositie (een SysML Bdd). (de functionele decompositie biedt daarvoor meestal goede aanknopingspunten). Verduidelijk fysieke compositie-relaties middels "Constraints" (NB: dat zijn in dit geval gekwantificeerde ontwerpkeuzes die zowel uit de eerdere constraints als de niet-functionele requirements bestaan.``

### Code

``Ontwerp vanuit de STD's de bijbehorende code. Geef waar nodig additionele toelichting. Voeg per STD een link toe naar de betreffende code in de team repo. Voeg ook links naar de overige code toe``

### Unit Tests

``Voeg in dit hoofdstuk sub-hoofdstukken toe met Unit Tests, elk bestaande uit een Testplan, een link naar de testcode, een samenvatting van de bevindingen van de meest recente uitvoer van die test en een link naar een bestand met die meest recente test-output.``

### Integratie Tests

``Voeg in dit hoofdstuk sub-hoofdstukken toe met Integratie Tests, elk bestaande uit een Testplan, een link naar de testcode, een samenvatting van de bevindingen van de meest recente uitvoer van die test en een link naar een bestand met die meest recente test-output. Belangrijke integratie-tests zijn uiteraard ook de tests van het complete product. Hoe goed doet het product wat het moet doen?``

``Verder zou je in dit hoofdstuk Realisatie ook subhoofdstukken kunnen opnemen met : beslissingstabellen, waarin gemaakte realisatiekeuzes worden verantwoord, Failure Mode Effect Analyses, en andere realisatie gerelateerde overwegingen.``

### Eindresultaat

`` Nog eens op een rijtje de behaalde functionaliteiten en de performance. ``

## Conclusie en Advies

`` Reflecteer op in welke mate de aanvankelijk opgestelde requirements daadwerkelijk zijn behaald. Geef goed onderbouwde aanbevelingen t.a.v. mogelijke toekomstige doorontwikkeling.`` 

## Appendices

### Appendix 1: Mindmaps

`` Bijvoorbeeld voorafgaand aan het maken van het systeem context diagram is een mooi moment om met zijn allen een mindmap te maken die een samenhang duidt van alles wat je als team maar kunt bedenken in relatie tot het systeem. ``

### Appendix 2: Gespreksverslagen

#### Notities bij Kickoff-Meeting

``... etc``

### Appendix 3: Upgradeonderzoeksverslagen

``zet hier een lijst met links naar de upgradeonderzoeksverslagen. Licht het toe met wat tekst met de belangrijkste samenvatttingen ervan.``

### Appendix 4: Referenties

`` lijst 3rd party materiaal waar naar verwezen is. Boeken, site-links.``

`` Verder mogelijk nog appendices over: de geschiedenis van de ontwikkeling van het product in grote lijnen (inclusief inmiddels afgeschoten tussenproducten), over het opzetten van de ontwikkelomgevingen, over hoe te debuggen. Houdt het bij samenvattende dingen. Lange teksten (zoals testuitvoer) niet in de appendices maar via links naar bestanden opnemen.``
