## Wat is het Dijkstra algoritme
Bij het Dijkstra algoritme (kortse pad algoritme) wordt steeds de kortste route berekend. Het algoritme koppelt aan een startpunt een afstand van 0 toe en alle andere punten hebben een afstand van oneindig. Voor het verkennen begint je bij het startpunt en wordt er gekeken naar alle directe buren. De afstand tot deze buren wordt berekend en opgeslagen. Het algoritme kiest vervolgens het niet bezochte punt dat het dichts bij het startpunt ligt. Vanuit dat punt worden de afstanden naar de omliggende punten opnieuw berekend. Als er een kortere route naar een bepaald punt wordt gevonden, wordt de oude (langere) afstand overschreden met de nieuwe. Dit proces wordt steeds herhaald totdat alle punten zijn bezocht en de kortste route naar elk punt in het netwerk bekend is.

Voor de pathfinding hebben wij eerst gebruik gemaakt van het Breath first search algoritme. Om de pathfinding te verbeteren hebben wij Dijkstra later geïmplementeerd.

## Dijkstra in code
Voor de code is gebruik gemaakt van https://www.redblobgames.com/pathfinding/a-star/introduction.html als bron.

Voor de initialisatie maken we voor "oneinigen afstanden" gebruik van *Dicts*, alles wat we nog niet kennen heeft een oneindige afstand. 

Bij **frontier = Queue()** wordt er een wachtrij aan gemaakt. Dit is waar we alle punten in stoppen die we nog moeten bezoeken. De wachtrij zorgt er automatisch voor dat het punt met de kortste afstand altijd vooraan staat.

Bij **frontier.put((0.0, start))** voegen we een startpunt toe aan de wachtrij en koppelen hier direct een afstand van 0.0 aan oftewel (afstand, punt).

Ook maken we twee legen dicts aan: **came_from = dict()** en **cost_so_far = dict()** *came_from* onhoudt wie de voortganger van een bepaalde punt was. *cost_so_far* houdt de kortste bekende afstand vanaf het startpunt bij.

Bij **came_from[start] = None** en **cost_so_far[start] = 0.0** registeren we de startwaardes in de dicts. Het start punt heeft geen voortganger (none) en de afstand tot zichzelf is 0.0. Alle andere punten in de matrix die *niet* in *cost_so_far* staan hebben op dit moment theoretisch een afstand van oneindig.

**current = start** hier bepalen we dat we nu gaan starten bij het startpunt.

## Hoofdloop

**while not frontier.empty()** zorgt ervoor dat zolang er punten in de wachtrij(frontier) zitten die we nog moeten bekijken, de code blijft loopen.

Bij **if not edge_cells: break** als alle onbekende grenzen op de map al zijn verwerkt hoeft het algoritme niet de hele rest van de wereld te berekenen, we stoppen dan al eerder

Bij **current_cost, current = frontier.get()** kiest het algoritme het niet bezochte punt wat het dichts bij het startpunt ligt. De wachtrij geeft ons met .get() direct het punt met de allerlaagste current_cost die op dat moment bekend is. Dit punt wordt nu ons huidge standpunt (current).

**if current in edge_cells: edge_cells.remove(current)** als het gekozen dichtstbijzijnde punt een van edge cells is verwijderderen we hem uit de lijst met edge cells

Bij **neighbours = find_neighbouring_cells(grid, current[0], current[1], diagonals=True)** krijg je een lijst terug van alle 8 cellen die direct rondom het huidge punt liggen.

Bij **for ni, step_cost in neighbours:** loopen we door alle neigboors heen. **ni** is de coordinaat van de neighbor en **step_cost** is de afstand om er te komen (1.0) voor rechts en (1.414) voor diagnonaal. Daarom is de logica van de functie **find_neighbouring_cells** aangepast om ook diagnonale bewegingen te maken.

Bij **if grid[ni] == 100: continue** negeren we de neighboor als het een obstakel is (100).

Bij **new_cost = cost_so_far[current] + step_cost** nemen we de afstand die we al hadden afgelegd om tot het huidge punt te komen (cost_so_far[current]) en tellen daar de stap naar de buur (step_cost) bij op. Dit wordt mogelijk nieuwe route

Bij **if ni not in cost_so_far or new_cost < cost_so_far[ni]:** controleren we of we deze neigboor nog nooit eerder hebben bezocht (not in cost_so_far) én of onze zojuist berekende *new_cost* korter is dan de route die we al eerder voor deze neighboor hadden opgeslagen

Bij **cost_so_far[ni] = new_cost** als de route korter is, wordt de oude afstand overschereven met de nieuwe kortere afstand(new_cost)

**priority = new_cost** en **frontier.put((priority, ni))** omdat we een kortere route naar dit punt hebben gevonden, zetten we dit punt opnieuw in de wachtrij. Het algoritme zal dit punt hierdoor sneller gaan verkennen.

**came_from[ni] = current** om het snelst bij neighboor *ni* te komen moet je vanaf het punt *current* loopen. Dit is belangrijk om straks de route terug te vinden.

## Route teruglezen

We maken een lege lijst aan met **path = []**

**if current not in came_from and current != start: return [start]** als het algoritme gestopt is maar er nooit een geldige route naar een doel is gevonden dan geven we het startpunt terug aan flip.

**while current != start and current is not None: path.append(current) current = came_from.get(current)** we loopen achteruit aan de hand van **came_from**. We voegen het huidige punt toe aan de route en kijken via **came_from.get(current)** wie de voorganger was. Dit proces herhalen we totdat we weer bij het startpunt uitkomen.

Bij **path.append(start)** en **path.reverse()** voegen we het startpunt als laatste toe aan de lijst. Omdat we van achter naar voren hebben gewerkr staat de route nu in de verkeerde volgorde. Met **path.reverse()** draaien we de lijst om zodat de route klopt.

Tot slot ontvangen we de route met **return path**

## Veranderingen aan main.py
- from queue import PriorityQueue
- Update **find_neighbouring_cells** zodat de afstand wordt berekend
- Vervang **breadth_first_search** met de **Dijkstra** functie

**find_neighbouring_cells**:

```
def find_neighbouring_cells(self, array, iy, ix, diagonals=False):

        height, width = array.shape

        neighbours = []

        # All neighbouring cells (including diagonals)
        directions = [
            (0, 1),
            (0, -1), 
            (1, 0),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1) 
        ]

        for i, (x, y) in enumerate(directions):
            if diagonals == False and i > 3:
                break
            nx = ix + x
            ny = iy + y
            # Check whether each cell has a neighbour (isn't at the edge of the map)
            if 0 <= nx < width and 0 <= ny < height:
                index_pair = (ny, nx)
                # Calculate cost based on path movement type
                cost = 1.414 if i > 3 else 1.0
                neighbours.append((index_pair, cost))

        return neighbours

```

**Dijkstra:**

```
def dijkstra(self, grid, start: tuple):

        edge_cells = self.candidate_boundaries(grid)

        if not edge_cells:
            self.get_logger().warn("No frontier cells found")
            return [start]

        frontier = PriorityQueue()
        frontier.put((0.0, start))
        
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0.0
        current = start

        while not frontier.empty():
            if not edge_cells:
                break

            current_cost, current = frontier.get()

            if current in edge_cells:
                edge_cells.remove(current)

            # look for 8 way directional neighbors
            neighbours = self.find_neighbouring_cells(grid, current[0], current[1], diagonals=True)

            for ni, step_cost in neighbours:
                if grid[ni] == 100:  # Obstacle check
                    continue
                
                new_cost = cost_so_far[current] + step_cost
                if ni not in cost_so_far or new_cost < cost_so_far[ni]:
                    cost_so_far[ni] = new_cost
                    priority = new_cost
                    
                    frontier.put((priority, ni))
                    came_from[ni] = current

        path = []
        if current not in came_from and current != start:
            return [start]
            
        while current != start and current is not None:
            path.append(current)
            current = came_from.get(current)
        path.append(start)
        path.reverse()

        return path
```
