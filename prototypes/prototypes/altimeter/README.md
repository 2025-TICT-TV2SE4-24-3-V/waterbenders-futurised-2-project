# Belangrijk!

De altimeter sensor hebben we uiteindelijk **niet** geïmplementeerd in het eindproduct. De reden hiervoor is dat in de simulatie flip (de robot), niet navigeert over verschillende hoogtes. Hierdoor zal de altimeter altijd de waarde 0 teruggeven. Wel hebben wij de altimeter geïmplementeerd in een sdf file en een subsciber cc code. Mogelijk in een verdere uitbreiding met verschillende hoogtes kan de altimeter sensor wel worden geïmplementeerd.

### Testen via de Terminal
Om te testen of de altimeter correct waardes meet stuur ik het blokje met de altimeter handmatig aan via de terminal.

Omhoog:
```
gz topic -t /model/cube_with_thruster/joint/thruster_joint/cmd_thrust \
  -m gz.msgs.Double \
  -p "data: 3.0"
```

Stoppen:
```
gz topic -t /model/cube_with_thruster/joint/thruster_joint/cmd_thrust \
  -m gz.msgs.Double \
  -p "data: 0.0"
```