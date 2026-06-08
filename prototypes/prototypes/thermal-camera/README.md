# Thermal Camera Test

## How To
De thermal camera kan met behulp van **OpenCV** een apart venster openen wanneer er een Python script is die de waardes uitleest. Ook in Gazebo zelf kan je de warmte van verschillende objecten zien op de *'Grey Scale'*. De warmte wordt niet met kleur weergegeven maar met de Grey Scale. Bedoeling is om dit in de toekomst nog mogelijk aan te kunnen passen zodat er met *kleuren* duidelijker wordt weergegeven wat de temperaturen zijn van de objecten.

Het is zowel met C++ als Python getest om de input in de terminal terug te zien als Subscriber, maar echter is dit alleen succesvol gebleken met de **Python file**. Voor nu, negeer de `build` folder, hier zit de CMake en .cc file in.

### Runnen:
1. Eerst in de container komen (met opgestelde .ps1 file)
```bash
.\rungazebo.ps1
```
2. Navigeer naar thermal_camera folder:
```bash
cd prototypes/thermal-camera
```
3. Run .sdf file:
```bash
gz sim test.sdf
```
4. Druk op 'Start'in Gazebo wereld
5. Run .py file:
```bash
python3 Tcamera.py
```

## Omgeving.sdf
Tot nmu toe is het nog niet gelukt om de eerder gemaakt omgeving werkend te krijgen. De Python file werkt goed, maar `omgevingTC.sdf` werkt nog niet. 

## Limitations Thermal Camera in GZ
- Since this thermal camera has a linear resolution of .01 (10mK), and the maximum value that can be stored in an unsigned 16-bit integer is 65535, this means that the maximum temperature that can be recorded by this camera is (65535 * .01) = 655.35 Kelvin / 680.72 F / **360.4 C.**
- If one object with a given temperature blocks another object with a given temperature (with respect to the thermal camera's point of view), the blocked object **will not be displayed** at all, **regardless** of each object's thickness and temperature difference between them (see the image below for an example). A more realistic implementation would have the camera display temperature fluctuations in the closer object if the temperature difference between the two objects is large enough, and if the closer object isn't too thick.
- There's a precision loss when the thermal camera converts temperature readings to gray scale output. To help quantify the magnitude of this precision loss, running the conversion code above on this SDF file results in the following processed temperatures for each object in the camera's image:
    - **sphere:** listed in the SDF file as 600 Kelvin, processed from the thermal camera image topic as **598.81 Kelvin**
    - **box:** listed in the SDF file as 200 Kelvin, processed from the thermal camera image topic as **200.46 Kelvin**
    - **cylinder:** listed in the SDF file as 400 Kelvin, processed from the thermal camera image topic as **400.92 Kelvin**

