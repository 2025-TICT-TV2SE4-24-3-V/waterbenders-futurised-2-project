## 1.) Running
1. Run `yolo-objectherkenning.py` with vpy from venv. See [venv setup file](../../../setup/containerVenv/ReadME.md)
```bash
cd /workspace/prototypes/YOLO-HumanDetection
vpy yolo-objectherkenning.py
```

2. Run `room-v2.sdf`
```bash
cd /workspace/prototypes/YOLO-HumanDetection
gz sim room-v2.sdf
```


## 2.) Problems
### Running python files without certain packages
- When testing our prototypes for object / human recognition and other components, we found it difficult to update pip packages without breaking the container. To solve this, we implemented a virtual environment (venv) so pip packages were isolated and could be installed / updated separately. See [venv guide](../../../setup/containerVenv/ReadME.md) for the setup. You do not have to setup aliases of course as suggested in the guide but it is handy. You could however choose to source the venv everytime you wish to run python files or need to install certain packages. 

- Further more we needed a specfic version of numpy for our project which is why we chose to install version `numpy-1.26.4` by running the command 
```bash 
vpip install "numpy<2" 
```

or

```bash
source /root/venv/bin/activate
pip install "numpy<2"
```