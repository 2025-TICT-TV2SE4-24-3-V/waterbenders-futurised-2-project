# Run YOLO human detection
These prompts assume you're using `vpip` see the venv setup file if you don't/can't use this for the other option.

1. Run `yolo-objectherkenning.py` with vpy from venv. See [venv setup file](../../../setup/containerVenv/ReadME.md)
- with topic `--topic /front/image`
```bash
cd /workspace/prototypes/cv2-yolo-detection
vpy cv2-yolo-detection.py --topic /front/image
```

- with the mask window
```bash
cd /workspace/prototypes/cv2-yolo-detection
vpy cv2-yolo-detection.py --topic /front/image --show-mask
```

- with a different YOLO model
```bash
cd /workspace/prototypes/cv2-yolo-detection
vpy cv2-yolo-detection.py --topic /front/image --model yolo11s.pt
```

2. Run `room-v2.sdf`
```bash
cd /workspace/prototypes/cv2-yolo-detection
gz sim room-v2.sdf
```