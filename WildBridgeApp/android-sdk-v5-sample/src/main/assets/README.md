# WildBridge model assets

Place the following files here before building the app:

| File                         | Description                                   |
|------------------------------|-----------------------------------------------|
| `rhino_yolo26s.tflite`       | Edge inference model (int8 preferred, TFLite) |
| `rhino_yolo26s_labels.txt`   | One class name per line, no index prefixes    |

## How to generate these files

Generate the files in your model-training repository (the repo that contains
`rhino_yolo26s.pt`) and copy the outputs here:

```bash
# Example (run in model repo):
python - <<'PY'
from ultralytics import YOLO
m = YOLO("models/rhino/rhino_yolo26s.pt")
m.export(format="tflite", imgsz=1280, int8=True, data="your_dataset.yaml", simplify=True)
print(m.names)
PY

# Copy outputs into this app repo:
cp wildbridge_model/rhino_yolo26s.tflite       src/main/assets/
cp wildbridge_model/rhino_yolo26s_labels.txt   src/main/assets/
```

The `RhinoYoloDetector` class will print a warning at run-time if either file
is missing and will gracefully return empty detections.
