# strawberry_shortcake_ice_cream_bar model (scene_6)

`scene_6` (`custom_object_sorting`, wired via `src/app/launch/custom_object_sorting.launch.py`)
expects a TensorRT engine here:

```
src/example/example/yolo_detect/models/strawberry_shortcake_ice_cream_bar.engine
```

(matching the layout the other models in this directory use — `v11/` and `26/`
subfolders are only used when the model name contains `"11"` or `"26"`,
see `example.yolo_detect.yolo_node.YoloNode.__init__`).

Train/export your model the same way the existing `best_traffic`/
`best_garbage`/`yolov11n` models were produced for this workspace. Label
the object as class **"strawberry shortcake ice cream bar"** — that exact
string is what `custom_object_sorting.launch.py`'s `classes` parameter and
`src/app/config/plays/scene6_custom_object_sorting.yaml`'s `place_targets`
key both expect. Then drop the compiled `.engine` file here.

Until a real model is in place, `scene_6` will start but the detector will
log a "model file not found" error and never publish detections.
