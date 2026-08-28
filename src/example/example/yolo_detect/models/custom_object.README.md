# custom_object model (scene_6 placeholder)

`scene_6` (`custom_object_sorting`, wired via `src/app/launch/custom_object_sorting.launch.py`)
expects a TensorRT engine here:

```
src/example/example/yolo_detect/models/custom_object.engine
```

(matching the layout the other models in this directory use — `v11/` and `26/`
subfolders are only used when the model name contains `"11"` or `"26"`,
see `example.yolo_detect.yolo_node.YoloNode.__init__`).

Train/export your model the same way the existing `best_traffic`/
`best_garbage`/`yolov11n` models were produced for this workspace, name the
class `custom_object` to match `src/app/config/plays/scene6_custom_object_sorting.yaml`
and the `classes` parameter in `custom_object_sorting.launch.py` (or update
both to your real class name), then drop the compiled `.engine` file here.

Until a real model is in place, `scene_6` will start but the detector will
log a "model file not found" error and never publish detections.
