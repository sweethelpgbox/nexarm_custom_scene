# strawberry_shortcake_ice_cream_bar model (scene_6)

`scene_6` (`custom_object_sorting`, wired via `src/app/launch/custom_object_sorting.launch.py`)
loads its model through `example.yolo_detect.yolo_node`.

**Important**: on the actual deployed robot this was built against, that
node's `__init__` loads an **OpenVINO model directory**, not a TensorRT
`.engine` file (the two differ from the copy of this file in this
repository — the deployed one has since diverged locally). It expects:

```
src/example/example/yolo_detect/models/strawberry_shortcake_ice_cream_bar_openvino_model/
```
containing the `.xml`/`.bin` IR pair (Ultralytics `format='openvino'`
export output), matching the `26/` and `v11/` model directories already
in this folder.

**A pre-existing bug you'll hit with any model name that doesn't contain
`"11"` or `"26"`**: the deployed `yolo_node.py`'s model-path selection is
```python
if '11' in self.model_name:
    model_path = f'.../models/v11/{self.model_name}_openvino_model/'
if '26' in self.model_name:
    model_path = f'.../models/26/{self.model_name}_openvino_model/'
self.model = YOLO(model_path, task=self.task)
```
with no `else` — for any other name, `model_path` is never assigned and
the load crashes (or, worse, silently ends up running whatever model an
*earlier*-declared `model` parameter value resolved to, if one leaked in
— see the launch-argument-collision note below). Add a generic branch
before deploying a new class:
```python
if '11' in self.model_name:
    model_path = f'.../models/v11/{self.model_name}_openvino_model/'
elif '26' in self.model_name:
    model_path = f'.../models/26/{self.model_name}_openvino_model/'
else:
    model_path = f'.../models/{self.model_name}_openvino_model/'
self.model = YOLO(model_path, task=self.task)
```

**Export from your trained `best.pt`** (on the Jetson, matching the
other models here):
```bash
yolo export model=best.pt format=openvino imgsz=640
mv best_openvino_model strawberry_shortcake_ice_cream_bar_openvino_model
```
Then move that directory into `models/` here.

Label the object as class **"strawberry shortcake ice cream bar"** —
that exact string is what `custom_object_sorting.launch.py`'s `classes`
parameter and `src/app/config/plays/scene6_custom_object_sorting.yaml`'s
`place_targets` key both expect.

**Also watch for launch-argument collisions** if you add more scenes
this way: `custom_object_sorting.launch.py`'s own launch arguments are
namespaced (`scene6_*`) specifically because ROS 2 launch arguments are
global to the whole launch tree by name, and `waste_classification.launch.py`
(an always-on baseline include) declares plain `model_name`/`conf`/
`model_size`/`camera_topic` first — reusing those names anywhere else
in the tree silently inherits its defaults instead of your own.
