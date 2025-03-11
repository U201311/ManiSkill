# 3d Asset 

# step1: import 3d asset(*.glb)

```bash
mkdir asset
cd asset
import *.glb

```

# step2: create pick task:

```bash
cd mani_skill/examples/motionplanning/solutions

# create task  for example: pick_banana()
touch pick_banana.py 

```
# step3: crate build actor func

```bash
cd mani_skill/utils/building/actors/common.py

# create task  for example: pick_banana()
def build_glb_obj(
  builder = scene.create_actor_builder()
    builder.set_mass_and_inertia(
        mass=0.1,
        cmass_local_pose=sapien.Pose([0,0,0]),
        inertia=[0,0,0], 
    )
    if add_collision:
        builder.add_nonconvex_collision_from_file(filename=glb_path, scale=(half_size, half_size, half_size),desity=0.01)
    builder.add_visual_from_file(filename=glb_path,scale=(half_size, half_size, half_size),#q=[0.5, 0.5, 0.5, 0.5]
                                 pose=sapien.Pose(p=[0, 0, 0], q=[0.5, 0.5, 0.5, 0.5]))
    return _build_by_type(builder, name, body_type, scene_idxs, initial_pose)
)

```


# step4 visuliza

```bash

python -m mani_skill.examples.teleoperation.interactive_panda -e "PickBanana-v1" 


python -m mani_skill.examples.motionplanning.panda.run -e "PickBanana-v1" --vis # opens up the GUI

```