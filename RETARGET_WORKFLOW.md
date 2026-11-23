# Mixamo retarget workflow

The tool is [mixamo_retarget.py](mixamo_retarget.py). It copies a Mixamo pose onto another armature without baking that model's rest pose into the aim. A JSON map pairs target bones to `mixamorig:` bones. The correction is chosen from the Mixamo name, not the target name.

## Three corrections

Angles below are degrees from straight down. A T-pose arm is 90. An arm at 135 is 45 above horizontal.

**Track** (upper arm, forearm, hand, thigh, calf, foot). The bone points the same way Mixamo's does. The only change is a roll around the bone so a flipped rest X/Z does not twist the mesh. An A-pose rest is not added on top of a T-pose, so a 135 degree arm stays 135. Thighs, calves, and feet track for the same reason. Keeping the wider rest stance stacked that angle onto a run that already lifts the thigh, and the foot folded up into the chest.

**Offset** (hips, shoulders). The target keeps its own rest direction. Only Mixamo's change from his rest is applied. This is why Grapple Girl's shoulders stay on her rest angle instead of rising to his T-pose.

**Spine** (spine, neck, head). Same aim rule as track. The first spine bone is planted on the hip's upright aim, not on the hip bone's raw axis. Grapple Girl's hip points forward. Planting the waist on that axis pitched her chest over. The waist is disconnected and placed above the hip.

Roll, in armature space: the signed angle around local +Y that turns Mixamo's rest X toward the target's rest X. Track and spine then use `pose @ Ry(roll)`. Offset uses `pose @ rest_mixamo.inverted() @ rest_target`.

## Rigs that are not a single FK chain

Rigify DEF bones are parented to ORG bones, not to each other. Writing a rotation and leaving the head on the unposed ORG parent leaves twist segments at the rest pose, and the mesh stretches. The tool connects a bone to the mapped bone whose rest tail meets its rest head, then places the head from that posed bone. Fingers, palms, and toes are not in the map. They keep their rest curl and ride the hand or foot.

`use_connect` is cleared on deform bones. A connected bone ignores the head we write.

## Commands

One pose, with a report of roll, rest-direction difference, posed angle, and Mixamo's angle:

```bash
blender --background --python tools/mixamo_retarget.py -- \
  --target models/characters/grapple_girl.blend \
  --map tools/retarget_maps/grapple_girl.json \
  --action "Idle Inactive" --frame 0 \
  --report-only
```

`--rest` is his bind pose. `--out image.png` renders the posed mesh and a Mixamo stick figure.

Bake every frame of one or more clips into a glTF binary. The clip syntax is `blend|Action Name|ExportName`. Hip translation is not copied, so locomotion stays in place and the game moves the body.

```bash
blender --background --python tools/mixamo_retarget.py -- \
  --target models/characters/night_demon/demon-RIG.blend \
  --map tools/retarget_maps/night_demon.json \
  --export models/characters/night_demon_retarget.glb \
  --clip animations/movement_animations_1/movement_animations_1.blend\|Run-Forwards\|Run \
  --clip animations/combat_animations_1/combat_animations_1.blend\|Hit\ Reaction\ From\ Front\|HitFront
```

A new model needs `armature`, `mesh`, and `bones` in a JSON file. Maps in this project: [grapple_girl.json](retarget_maps/grapple_girl.json), [night_demon.json](retarget_maps/night_demon.json).

## What the numbers said

Grapple Girl, Idle Inactive frame 0, after the shoulder offset: arms 8.1 and 9.5, matching Mixamo. Shoulders 53.5 and 46.4, which is her rest plus his delta, not his 78.9 and 71.8. Waist 172.7, matching. Left thigh 24.6 against his 13.4.

Night Demon (the Shadow Demon; the file is `demon-RIG.blend`) has the same rules. There is no separate Shadow Demon asset. Almost every DEF bone's rest X/Z is flipped by about 180 degrees, and the upper arms are rolled about 88 degrees. Idle arms still match, 8.1 and 9.5. Shoulders stay near his rest (about 109) instead of dropping to Mixamo's. His shin rest aims about 33 degrees off Mixamo's. Tracking the leg, instead of adding that offset to every frame, keeps a run foot under the hip. Grapple Girl's clips were baked before this and were not exported again.

## Baked models

`models/characters/grapple_girl_retarget.glb`

- FightStance, from Idle Active
- Punch, from Zombie Smack Left, the left-hand strike. Her grappling hand is the Claw bone, mapped to LeftHand. The hand is farthest forward around frames 41 to 61 of 117.

`models/characters/night_demon_retarget.glb`

- Run, from Run-Forwards
- HitFront, from Hit Reaction From Front

## Encounter

`scenes/encounter/Encounter.tscn` is the main scene. It is gameplay, not an animation timeline.

- Shadow Demon is a body that tracks Grapple Girl and moves toward her with velocity while Run plays. At arrival distance he stops. The clip does not carry him forward.
- Grapple Girl finds the nearest node in the `threat` group and turns to face it every physics frame. The retarget glbs face glTF +Z. Godot's characters face −Z (`atan2(-x, -z)` in PlayerRotationComponent and EnemyRotationComponent), so each visual is yawed 180° and the same yaw formula is used.
- Inside punch range she plays Punch. A hitbox in front of her is monitored only during the strike window.
- Overlap with his hurtbox calls `take_hit()`. That is what plays HitFront and stops him. The punch animation cannot damage him by itself.
