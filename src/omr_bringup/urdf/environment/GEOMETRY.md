# Cabinet geometry contract

This file records the geometry that may be used by simulation and later door-opening planning.
Unknown transforms are deliberately not guessed.

## Frames and conventions

The active Gazebo model is `cabinet.sdf`. Its model frame uses metres and is defined as:

- `+Z`: upward;
- `+X`: the operator's right when facing the cabinet;
- `-Y`: outward, toward the operator;
- `cabinet_door_joint = 0 rad`: door closed;
- positive `cabinet_door_joint`: door opens outward;
- provisional joint range: `0 .. pi/2 rad`.

The right-side door hinge is at `(0.600, -0.98425, 0.122) m` in the cabinet model frame. The
door-link origin is this lower hinge-axis point; the separate `pivot_shaft.STL` is a handle/lock
assembly datum and is not the door hinge. The hinge axis is model `+Z`.

The source meshes use millimetres and CAD `+Y` as height. SDF mesh visuals apply scale
`0.001 0.001 0.001` and a `+pi/2` rotation about X. Launch pose arguments are expressed in
the already-normalized Gazebo frame.

## Supplied coordinate conversion

The supplied assembly coordinates use `x` for front/back thickness, `y` for left/right, and `z`
for up/down. Millimetre assembly coordinates convert to the active SDF model frame as:

```text
Gazebo X = 0.300 + assembly y / 1000
Gazebo Y = -0.5025 - assembly x / 1000
Gazebo Z = 1.125 + assembly z / 1000
```

The resulting model-frame centres are:

| Part | Model-frame centre `(x, y, z)` m |
|---|---|
| Door | `(0.30000, -0.99625, 1.16450)` |
| Handle | `(0.05150, -1.00175, 1.03865)` |
| Button | `(0.05150, -1.00675, 0.97650)` |
| Handle/lock pivot | `(0.05150, -0.97975, 1.07250)` |

The pivot-to-handle distance is approximately `40.4 mm`, but this does not establish the
door-hinge-to-handle lever arm. The door hinge remains the right edge of the door at model
`X=0.600 m`.

## Planning transforms

| Transform | Status | Source / next action |
|---|---|---|
| cabinet model -> door hinge | Known for simulation | Supplied door placement and right-hinge requirement |
| door hinge -> door link | Known | Door link origin is the hinge axis point |
| door hinge -> handle grasp frame | Provisional in simulation | Replace with physical measurement before robot use |
| handle grasp frame -> approach frame | Unknown | Choose and validate gripper approach direction and clearance |
| `arm_link6` -> gripper TCP | Unknown here | Obtain from the gripper URDF or hand-eye/TCP calibration |
| robot base -> cabinet model | Runtime value | Cabinet spawn pose plus robot/base/rail transforms |

## Converted simulation handle and button placement

The active articulated SDF keeps the door, handle, and button on `cabinet_door`. Their collision
centres in the door-link frame are:

```text
door link -> handle centre = (-0.54850, -0.01750, 0.91665) m
door link -> button centre = (-0.54850, -0.02250, 0.85450) m
```

Adding the door-link pose produces the global centres in the table above. The handle visual uses
the supplied `handle.STL`; a vertical cylinder (`radius=10.5 mm`, `length=92.5 mm`) provides a
stable contact approximation. The button has its own converted centre and small box collision;
it is no longer artificially centred on the handle. Both parts follow the hinge joint.

These converted coordinates still require visual and contact validation in Gazebo and physical
measurement before real-robot use. In particular, the gripper grasp frame, approach direction,
and TCP transform are not defined by a mesh centre.

With the default launch pose `(x,y,yaw)=(1.60,-0.30,-pi/2)`, the corrected hinge is approximately
at world `(0.61575,-0.900) m`. The coordinate correction invalidates the previous numerical
clearance estimate, so the closed pose and the complete 0..90 degree sweep must be checked again
against the robot's full Gazebo collision geometry before grasp testing.

## Hinge physics test

The SDF loads Gazebo Fortress's `ApplyJointForce` system for `cabinet_door_joint`. It is passive
until a command arrives. From a terminal in the development container, apply a positive test
torque and observe `/cabinet/joint_states`:

```bash
ign topic -t /model/cabinet/joint/cabinet_door_joint/cmd_force \
  -m ignition.msgs.Double -p 'data: 1.0'
```

The door should move smoothly from its zero-angle stop toward the `pi/2` upper limit. Always clear
the persistent command after the test:

```bash
ign topic -t /model/cabinet/joint/cabinet_door_joint/cmd_force \
  -m ignition.msgs.Double -p 'data: 0.0'
```

A negative command tests return motion toward the zero-angle stop. These commands validate only
the simulated hinge, damping, friction, and limits; they are not authorized for real hardware.

## Provisional physical values

The following values exist only to make the Gazebo door physically simulated and must be tuned
against the real cabinet:

- door mass: `15 kg`;
- hinge damping: `0.5 N m s/rad`;
- hinge friction: `0.2 N m`;
- maximum open angle: `90 degrees`;
- shell mass and primitive collision dimensions.

Do not use these provisional values to size grasp force or certify collision clearance.
