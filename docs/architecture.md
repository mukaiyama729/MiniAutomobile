# Robot system architecture

This document separates the architecture implemented today from the larger
robotics target. The current slice is Linux/Jetson control of one Pico 2 motor
controller. ROS 2, Autoware, perception, localization, navigation workflows,
and additional sensors are extension points, not completed features.

## Current command and feedback paths

```text
CLI input adapter
  -> DriveControl inbound port
  -> DriveUseCase
  -> CommandGateway
       -> ResourceArbiter (mode, source, lease, validity, safety)
       -> device health check
       -> MotorPort outbound port
  -> PicoUsbMotor driver
  -> robot-protocol submodule
  -> USB CDC
  -> robot-firmware submodule
  -> Pico 2 -> ULN2003 -> motor

Pico motion reply -> PicoUsbMotor -> motion writer -> RobotStateStore
Pico device health -> DeviceManager -> CLI health view
RobotStateStore -> consistent StateSnapshot -> application readers
```

Emergency stop bypasses the ordinary command lock and arbiter:

```text
CLI input reader -> SafetySupervisor -> safety latch + SafeStop mode
                                      -> PicoUsbMotor urgent STOP
```

The input reader starts the emergency-stop path as soon as it reads `e`, even
if the main CLI loop is waiting for a status or motion reply. The application
latch is set before the driver waits for the device acknowledgement. The USB
worker checks the stop latch while waiting for replies, gives STOP priority
over queued ordinary jobs, and rejects status/heartbeat work while latched.

## Dependency boundaries

- `robot-domain` defines hardware- and middleware-independent motion, control,
  lease, resource, proposal, health, and safety types.
- `robot-application` defines inbound/outbound ports, `DriveUseCase`, the
  `CommandGateway`, `ResourceArbiter`, and `SafetySupervisor`. Application code
  does not depend on USB, ROS 2, or Pico implementation details.
- `robot-state` implements the application state ports. It stores motion,
  control ownership, and safety independently, with a versioned consistent
  snapshot and narrow writer handles.
- `robot-device-manager` registers devices and exposes their health through an
  application port.
- `crates/robot-adapter/drivers/pico-usb` implements the motor port and owns
  the serial worker. Other drivers and reusable transports belong under
  `crates/robot-adapter` when implemented.
- `crates/robot-adapter/protocol` and `crates/robot-adapter/firmware` are
  independent Git submodules. The Linux workspace excludes the embedded
  firmware workspace.
- `apps/robot-cli` is the current composition root: it creates concrete
  adapters, state storage, device management, use case, and the CLI.

The intended dependency direction is:

```text
robot-domain <- robot-application <- robot-state / robot-device-manager /
                                  <- robot-adapter <- robot-cli composition
```

Adapters implement ports defined inward. `robot-domain` must not depend on
application, adapter, protocol, firmware, or ROS 2 crates.

## Commands, observations, and events

The architecture distinguishes three input kinds:

- **Command:** a request to act. Current CLI motion keys become
  `CommandProposal`s for `ResourceId::DriveBase`.
- **Observation:** measured or reported state. Pico motion replies update the
  motion snapshot; device health remains in `DeviceManager`. Neither is a
  motion command.
- **Event:** a fact requiring a response. A CLI emergency-stop request latches
  safety; a future disconnect/fault monitor can use the same path.

An adapter may be bidirectional. `PicoUsbMotor` both sends motion requests and
receives state replies over one serial worker. The application port remains
about robot behavior, not the transport.

## Control ownership and arbitration

Only one control lease can own the current `DriveBase` resource. A proposal
contains its source, resource, command, issue time, validity deadline, and
lease ID. The arbiter checks:

- the safety latch and control mode;
- whether the source is accepted in that mode;
- whether the proposal and active lease are still valid;
- whether the proposal matches the active lease owner and ID.

Manual mode accepts CLI/teleoperation. Autonomous mode accepts Nav2, Autoware,
or mission sources, but those bridges are not implemented yet. The composition
root binds each `DriveUseCase` instance to one logical source and mode; a caller
cannot choose either on each request. This is conflict arbitration, not
authentication. A ROS 2 or network bridge must authenticate its caller and
assign its own source before exposing the port. The active lease in a state
snapshot is internal state, not an external authorization mechanism.

Lease duration is bounded to five seconds. The CLI uses a three-second lease
and renews it every 250 ms. Expiry causes the USB worker to send STOP before
processing the next queued job. An expired lease cannot be renewed with its
old ID; reacquiring control creates a new ID.

`DriveUseCase` creates proposals rather than touching the motor. `CommandGateway`
serializes ordinary operations, checks device health, and forwards an
authorized command through `MotorPort`. Future actuators should define useful
logical resources such as `DriveBase` or `RobotArm`, not necessarily one
resource per physical motor.

An active lease is never returned again by `acquire_control`, even to the
same logical source. A caller must present the latest `ControlLease` returned
by acquisition or renewal to renew, release, or issue commands. Renewal only
extends the lease; the new deadline is applied in both application state and
the device worker, and stale lease copies are rejected. After expiry, a new
owner is granted control only after the old owner has received a fresh STOP
acknowledgement.

## Shared state and single-writer rules

`RobotStateStore` keeps robot motion, control state, and safety state in
separate cells. Its snapshot version changes around each write so readers retry
if a write overlaps the snapshot. Writers receive narrow handles for only the
state they own.

The current snapshot contains only the motion direction, control mode/lease,
and safety latch/reason/generation. It is not a store for camera frames or
LiDAR point clouds. Future localization, perception, navigation, device, and
mission state can be added as independently owned state components when those
features exist.

Motion results are committed only if the safety state is still clear at the
generation observed before the command. Emergency-stop results are committed
only for the generation that issued them. Each stop event advances the safety
generation, including repeated stops while already latched. This prevents an
old command reply or a concurrent reset from erasing a newer stop event.

## Safety behavior and limits

The safety responsibilities are distinct:

- `ResourceArbiter` decides whether a source may control a resource.
- `SafetySupervisor` latches emergency stop, forces `SafeStop`, and requires
  healthy device status plus a fresh STOP acknowledgement reporting `Stopped`
  before an explicit reset.
- `PicoUsbMotor` prioritizes STOP, cancels ordinary serial exchanges when a
  stop arrives, enforces lease deadlines locally, and ceases background
  watchdog traffic after a lease ends.
- Pico firmware has a 1.5-second watchdog. Its current implementation refreshes
  the watchdog on every valid request, including STATUS and PING. The host
  driver therefore re-sends the active direction while a lease is valid and
  avoids polling once it has lost the lease and the last known motion was not
  stopped.

`?` sends a fresh STATUS only while the driver has a valid control lease. With
no active lease it returns the cached stopped state, or rejects the query if
the last known motion was not stopped. The application blocks status reads
while the safety latch is set. Safety reset does not rely on that cache: it
sends a new STOP and requires a matching stopped acknowledgement. Device health
is read from `DeviceManager`; it is not part of `StateSnapshot`. When the
worker is intentionally idle without a lease, health reflects its last observed
transport result and is not proof that the cable is still connected.

This is a best-effort software stop path. USB transport failure, a stalled
controller, motor-driver faults, and mechanical inertia can prevent or delay a
stop. The CLI `e` command is not a physical emergency stop. A deployed robot
needs an independent hardware stop circuit and validation of the full actuator
chain. Firmware, application, and hardware safeguards must remain independent
of ROS 2 or mission orchestration.

## Submodule and update policy

`robot-system` owns the Linux application, state, adapters, and runtime.
`robot-protocol` owns the shared Linux/Pico wire contract. `robot-firmware`
owns embedded implementations. The main repository pins exact submodule
commits; child `main` updates do not move those pins automatically.

Child pushes may send `repository_dispatch`. A scheduled check and manual
workflow can also discover updates. The main workflow advances the submodule
references, runs host compatibility checks and the Pico firmware cross-build,
then opens a reviewable pull request. Merging that pull request records the
new child commits in the main repository. Firmware changes are not auto-merged.

## Implemented versus future work

Implemented now:

- clean domain/application/adapter boundaries for directional motor control;
- command source, manual/autonomous mode, one DriveBase lease, arbitration,
  versioned state snapshot, and a latched safety stop;
- CLI composition and one Pico USB CDC motor adapter;
- separate protocol and firmware submodules.

Future work includes typed velocity/pose/trajectory values, independent state
domains for localization and perception, command/event/observation streaming,
mission orchestration and lifecycle cancellation, camera/LiDAR drivers,
ROS 2/Nav2/Autoware bridges, simulation, reconnection/discovery, and hardware
integration tests. Add each when its contract and an end-to-end use case are
defined; do not create empty crates that imply the feature is already usable.
