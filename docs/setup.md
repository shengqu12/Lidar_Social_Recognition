# Workstation Setup

How the development workstation (the Legion Pro) is set up to run the pipeline.
The Jetson edge nodes only run the driver, background removal, and rosbridge;
everything else — detection, tracking, encounter analysis, and any deep-learning
detection — runs here. This document focuses on the parts that have sharp edges,
because those are what cost time to rediscover.

## Overview

| Component | Version / setting |
|-----------|-------------------|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy (system install) |
| Python env | conda env `livox`, Python 3.10 |
| GPU | NVIDIA RTX 5070 (Blackwell, sm_120) |
| CUDA toolkit | 12.8 |
| PyTorch | 2.11 + cu128 |
| LiDAR driver | `livox_ros_driver2` v1.2.6 |
| Network | Tailscale `100.113.199.85` |

## ROS 2 Jazzy and the conda Python conflict

This is the setup detail most likely to waste an afternoon, so it comes first.

System ROS 2 Jazzy on Ubuntu 24.04 is built against the **system Python (3.12)**.
The project's conda env `livox` uses **Python 3.10** (for compatibility with the
detection/ML stack). These two cannot be mixed: **sourcing the system ROS 2 Jazzy
setup from inside the `livox` conda env breaks**, because ROS 2's Python modules
are compiled for 3.12 and will not import under 3.10.

The practical consequence is that work splits along this line. ROS 2 launches and
anything that links the system ROS 2 Python live outside the conda env; the
detection/ML code and the workstation-side pipeline tooling (which talk to the
Jetsons over rosbridge WebSocket via `roslibpy`, not via the native ROS 2 Python
client) run inside `livox`. Because the workstation reaches the Jetsons through
rosbridge rather than the native ROS 2 transport, the conda env does not need the
system ROS 2 Python at all — which is what makes the split workable.

If you hit import errors that mention Python 3.12 vs 3.10, or ROS 2 modules
failing to load inside conda, this is the cause. Do not try to force the two
together; keep them separate.

## GPU, CUDA, and PyTorch

The RTX 5070 is a **Blackwell** card (compute capability **sm_120**). Blackwell
is only supported by recent CUDA, so the stack is pinned accordingly:

- **CUDA toolkit 12.8** — earlier CUDA does not support sm_120.
- **PyTorch 2.11 + cu128** — a PyTorch build matching CUDA 12.8.

If you see errors about an unsupported GPU architecture, `sm_120` not being in
the list of compiled architectures, or a PyTorch build that "doesn't support your
GPU," it is a CUDA/PyTorch version that predates Blackwell support. The fix is to
match these versions, not to downgrade the driver.

## Livox driver

The MID-360 driver is **`livox_ros_driver2` v1.2.6**, built with:

```bash
./build.sh humble
```

Note the build target is `humble` even though the system ROS 2 is Jazzy — that is
the recorded working build flag, not a typo. (The MID-360S sensors planned for
expansion need a different launch file and `lidar_type`; see
[hardware_network.md](hardware_network.md).)

## OpenPCDet patch (deep-learning detection component)

The live pipeline uses geometric clustering for detection, but the repository
also carries a deep-learning detection path under `third_party/` whose
predictions are consumed by the visualisation stage. That path uses **OpenPCDet**,
which needs a one-line patch in `centernet_utils.py`:

- The original code hard-codes a `K = 500` reshape, which fails when the actual
  number of detections differs.
- The fix replaces the hard-coded value with `actual_K = scores.shape[1]`, so the
  reshape uses the real detection count.

If OpenPCDet throws a reshape/size-mismatch error in `centernet_utils.py`, this
patch is missing or was overwritten by a reinstall.

## The src/ vs install/ gotcha

This is not strictly a setup step, but it is the gotcha that most often makes a
correct setup look broken. **ROS 2 launches run from the installed copy under
`install/`, not from `src/`.** Editing a file in `src/` and re-running
`ros2 launch` runs the old installed version. After changing anything that ROS 2
launches, rebuild/reinstall, or confirm which copy is actually executing. This,
and the related "a config value present in the file is not necessarily the value
in effect," are covered in [operations.md](operations.md).
