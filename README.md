# Kestrel SLAM

**Stereo-inertial SLAM for ROS 2 with loop closure and lifelong mapping in changing environments,
built for the Orbbec Gemini 335L / 335Lg and written as teaching code.**

> 🚧 **Status: milestone M0 (scaffolding) complete.** The SLAM algorithms are being built milestone
> by milestone; see [docs/DEV_PLAN.md](docs/DEV_PLAN.md). The data-capture tool already works, so
> you can start recording datasets now.

## What it will do

* **Visual-inertial odometry** from the 335L's global-shutter stereo IR pair and internal IMU
  (sliding-window optimization with IMU preintegration).
* **Loop closure**: bag-of-words place recognition, geometric verification and a 4-DoF pose graph.
* **Lifelong maps**: per-landmark *persistence filters* remove objects that moved or disappeared,
  and a voxel map reports appeared and disappeared regions.
* **Dynamic-object robustness**: moving people and vehicles are detected and ignored.
* **Multi-session**: save a map, reload it later, relocalize, and keep updating it.
* **ROS 2 / Nav2 ready**: `map → odom → base_link` TF, odometry, and a 2D occupancy grid.
* **Teaching code**: each module comes with a theory chapter ([docs/theory](docs/theory/)) and
  references ([docs/REFERENCES.md](docs/REFERENCES.md)).

## Quick start (only Docker required)

```bash
docker compose build              # build the dev image (first time ~10-15 min)
docker compose run --rm test      # build everything and run the unit tests
docker compose run --rm shell     # interactive shell with all tools
```

### Record data with the camera

```bash
docker compose run --rm capture session:=office scenario:=loop
```

See [docs/hardware/capture_guide.md](docs/hardware/capture_guide.md) for what to record:
calibration, loops, and before/after pairs for changing environments.

### Develop in VS Code

Open the folder and choose **Reopen in Container** (Dev Containers extension).

## Learning from this repository

Kestrel is meant to be read as well as run:

1. [docs/theory/](docs/theory/README.md): textbook-style chapters covering Lie groups, cameras and
   stereo, IMU preintegration, feature tracking, nonlinear least squares, sliding-window VIO,
   dynamic objects, loop closure, lifelong mapping, volumetric mapping and evaluation. Each has
   derivations, pitfalls and exercises.
2. [docs/REFERENCES.md](docs/REFERENCES.md): the papers behind every module, with a note on why
   each one is worth reading.
3. The code: every source file opens with a *TUTORIAL NOTE* explaining the design, and the
   comments cite the theory chapter or paper they implement.

## Repository layout

| Path | What |
|------|------|
| `kestrel_core/` | The SLAM library: pure C++17, no ROS dependency, unit-tested |
| `kestrel_ros/` | ROS 2 node (adapter between ROS messages and the library) |
| `kestrel_interfaces/` | ROS 2 service definitions (save/load/reset map) |
| `kestrel_capture/` | Rosbag recorder + live data-quality monitor for the Gemini 335Lg |
| `docker/`, `compose.yaml` | Container images and services |
| `docs/` | Dev plan, theory chapters, references, hardware guides |

## License

Apache-2.0. See [LICENSE](LICENSE).
