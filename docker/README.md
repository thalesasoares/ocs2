## OCS2 Jazzy Docker

Two Docker setups are provided:

| File | Purpose |
|------|---------|
| `Dockerfile.jazzy` | Self-contained production image — copies and builds the whole workspace inside the image. |
| `Dockerfile.dev` + `docker-compose.yml` | Developer image — mounts source and build artifacts from the host so you can edit code and rebuild without touching the image. |

---

### Production image (Dockerfile.jazzy)

Build and run a fully self-contained image:

```bash
# Build (run from the repo root)
docker build -f docker/Dockerfile.jazzy -t ocs2:jazzy .

# Run
docker run --rm -it --net=host ocs2:jazzy
```

Note: the Dockerfile clones `ocs2_robotic_assets` from its `ros2` branch.
It also sparse-checkouts the `plane_segmentation/` packages from `elevation_mapping_cupy` (branch `ros2`) for the perceptive examples.

---

### Developer image (Dockerfile.dev + docker-compose.yml)

The developer setup binds your local source tree and build artifacts into the container so that:

- Code edits on the host are **immediately visible** inside the container — no image rebuild needed.
- Compiled artifacts (`build/`, `install/`, `log/`) live in `ws/` at the repo root and **survive container restarts**.
- GUI tools (RViz2, etc.) are forwarded to your host display via X11.
- Terminal output is **fully colored**.

#### Directory layout after first build

```
ocs2/               ← repo root (this folder)
├── docker/
│   ├── Dockerfile.dev
│   ├── docker-compose.yml
│   └── .env            ← ROS_DOMAIN_ID, DISPLAY defaults
└── ws/                 ← created automatically, gitignored
    ├── build/
    ├── install/
    └── log/
```

#### One-time host setup

Allow the container to open windows on your display (run once per login session):

```bash
xhost +local:docker
```

#### Build the dev image (once, or after apt/Dockerfile changes)

```bash
# Run from the repo root
docker compose -f docker/docker-compose.yml build
```

#### Start the container

```bash
docker compose -f docker/docker-compose.yml up -d
```

#### Attach a shell

```bash
docker compose -f docker/docker-compose.yml exec ocs2 bash
```

#### First build inside the container

```bash
# Inside the container — /ws is the colcon workspace root
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`--symlink-install` avoids copying Python files into `install/`, so Python edits take effect without rebuilding.

#### Rebuilding after C++ changes

```bash
# Rebuild only the packages that changed
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    --packages-select <package_name>

# Or rebuild everything
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

#### Stop the container

```bash
docker compose -f docker/docker-compose.yml down
```

#### Environment variables

Edit `docker/.env` to change defaults:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | `0` | ROS2 DDS domain — isolates traffic from other ROS2 systems |
| `DISPLAY` | `:0` | X11 display server used when `$DISPLAY` is not set in the host shell |

You can also override any variable inline without editing `.env`:

```bash
ROS_DOMAIN_ID=42 docker compose -f docker/docker-compose.yml up -d
```
