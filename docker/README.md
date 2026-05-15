## OCS2 Jazzy Docker

Build:

```bash
docker build -f docker/Dockerfile.jazzy -t ocs2:jazzy .
```

Run:

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ocs2:jazzy
```

Note: the Dockerfile clones `ocs2_robotic_assets` from its `ros2` branch.
It also sparse-checkouts the `plane_segmentation/` packages from `elevation_mapping_cupy` (branch `ros2`) for the perceptive examples.
