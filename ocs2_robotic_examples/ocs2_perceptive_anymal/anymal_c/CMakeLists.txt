cmake_minimum_required(VERSION 3.5)
project(anymal_c)

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Locate the ocs2_robotic_assets mesh source directory.
# ros2 pkg prefix returns the install prefix of the package, e.g.
#   /ws/install/ocs2_robotic_assets
# ---------------------------------------------------------------------------
execute_process(
  COMMAND ros2 pkg prefix ocs2_robotic_assets
  OUTPUT_VARIABLE _ra_prefix
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

if(NOT _ra_prefix)
  message(FATAL_ERROR
    "anymal_c: could not find ocs2_robotic_assets via 'ros2 pkg prefix'. "
    "Make sure the package is built and sourced before building anymal_c.")
endif()

set(_mesh_src "${_ra_prefix}/share/ocs2_robotic_assets/resources/anymal_c/meshes")
set(_urdf_dst "${CMAKE_CURRENT_BINARY_DIR}/urdf")

message(STATUS "anymal_c: sourcing meshes from ${_mesh_src}")

# ---------------------------------------------------------------------------
# Helper macro: copy one mesh file from the flat meshes/ directory into the
# versioned subdirectory tree expected by anymal_camel_rsl.urdf.
# Also copies the matching .jpg texture so relative references inside the
# .dae remain valid (Collada files reference textures by filename in the
# same directory).
# Usage: copy_mesh(<src_basename_no_ext> <dest_subdir> <dest_mesh_name>)
# ---------------------------------------------------------------------------
macro(copy_mesh SRC_BASE DST_SUBDIR DST_NAME)
  file(MAKE_DIRECTORY "${_urdf_dst}/${DST_SUBDIR}")
  # geometry
  configure_file(
    "${_mesh_src}/${SRC_BASE}.dae"
    "${_urdf_dst}/${DST_SUBDIR}/${DST_NAME}"
    COPYONLY
  )
  # texture — keep the original filename so the .dae's internal reference works
  if(EXISTS "${_mesh_src}/${SRC_BASE}.jpg")
    configure_file(
      "${_mesh_src}/${SRC_BASE}.jpg"
      "${_urdf_dst}/${DST_SUBDIR}/${SRC_BASE}.jpg"
      COPYONLY
    )
  endif()
endmacro()

# base body
copy_mesh(base              "base"                           "base_3_1_1_mesh.dae")
copy_mesh(bottom_shell      "base/shell"                     "bottom_shell_3_1_1_mesh.dae")
copy_mesh(top_shell         "base/shell"                     "top_shell_3_1_1_mesh.dae")
copy_mesh(battery           "base/battery"                   "battery_3_0_4_mesh.dae")
copy_mesh(face              "base/face"                      "face_3_1_1_mesh.dae")
copy_mesh(handle            "base/handle"                    "handle_3_1_1_mesh.dae")
copy_mesh(hatch             "base/hatch"                     "hatch_3_1_1_mesh.dae")
copy_mesh(remote            "base/remote"                    "remote_3_1_1_mesh.dae")

# sensors
copy_mesh(depth_camera      "base/sensors/depth_camera"      "realsense_d435_3_1_1_mesh.dae")
copy_mesh(lidar             "base/sensors/lidar"             "velodyne_3_1_1_mesh.dae")
copy_mesh(wide_angle_camera "base/sensors/wide_angle_camera" "blackfly_3_1_1_mesh.dae")

# legs
copy_mesh(drive             "leg/drives"                     "drive_3_0_2_mesh.dae")
copy_mesh(foot              "leg/foot"                       "foot_3_1_1_mesh.dae")
copy_mesh(hip_l             "leg/hip"                        "hip_3_1_1_mesh.dae")
copy_mesh(hip_r             "leg/hip"                        "hip_3_1_1_mesh_mirrored.dae")
copy_mesh(shank_l           "leg/shank"                      "shank_3_1_1_mesh.dae")
copy_mesh(shank_r           "leg/shank"                      "shank_3_1_1_mesh_mirrored.dae")
copy_mesh(thigh             "leg/thigh"                      "thigh_3_1_1_mesh.dae")

install(
  DIRECTORY "${_urdf_dst}/"
  DESTINATION "share/${PROJECT_NAME}/urdf"
)

ament_package()
