# -*- bazel -*-


def drake_repository(name, *, excludes = [], **kwargs):
    """Declares the @drake repository based on an installed Drake binary image,
    along with repositories for its dependencies @eigen, @fmt, and
    @drake_models.

    This enables downstream BUILD files to refer to certain Drake targets when
    using precompiled Drake releases.

    Only a limited number of targets are supported, currently only:
    - @drake//bindings/pydrake
    - @drake//:drake_shared_library

    For an example of proper use, see
    https://github.com/RobotLocomotion/drake-external-examples/tree/master/drake_bazel_installed

    The `excludes` argument may be used to skip over @eigen and/or @fmt (by
    passing, e.g., `excludes = ["eigen"]`, in which case you are responsible
    for providing your own definition of those repositories.
    """
    _drake_repository(name = name, **kwargs)

    # The list of external repositories here is roughly consistent with Drake's
    # allowed list of public headers as seen in header_dependency_test.py.
    if "eigen" not in excludes:
        _eigen_repository(name = "eigen")
    if "fmt" not in excludes:
        _fmt_repository(name = "fmt", drake_name = name)
    if "drake_models" not in excludes:
        _drake_models_repository(name = "drake_models")

def _drake_impl(repo_ctx):
    # Obtain the root of the @drake_loader repository (i.e., wherever this
    # repo.bzl file came from).
    loader_workspace = repo_ctx.path(Label("//:WORKSPACE")).dirname

    # If the loader came from an http_archive of a Drake binary release, its
    # workspace will have paths like drake/lib/..., drake/share/..., etc.
    # If the loader came from a new_local_repository on disk, the `path = ...`
    # provided to new_local_repository might already incorporate the "drake"
    # prefix so have paths like lib/..., share/..., etc.  We'll automatically
    # detect which case is in effect.
    for prefix in [
        loader_workspace,
        loader_workspace.get_child("drake"),
        None,
    ]:
        if prefix == None:
            fail("Could not find drake sentinel under {}".format(
                loader_workspace,
            ))
        share = prefix.get_child("share")
        share_drake = share.get_child("drake")
        sentinel = share_drake.get_child(".drake-find_resource-sentinel")
        if sentinel.exists:
            break

    # Sanity check ${prefix}.
    required_files = [
        "include/drake/common/drake_assert.h",
        "lib/libdrake.so",
    ]
    for required_file in required_files:
        if not repo_ctx.path(str(prefix) + "/" + required_file).exists:
            fail("The drake install prefix {} is missing file {}".format(
                prefix,
                required_file,
            ))

    # Symlink our libraries into the repository.  We can use any path for
    # these, since our BUILD rules will declare new names for everything,
    # unrelated to their physical structure here.
    repo_ctx.symlink(prefix.get_child("include"), ".include")
    repo_ctx.symlink(prefix.get_child("lib"), ".lib")

    # Create the stub BUILD files.  (During development, these live at
    # drake/tools/install/bazel/drake**.BUILD.bazel in the source tree.)
    for path, body in _BUILD_FILE_CONTENTS.items():
        repo_ctx.file(path, content = body, executable = False)

    # Symlink the data resources into the repository.  These must exactly match
    # a Drake source tree's physical structure, since we cannot easily alter
    # the path for runfiles via our BUILD files.
    for relpath in _MANIFEST["runfiles"]["drake"]:
        repo_ctx.symlink(str(share_drake) + "/" + relpath, relpath)

    # Symlink all drake LCM types to this repository's root package, since it
    # should be named `drake` (see bazelbuild/bazel#3998).
    if repo_ctx.name != "drake":
        print("WARNING: Drake LCM types will not be importable via `drake` " +
              "if this repository is not named `drake`.")
    python_site_packages_relpath = _MANIFEST["python_site_packages_relpath"]
    drake_lcmtypes_package = "." + python_site_packages_relpath + "/drake"
    for relpath in _MANIFEST["lcmtypes_drake_py"]:
        repo_ctx.symlink(drake_lcmtypes_package + "/" + relpath, relpath)

    # Emit the manifest for later loading.
    manifest_bzl = "MANIFEST = " + struct(**_MANIFEST).to_json()
    repo_ctx.file(".manifest.bzl", content = manifest_bzl, executable = False)

    # Annotate the OS for use by our BUILD files.
    os_bzl = "NAME = \"{}\"\n".format(repo_ctx.os.name)
    repo_ctx.file(".os.bzl", content = os_bzl, executable = False)

def _eigen_repository(name):
    native.new_local_repository(
        name = name,
        path = "/usr/include/eigen3",
        build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**", "unsupported/Eigen/**"], allow_empty = False),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
    )

_drake_repository = repository_rule(
    implementation = _drake_impl,
    local = True,
)

def _fmt_impl(repo_ctx):
    repo_ctx.file("BUILD.bazel", """
cc_library(
    name = "fmt",
    deps = ["@{}//:.fmt_headers"],
    visibility = ["//visibility:public"],
)
""".format(repo_ctx.attr.drake_name), executable = False)

_fmt_repository = repository_rule(
    implementation = _fmt_impl,
    attrs = {
        "drake_name": attr.string(mandatory = True),
    },
)

def _drake_models_impl(repo_ctx):
    repo_ctx.file(
        "BUILD.bazel",
        _BUILD_FILE_CONTENTS["external-drake_models-BUILD.bazel"],
    )
    repo_ctx.download_and_extract(
        url = ["https://github.com/RobotLocomotion/models/archive/8cdc23d313c997d7f317de0c6b36014e2906f3c0.tar.gz", "https://drake-mirror.csail.mit.edu/github/RobotLocomotion/models/8cdc23d313c997d7f317de0c6b36014e2906f3c0.tar.gz", "https://s3.amazonaws.com/drake-mirror/github/RobotLocomotion/models/8cdc23d313c997d7f317de0c6b36014e2906f3c0.tar.gz"],
        sha256 = "f1b37fb0f82be4b221bac0497d62cc4505afd8ddf9f46062e97f5797b8d73baa",
        stripPrefix = "models-8cdc23d313c997d7f317de0c6b36014e2906f3c0",
    )

_drake_models_repository = repository_rule(
    implementation = _drake_models_impl,
)

_BUILD_FILE_CONTENTS = {
  "BUILD.bazel": r"""
# -*- bazel -*-

load("@rules_python//python:defs.bzl", "py_library")
load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

_DRAKE_RUNFILES = MANIFEST["runfiles"]["drake"]

_DRAKE_SHLIBS = glob([
    ".lib/libdrake*.so",
    ".lib/libvtk*.so.*",
    # For Mosek (not enabled by default).
    ".lib/libtbb*.so*",
    ".lib/libmosek64*.so*",
    # For Gurobi (not enabled by default).
    ".lib/libgurobi*.so*",
], exclude = [
    ".lib/libvtk*Python*",
    ".lib/libvtk*-8.2.so.*",
])

_PYTHON_SITE_PACKAGES_RELPATH = MANIFEST["python_site_packages_relpath"]

_DRAKE_ROOT_PACKAGE_RUNFILES = [x for x in _DRAKE_RUNFILES if "/" not in x]

_EXPECTED_DRAKE_RUNFILES_PACKAGES = [
    "bindings/pydrake",
    "common",
    "examples",
    "geometry",
    "manipulation",
    "multibody",
]

_COVERED_DRAKE_RUNFILES = _DRAKE_ROOT_PACKAGE_RUNFILES + [
    x
    for x in _DRAKE_RUNFILES
    if any([
        x.startswith(package + "/")
        for package in _EXPECTED_DRAKE_RUNFILES_PACKAGES
    ])
]

(len(_COVERED_DRAKE_RUNFILES) == len(_DRAKE_RUNFILES)) or fail(
    "EXPECTED_DRAKE_RUNFILES_PACKAGES {} did not cover {}".format(
        _EXPECTED_DRAKE_RUNFILES_PACKAGES,
        _DRAKE_RUNFILES,
    ),
)

filegroup(
    name = ".installed_runfiles",
    data = _DRAKE_ROOT_PACKAGE_RUNFILES,
)

filegroup(
    name = ".all_runfiles",
    data = [
        "//:.installed_runfiles",
        "@drake_models",
    ] + [
        "//{}:.installed_runfiles".format(x)
        for x in _EXPECTED_DRAKE_RUNFILES_PACKAGES
    ],
)

cc_library(
    name = ".lcm_coretypes",
    hdrs = [".include/lcm/lcm/lcm_coretypes.h"],
    strip_include_prefix = ".include/lcm",
)

cc_library(
    name = ".drake_lcm_headers",
    hdrs = glob([".include/drake_lcmtypes/drake/**"]),
    strip_include_prefix = ".include/drake_lcmtypes",
    deps = [":.lcm_coretypes"],
)

cc_library(
    name = ".drake_headers",
    hdrs = glob([".include/drake/**"], exclude = ["**/drake_lcmtypes/**"]),
    strip_include_prefix = ".include",
    deps = [":.drake_lcm_headers"],
)

cc_library(
    name = ".fmt_headers",
    hdrs = glob([".include/fmt/**"], allow_empty = True),
    strip_include_prefix = ".include/fmt",
    visibility = ["@fmt//:__pkg__"],
)

[
    cc_import(
        name = ".imported{}".format(shlib),
        shared_library = shlib,
    )
    for shlib in _DRAKE_SHLIBS
]

cc_library(
    name = "drake_shared_library",
    data = [
        ":.all_runfiles",
    ],
    deps = [
        ":.drake_headers",
        "@eigen",
        "@fmt",
    ] + [
        ":.imported{}".format(shlib)
        for shlib in _DRAKE_SHLIBS
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = ".all_shlib_data",
    data = glob([
        ".lib/*.so",
        ".lib/*.so.*",
    ]),
)

_IMPORT = "." + _PYTHON_SITE_PACKAGES_RELPATH

# N.B. This is not a standalone Python library.
# TODO(eric.cousineau): Expose this as an alias
# `@drake//lcmtypes:lcmtypes_drake_py` when it can only depend on specific
# parts of the runfiles (not all of pydrake).
py_library(
    name = ".lcmtypes_drake_py",
    srcs = glob(["*.py"]),
)

py_library(
    name = ".pydrake",
    srcs = glob(include = [
        _IMPORT + "/**/*.py",
    ]),
    data = glob(include = [
        _IMPORT + "/**/*.so",
    ]) + [
        ":.all_runfiles",
        ":.all_shlib_data",
    ],
    deps = [
        ":.lcmtypes_drake_py",
    ],
    imports = [
        _IMPORT,
    ],
)

""",
  "bindings/pydrake/BUILD.bazel": r"""
# -*- bazel -*-

# This is the @drake//bindings/pydrake package.

load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

alias(
    name = "pydrake",
    actual = "//:.pydrake",
    visibility = ["//visibility:public"],
)

_subdir = "bindings/pydrake/"

_runfiles = [
    x[len(_subdir):]
    for x in MANIFEST["runfiles"]["drake"]
    if x.startswith(_subdir)
]

filegroup(
    name = ".installed_runfiles",
    data = _runfiles,
)

""",
  "common/BUILD.bazel": r"""
# -*- bazel -*-

# This is the @drake//common package.

load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

_subdir = "common/"

_runfiles = [
    x[len(_subdir):]
    for x in MANIFEST["runfiles"]["drake"]
    if x.startswith(_subdir)
]

filegroup(
    name = ".installed_runfiles",
    data = _runfiles,
)

""",
  "examples/BUILD.bazel": r"""
# -*- bazel -*-

# This is the @drake//examples package.

load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

_subdir = "examples/"

_runfiles = [
    x[len(_subdir):]
    for x in MANIFEST["runfiles"]["drake"]
    if x.startswith(_subdir)
]

filegroup(
    name = ".installed_runfiles",
    data = _runfiles,
)

""",
  "geometry/BUILD.bazel": r"""
# -*- bazel -*-

# This is the @drake//geometry package.

load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

_subdir = "geometry/"

_runfiles = [
    x[len(_subdir):]
    for x in MANIFEST["runfiles"]["drake"]
    if x.startswith(_subdir)
]

filegroup(
    name = ".installed_runfiles",
    data = _runfiles,
)

""",
  "manipulation/BUILD.bazel": r"""
# -*- bazel -*-

# This is the @drake//manipulation package.

load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

_subdir = "manipulation/"

_runfiles = [
    x[len(_subdir):]
    for x in MANIFEST["runfiles"]["drake"]
    if x.startswith(_subdir)
]

filegroup(
    name = ".installed_runfiles",
    data = _runfiles,
)

""",
  "multibody/BUILD.bazel": r"""
# -*- bazel -*-

# This is the @drake//multibody package.

load("//:.manifest.bzl", "MANIFEST")

package(default_visibility = ["//:__subpackages__"])

_subdir = "multibody/"

_runfiles = [
    x[len(_subdir):]
    for x in MANIFEST["runfiles"]["drake"]
    if x.startswith(_subdir)
]

filegroup(
    name = ".installed_runfiles",
    data = _runfiles,
)

""",
  "external-drake_models-BUILD.bazel": r"""
# -*- bazel -*-

package(default_visibility = ["//visibility:public"])

_SRCS = glob(["**/*"])

exports_files(_SRCS)

filegroup(
    name = "drake_models",
    srcs = _SRCS,
)

# Nominally, the `@drake_models` external is fetched via `github_archive()`,
# which creates a json metadata files to explain what it downloaded. Drake's
# install rules use that file to pin the lazy-download version of the models.
#
# However, in case a developer is using a local checkout of `@drake_models`,
# the json file will not exist. In that case, we need to generate a stub file
# to take its place, so that our Bazel install rules can still find it. We'll
# fill it with dummy data. To guard against shipping a Drake release with the
# dummy data, the package_map_remote_test checks the content of the json file.
glob(["drake_repository_metadata.json"], allow_empty = True) or genrule(
    name = "_gen_dummy_metadata",
    outs = ["drake_repository_metadata.json"],
    cmd = "echo '{}' > $@".format(
        json.encode(dict(
            urls = [],
            sha256 = "",
            strip_prefix = "",
        )),
    ),
)

""",
}


_MANIFEST = {"lcmtypes_drake_py":["__init__.py","experimental_lcmt_deformable_tri.py","experimental_lcmt_deformable_tri_mesh_init.py","experimental_lcmt_deformable_tri_mesh_update.py","experimental_lcmt_deformable_tri_meshes_init.py","experimental_lcmt_deformable_tri_meshes_update.py","lcmt_acrobot_u.py","lcmt_acrobot_x.py","lcmt_acrobot_y.py","lcmt_allegro_command.py","lcmt_allegro_status.py","lcmt_call_python.py","lcmt_call_python_data.py","lcmt_contact_results_for_viz.py","lcmt_drake_signal.py","lcmt_force_torque.py","lcmt_header.py","lcmt_hydroelastic_contact_surface_for_viz.py","lcmt_hydroelastic_quadrature_per_point_data_for_viz.py","lcmt_iiwa_command.py","lcmt_iiwa_status.py","lcmt_iiwa_status_telemetry.py","lcmt_image.py","lcmt_image_array.py","lcmt_jaco_command.py","lcmt_jaco_status.py","lcmt_panda_command.py","lcmt_panda_status.py","lcmt_planar_gripper_command.py","lcmt_planar_gripper_finger_command.py","lcmt_planar_gripper_finger_face_assignment.py","lcmt_planar_gripper_finger_face_assignments.py","lcmt_planar_gripper_finger_status.py","lcmt_planar_gripper_status.py","lcmt_planar_manipuland_status.py","lcmt_planar_plant_state.py","lcmt_point.py","lcmt_point_cloud.py","lcmt_point_cloud_field.py","lcmt_point_pair_contact_info_for_viz.py","lcmt_quaternion.py","lcmt_robot_plan.py","lcmt_robot_state.py","lcmt_schunk_wsg_command.py","lcmt_schunk_wsg_status.py","lcmt_scope.py","lcmt_viewer_command.py","lcmt_viewer_draw.py","lcmt_viewer_geometry_data.py","lcmt_viewer_link_data.py","lcmt_viewer_load_robot.py"],"python_site_packages_relpath":"lib/python3.10/site-packages","runfiles":{"drake":[".drake-find_resource-sentinel","common/resource_tool","examples/acrobot/Acrobot.sdf","examples/acrobot/Acrobot.urdf","examples/acrobot/Acrobot_no_collision.urdf","examples/hydroelastic/spatula_slip_control/models/schunk_wsg_50_hydro_bubble.sdf","examples/hydroelastic/spatula_slip_control/models/spatula.sdf","examples/kuka_iiwa_arm/models/desk/transcendesk55inch.sdf","examples/kuka_iiwa_arm/models/objects/black_box.urdf","examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place.urdf","examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place_large_size.urdf","examples/kuka_iiwa_arm/models/objects/block_for_pick_and_place_mid_size.urdf","examples/kuka_iiwa_arm/models/objects/folding_table.urdf","examples/kuka_iiwa_arm/models/objects/open_top_box.urdf","examples/kuka_iiwa_arm/models/objects/round_table.urdf","examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf","examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf","examples/kuka_iiwa_arm/models/objects/yellow_post.urdf","examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf","examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf","examples/manipulation_station/models/061_foam_brick.sdf","examples/manipulation_station/models/amazon_table_simplified.sdf","examples/manipulation_station/models/bin.sdf","examples/manipulation_station/models/bin2.sdf","examples/manipulation_station/models/cupboard.sdf","examples/manipulation_station/models/cylinder.sdf","examples/manipulation_station/models/shelves.sdf","examples/manipulation_station/models/sphere.sdf","examples/manipulation_station/models/table_wide.sdf","examples/manipulation_station/models/thin_box.sdf","examples/manipulation_station/models/thin_cylinder.sdf","examples/multibody/cart_pole/cart_pole.sdf","examples/pendulum/Pendulum.urdf","examples/planar_gripper/planar_brick.sdf","examples/planar_gripper/planar_gripper.sdf","examples/pr2/README.md","examples/pr2/models/pr2_description/meshes/base_v0/base.obj","examples/pr2/models/pr2_description/meshes/base_v0/base.stl","examples/pr2/models/pr2_description/meshes/base_v0/base_L.obj","examples/pr2/models/pr2_description/meshes/base_v0/base_L.stl","examples/pr2/models/pr2_description/meshes/base_v0/caster.obj","examples/pr2/models/pr2_description/meshes/base_v0/caster.stl","examples/pr2/models/pr2_description/meshes/base_v0/caster_L.obj","examples/pr2/models/pr2_description/meshes/base_v0/caster_L.stl","examples/pr2/models/pr2_description/meshes/base_v0/pr2_wheel.obj","examples/pr2/models/pr2_description/meshes/base_v0/pr2_wheel.stl","examples/pr2/models/pr2_description/meshes/base_v0/wheel.obj","examples/pr2/models/pr2_description/meshes/base_v0/wheel.stl","examples/pr2/models/pr2_description/meshes/forearm_v0/forearm.obj","examples/pr2/models/pr2_description/meshes/forearm_v0/forearm.stl","examples/pr2/models/pr2_description/meshes/forearm_v0/wrist_flex.obj","examples/pr2/models/pr2_description/meshes/forearm_v0/wrist_flex.stl","examples/pr2/models/pr2_description/meshes/forearm_v0/wrist_roll.obj","examples/pr2/models/pr2_description/meshes/forearm_v0/wrist_roll.stl","examples/pr2/models/pr2_description/meshes/forearm_v0/wrist_roll_L.obj","examples/pr2/models/pr2_description/meshes/forearm_v0/wrist_roll_L.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_l.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_l.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_pad2_l.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_pad2_l.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_pad2_r.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_pad2_r.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_r.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/finger_tip_r.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/gripper_palm.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/gripper_palm.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/l_finger.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/l_finger.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/l_finger_tip.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/l_finger_tip.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/l_floating.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/l_floating.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/upper_finger_l.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/upper_finger_l.stl","examples/pr2/models/pr2_description/meshes/gripper_v0/upper_finger_r.obj","examples/pr2/models/pr2_description/meshes/gripper_v0/upper_finger_r.stl","examples/pr2/models/pr2_description/meshes/head_v0/head_pan.obj","examples/pr2/models/pr2_description/meshes/head_v0/head_pan.stl","examples/pr2/models/pr2_description/meshes/head_v0/head_pan_L.obj","examples/pr2/models/pr2_description/meshes/head_v0/head_pan_L.stl","examples/pr2/models/pr2_description/meshes/head_v0/head_tilt.obj","examples/pr2/models/pr2_description/meshes/head_v0/head_tilt.stl","examples/pr2/models/pr2_description/meshes/head_v0/head_tilt_L.obj","examples/pr2/models/pr2_description/meshes/head_v0/head_tilt_L.stl","examples/pr2/models/pr2_description/meshes/sensors/kinect_v0/kinect_mount.obj","examples/pr2/models/pr2_description/meshes/sensors/kinect_v0/kinect_mount.stl","examples/pr2/models/pr2_description/meshes/shoulder_v0/shoulder_lift.obj","examples/pr2/models/pr2_description/meshes/shoulder_v0/shoulder_lift.stl","examples/pr2/models/pr2_description/meshes/shoulder_v0/shoulder_pan.obj","examples/pr2/models/pr2_description/meshes/shoulder_v0/shoulder_pan.stl","examples/pr2/models/pr2_description/meshes/shoulder_v0/shoulder_yaw.obj","examples/pr2/models/pr2_description/meshes/shoulder_v0/shoulder_yaw.stl","examples/pr2/models/pr2_description/meshes/shoulder_v0/upper_arm_roll.obj","examples/pr2/models/pr2_description/meshes/shoulder_v0/upper_arm_roll.stl","examples/pr2/models/pr2_description/meshes/shoulder_v0/upper_arm_roll_L.obj","examples/pr2/models/pr2_description/meshes/shoulder_v0/upper_arm_roll_L.stl","examples/pr2/models/pr2_description/meshes/tilting_laser_v0/hok_tilt.obj","examples/pr2/models/pr2_description/meshes/tilting_laser_v0/hok_tilt.stl","examples/pr2/models/pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.obj","examples/pr2/models/pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.stl","examples/pr2/models/pr2_description/meshes/tilting_laser_v0/tilting_hokuyo_L.obj","examples/pr2/models/pr2_description/meshes/tilting_laser_v0/tilting_hokuyo_L.stl","examples/pr2/models/pr2_description/meshes/torso_v0/torso.obj","examples/pr2/models/pr2_description/meshes/torso_v0/torso.stl","examples/pr2/models/pr2_description/meshes/torso_v0/torso_lift.obj","examples/pr2/models/pr2_description/meshes/torso_v0/torso_lift.stl","examples/pr2/models/pr2_description/meshes/torso_v0/torso_lift_L.obj","examples/pr2/models/pr2_description/meshes/torso_v0/torso_lift_L.stl","examples/pr2/models/pr2_description/meshes/upper_arm_v0/elbow_flex.obj","examples/pr2/models/pr2_description/meshes/upper_arm_v0/elbow_flex.stl","examples/pr2/models/pr2_description/meshes/upper_arm_v0/forearm_roll.obj","examples/pr2/models/pr2_description/meshes/upper_arm_v0/forearm_roll.stl","examples/pr2/models/pr2_description/meshes/upper_arm_v0/forearm_roll_L.obj","examples/pr2/models/pr2_description/meshes/upper_arm_v0/forearm_roll_L.stl","examples/pr2/models/pr2_description/meshes/upper_arm_v0/upper_arm.obj","examples/pr2/models/pr2_description/meshes/upper_arm_v0/upper_arm.stl","examples/pr2/models/pr2_description/urdf/pr2_simplified.urdf","examples/quadrotor/office.urdf","examples/quadrotor/quadrotor.urdf","examples/quadrotor/warehouse.sdf","geometry/meshcat.html","geometry/meshcat.ico","geometry/meshcat.js","geometry/stats.min.js","manipulation/models/allegro_hand_description/sdf/allegro_hand_description_left.sdf","manipulation/models/allegro_hand_description/sdf/allegro_hand_description_right.sdf","manipulation/models/allegro_hand_description/urdf/allegro_hand_description_left.urdf","manipulation/models/allegro_hand_description/urdf/allegro_hand_description_right.urdf","manipulation/models/franka_description/README.md","manipulation/models/franka_description/urdf/hand.urdf","manipulation/models/franka_description/urdf/panda_arm.urdf","manipulation/models/franka_description/urdf/panda_arm_hand.urdf","manipulation/models/franka_description/urdf/panda_hand.urdf","manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf","manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf","manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf","manipulation/models/iiwa_description/sdf/iiwa14_polytope_collision.sdf","manipulation/models/iiwa_description/urdf/dual_iiwa14_polytope_collision.urdf","manipulation/models/iiwa_description/urdf/iiwa14_no_collision.urdf","manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf","manipulation/models/iiwa_description/urdf/iiwa14_primitive_collision.urdf","manipulation/models/iiwa_description/urdf/iiwa14_spheres_collision.urdf","manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf","manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_elbow_collision.urdf","manipulation/models/iiwa_description/urdf/planar_iiwa14_spheres_dense_elbow_collision.urdf","manipulation/models/jaco_description/README.md","manipulation/models/jaco_description/urdf/j2n6s300.urdf","manipulation/models/jaco_description/urdf/j2n6s300_col.urdf","manipulation/models/jaco_description/urdf/j2s7s300.urdf","manipulation/models/jaco_description/urdf/j2s7s300_arm.urdf","manipulation/models/jaco_description/urdf/j2s7s300_arm_sphere_collision.urdf","manipulation/models/jaco_description/urdf/j2s7s300_hand.urdf","manipulation/models/jaco_description/urdf/j2s7s300_hand_sphere_collision.urdf","manipulation/models/jaco_description/urdf/j2s7s300_sphere_collision.urdf","manipulation/models/realsense2_description/meshes/d415.obj","manipulation/models/realsense2_description/urdf/d415.urdf","manipulation/models/tri_homecart/homecart.dmd.yaml","manipulation/models/tri_homecart/homecart_bimanual.urdf","manipulation/models/tri_homecart/homecart_collision_walls.sdf","manipulation/models/tri_homecart/homecart_cutting_board.sdf","manipulation/models/tri_homecart/homecart_grippers.dmd.yaml","manipulation/models/tri_homecart/homecart_no_grippers.dmd.yaml","manipulation/models/ur3e/ur3e_cylinders_collision.urdf","manipulation/models/ur3e/ur3e_spheres_collision.urdf","manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf","manipulation/models/wsg_50_description/sdf/schunk_wsg_50_ball_contact.sdf","manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf","manipulation/models/wsg_50_description/sdf/schunk_wsg_50_welded_fingers.sdf","manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf","manipulation/models/ycb/sdf/003_cracker_box.sdf","manipulation/models/ycb/sdf/004_sugar_box.sdf","manipulation/models/ycb/sdf/005_tomato_soup_can.sdf","manipulation/models/ycb/sdf/006_mustard_bottle.sdf","manipulation/models/ycb/sdf/009_gelatin_box.sdf","manipulation/models/ycb/sdf/010_potted_meat_can.sdf","multibody/parsing/drake_models.json","multibody/parsing/package_downloader.py","package.xml"]}}