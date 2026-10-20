Mesh and textures: https://github.com/clearpathrobotics/cpr_gazebo
Commit: 509afbeec942c6aad376a5a76a24330f1c56dc17
Copied from cpr_agriculture_gazebo/meshes; BSD license retained in LICENSE.
model.sdf geometry, collision, and pose come from the local robotdog_nav
robot_scene/worlds/agriculture.world. Mesh scale remains 1:1.
Classic world removes saved state while retaining original geometry, scale and physics.
The reference copy omits the mesh; upstream resources restore that dependency.
