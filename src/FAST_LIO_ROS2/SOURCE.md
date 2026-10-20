# Source and local adaptations

- Upstream: https://github.com/Ericsii/FAST_LIO_ROS2
- Commit: `2fffc570a25d0df172720bac034fbdb6a13d2162`
- ikd-Tree submodule: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- Preserve upstream LICENSE and source copyright notices.

Local changes: optional Livox CustomMsg support when livox_ros_driver2 is
available; lidar_type 5 reads simulated XYZIt, preserves relative timestamps,
converts them using timestamp_unit, and orders points by time. This mode
requires FLOAT32 x/y/z/intensity/time fields. See ../../docs/mapping.md.
Map-save errors (empty map or PCD write failure) return service failure instead
of terminating the node. A preprocessing regression test is included.
The accumulated display/saved map is voxel-filtered with output_map_leaf_size;
the RViz preset shows this map without overlaying a history of dense scans.
Go2 recovery latches mapping into a stopped state, rejects new scans/IMU and
protects the accumulated map until the mapping process is restarted.
