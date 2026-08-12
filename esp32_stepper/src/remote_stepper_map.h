#pragma once

// Arduino-independent address mapping, also exercised by the native tests.
static inline bool remote_stepper_global_to_node(int global_axis,
                                                 int local_axis_count,
                                                 int node_count,
                                                 int axes_per_node,
                                                 int& node,
                                                 int& local_axis) {
  const int offset = global_axis - local_axis_count;
  if (offset < 0 || offset >= node_count * axes_per_node) return false;
  node = offset / axes_per_node + 1;
  local_axis = offset % axes_per_node;
  return true;
}

static inline bool remote_stepper_node_to_global(int node,
                                                 int local_axis,
                                                 int local_axis_count,
                                                 int node_count,
                                                 int axes_per_node,
                                                 int& global_axis) {
  if (node < 1 || node > node_count ||
      local_axis < 0 || local_axis >= axes_per_node) return false;
  global_axis = local_axis_count + (node - 1) * axes_per_node + local_axis;
  return true;
}
