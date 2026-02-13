#!/bin/bash

# Script to activate all Nav2 lifecycle nodes
# This should be run after Nav2 is launched

echo "Activating Nav2 navigation nodes..."

# List of Nav2 nodes to activate
NODES=(
  "/bt_navigator"
  "/controller_server"
  "/planner_server"
  "/behavior_server"
  "/smoother_server"
  "/waypoint_follower"
)

# Function to activate a node
activate_node() {
  local node=$1
  echo "Activating $node..."

  # Check if node exists
  if ! ros2 lifecycle list "$node" &>/dev/null; then
    echo "  ⚠️  Node $node not found, skipping..."
    return 1
  fi

  # Get current state
  current_state=$(ros2 lifecycle get "$node" 2>&1 | head -1)
  echo "  Current state: $current_state"

  # Transition based on current state
  if echo "$current_state" | grep -q "unconfigured"; then
    echo "  Configuring..."
    ros2 lifecycle set "$node" configure
    sleep 0.5
  fi

  if echo "$current_state" | grep -q "inactive" || echo "$current_state" | grep -q "unconfigured"; then
    echo "  Activating..."
    ros2 lifecycle set "$node" activate
    sleep 0.5
  elif echo "$current_state" | grep -q "active"; then
    echo "  ✓ Already active"
  fi
}

# Activate all nodes
for node in "${NODES[@]}"; do
  activate_node "$node"
done

echo ""
echo "Checking available actions..."
ros2 action list

echo ""
echo "✓ Nav2 activation complete!"
echo "The navigate_to_pose action should now be available."
