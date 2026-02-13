# Nav2 Navigation Server Troubleshooting

## Issue: "Navigation action server not available"

### Root Cause

The Nav2 lifecycle nodes (especially `bt_navigator`) are not automatically transitioned to the `active` state when launched. ROS 2 lifecycle nodes go through these states:

```
unconfigured → inactive → active
```

The `navigate_to_pose` action is only available when `bt_navigator` is in the `active` state.

### Quick Fix (Temporary)

Run the activation script after launching Nav2:

```bash
cd /home/lemon/dev/roscamp-repo-3/porter
./activate_nav2.sh
```

### Manual Activation

If you need to manually activate nodes:

```bash
# Check node state
ros2 lifecycle get /bt_navigator

# Configure node (if unconfigured)
ros2 lifecycle set /bt_navigator configure

# Activate node (if inactive)
ros2 lifecycle set /bt_navigator activate

# Verify action is available
ros2 action list | grep navigate_to_pose
```

### Required Nav2 Nodes

These nodes must be active for full navigation functionality:

- `/bt_navigator` - Provides `navigate_to_pose` action
- `/controller_server` - Local trajectory controller
- `/planner_server` - Global path planner
- `/behavior_server` - Recovery behaviors
- `/smoother_server` - Path smoothing
- `/waypoint_follower` - Waypoint following

### Permanent Fix

The proper solution is to ensure a lifecycle manager is configured to automatically activate navigation nodes on startup. This typically requires:

1. A `lifecycle_manager_navigation` node configured with all navigation nodes
2. `autostart: true` parameter in Nav2 launch configuration

Example configuration:

```python
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    parameters=[{
        'node_names': [
            'bt_navigator',
            'controller_server', 
            'planner_server',
            'behavior_server',
            'smoother_server',
            'waypoint_follower'
        ],
        'autostart': True,
        'bond_timeout': 4.0
    }]
)
```

### Checking Nav2 Status

```bash
# List all lifecycle nodes
ros2 lifecycle nodes

# Check if actions are available
ros2 action list

# Check nav2_container
ros2 node info /nav2_container

# Check lifecycle managers
ros2 node list | grep lifecycle
```

### Related Files

- `pinky_service.py` - Service that calls navigate_to_pose action (line 66)
- `activate_nav2.sh` - Script to activate Nav2 nodes

### Additional Notes

- The Nav2 stack may need to be relaunched if nodes crash
- Check `ros2 node list` to verify all Nav2 nodes are running
- The timeout in `pinky_service.py` is set to 5 seconds (line 66)
