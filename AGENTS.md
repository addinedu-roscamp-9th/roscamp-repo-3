# Agent Guidelines for ROS2 Human Care Robot Project

This document provides build commands, code style guidelines, and development conventions for AI coding agents working in this repository.

## Project Overview

- **Type**: ROS 2 (Jazzy) robotics project with Python services
- **Language**: Python 3.12
- **Main Components**:
  - `/porter/` - ROS 2 workspace for robot control nodes
  - `/arm/` - FastAPI service for Jetcobot robotic arm control
  - `/TechnicalResearch/server/` - Central gateway FastAPI server

## Build Commands

### ROS 2 Workspace (porter/)

```bash
# Navigate to workspace
cd /home/lemon/dev/roscamp-repo-3/porter

# Build all packages
colcon build

# Build with symlink (faster for Python development)
colcon build --symlink-install

# Build specific package only
colcon build --packages-select debugcrew

# Rebuild from scratch
colcon build --cmake-clean-cache

# Source the workspace after building
source install/setup.bash
```

### Python Services (arm/, TechnicalResearch/server/)

```bash
# Setup virtual environment (first time)
cd /home/lemon/dev/roscamp-repo-3/arm
bash venv_setup.sh

# Activate virtual environment
source .venv/bin/activate

# Run the service
python main.py
```

## Test Commands

### ROS 2 Packages

```bash
# From porter/ directory
cd /home/lemon/dev/roscamp-repo-3/porter

# Run all tests
colcon test

# Test specific package
colcon test --packages-select debugcrew

# Show test results
colcon test-result --verbose

# Run single test file directly with pytest
cd src/debugcrew
pytest test/test_flake8.py

# Run single test by name
pytest -k test_flake8

# Run with verbose output
pytest -v test/test_pep257.py
```

### Active Test Suites
- **flake8**: PEP 8 code style checking
- **pep257**: Docstring conventions (PEP 257)
- **copyright**: Header validation (currently skipped)

## Lint Commands

### ROS 2 Packages

```bash
# Linting is integrated into colcon test
colcon test --packages-select debugcrew

# Manual flake8 check
flake8 src/debugcrew/debugcrew/
```

### Python Services (arm/, TechnicalResearch/server/)

```bash
# Pylint check
pylint app/ main.py

# Pyright static type checking
pyright
```

## Code Style Guidelines

### Import Organization

```python
# 1. Standard library imports
import os
import threading
from typing import List, Optional

# 2. Third-party imports
import rclpy as rp
import uvicorn
from dotenv import load_dotenv
from fastapi import FastAPI
from geometry_msgs.msg import Twist

# 3. Local imports (relative)
from app.controller.controller import Controller
from app.model.posture import Posture
```

### Naming Conventions

- **Variables/Functions**: `snake_case`
- **Classes**: `PascalCase`
- **Constants**: `UPPER_SNAKE_CASE`
- **ROS 2 Node Names**: `snake_case` (e.g., "pinky_node")
- **ROS 2 Topics**: `/namespace/topic_name` (e.g., "/pinky/target")
- **Private Members**: Prefix with underscore `_private_method`

### Type Hints

Always use type hints for function signatures (Python 3.12):

```python
def main(args=None) -> None:
    pass

def calculate_distance(x: float, y: float) -> float:
    return (x**2 + y**2)**0.5

async def handle_request(req: List[Posture]) -> dict[str, bool]:
    return {"success": True}
```

### String Formatting

Use f-strings for all string interpolation:

```python
# Good
self.get_logger().info(f"Received target: x={msg.x}, y={msg.y}")
print(f"Error: {error_message}")

# Avoid
print("Error: " + error_message)
```

### Error Handling

Use try/except blocks with proper logging:

```python
# ROS 2 nodes
try:
    rp.spin(node)
except Exception as e:
    node.get_logger().error(f"Error: {e}")
finally:
    node.destroy_node()
    rp.shutdown()

# FastAPI services
try:
    result = controller.execute()
    return {"success": result}
except Exception as e:
    print(f"Error in handle_pose: {e}")
    return {"success": False, "error": str(e)}
```

### ROS 2 Specific Patterns

```python
class MyNode(Node):
    def __init__(self):
        super().__init__("node_name")
        
        # Publishers
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscriptions with callback
        self.subscription = self.create_subscription(
            PinkyTarget, "/pinky/target", self.callback, 10
        )
        
        # Logging
        self.get_logger().info("Node initialized")
        self.get_logger().warn("Warning message")
        self.get_logger().error("Error message")
    
    def callback(self, msg: PinkyTarget) -> None:
        """Process incoming message."""
        self.get_logger().info(f"Received: {msg.x}")
```

### Docstrings

Use simple, descriptive docstrings (PEP 257 enforced):

```python
def calculate_position(x: float, y: float) -> tuple[float, float]:
    """Calculate the target position based on coordinates."""
    return (x * 2, y * 2)

class Controller:
    """Controls robot posture execution."""
    
    def execute(self) -> bool:
        """Execute the posture sequence."""
        pass
```

### Linter Exceptions

The following are disabled in pylint configuration:
- `missing-docstring` - Docstrings not strictly required
- `too-few-public-methods` - Simple classes allowed
- `wrong-import-order` - Import order not enforced by pylint
- `broad-except` - General exception catching is acceptable

## Dependencies

### ROS 2 Packages
Dependencies declared in `package.xml`:
- Use `<exec_depend>` for runtime dependencies
- Use `<build_depend>` for build-time dependencies
- Use `<test_depend>` for test dependencies

### Python Services
Dependencies managed via `venv_setup.sh` scripts:
- Virtual environment in `.venv/`
- Packages installed via pip
- No requirements.txt files

## Environment Variables

Load from `.env` files:

```python
from dotenv import load_dotenv
import os

load_dotenv()
HOST = os.getenv("GATEWAY_HOST", "192.168.0.56")  # Provide defaults
PORT = int(os.getenv("GATEWAY_PORT", "8000"))
```

## Language

- **Code**: English (variable names, function names, comments)
- **Comments**: Korean comments are acceptable for internal documentation
- **Documentation**: Prefer English for reusability

## Key Files

- `porter/src/debugcrew/package.xml` - ROS 2 package manifest
- `porter/src/debugcrew/setup.py` - Python package setup
- `arm/.pylintrc` - Pylint configuration
- `arm/pyrightconfig.json` - Pyright type checker config
- `.gitignore` - Comprehensive ignore list for Python, ROS 2, venv

## Best Practices

1. **Always source the workspace** after building ROS 2 packages
2. **Use type hints** consistently for better code clarity
3. **Log errors** instead of silent failures
4. **Test changes** with `colcon test` before committing
5. **Follow ROS 2 conventions** for node names and topics
6. **Use virtual environments** for Python services
7. **Avoid force-pushing** to main branches
