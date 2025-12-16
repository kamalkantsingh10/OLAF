# Tests

Integration tests for OLAF's complete system.

## Structure

```
tests/
├── integration/    # Full system integration tests
└── fixtures/       # Test fixtures, mock data
```

## Test Philosophy

- **Unit tests:** Located with source code (`firmware/modules/*/test/`, ROS2 package tests)
- **Integration tests:** Located here, test cross-module interactions
- **End-to-end tests:** Full system behavior from user input to robot action

## Integration Tests

**Location:** `tests/integration/`

Test interactions between modules, ROS2 nodes, and AI services:

### test_i2c_communication.py

Test I2C bus communication between Pi and all modules:

```bash
pytest tests/integration/test_i2c_communication.py
```

Verifies:
- All modules respond to I2C queries
- Register reads/writes work correctly
- No bus conflicts or timing issues

### test_personality_sync.py

Test synchronized expressions across modules:

```bash
pytest tests/integration/test_personality_sync.py
```

Verifies:
- Eyes, ears, neck move in coordinated fashion
- Expression changes propagate within 500ms
- Personality state machine transitions correctly

### test_navigation_stack.py

Test navigation with self-balancing base:

```bash
pytest tests/integration/test_navigation_stack.py
```

Verifies:
- Nav2 path planning works
- Base responds to velocity commands
- Self-balancing maintains stability during movement

### test_projector_integration.py

Test projector control via Head+Ears module:

```bash
pytest tests/integration/test_projector_integration.py
```

Verifies:
- ROS2 projector commands reach ESP32 via I2C
- Power control (optocoupler) works
- Focus servo responds to commands
- HDMI video output synchronized with power state

## Running Tests

### Run All Integration Tests

```bash
# From project root
pytest tests/integration/

# With verbose output
pytest tests/integration/ -v

# With coverage report
pytest tests/integration/ --cov=ros2/src
```

### Run Specific Test

```bash
pytest tests/integration/test_projector_integration.py::test_projector_power_on
```

### Run in Simulation Mode

Use I2C module simulator for testing without hardware:

```bash
# Terminal 1: Start simulator
python3 tools/simulators/i2c_module_simulator.py

# Terminal 2: Run tests
pytest tests/integration/ --simulator
```

## Test Fixtures

**Location:** `tests/fixtures/`

Reusable test data and mock objects:

- `mock_i2c_devices.py` - Mock I2C module responses
- `sample_conversations.json` - Sample conversation data for AI tests
- `test_maps.pgm` - Test maps for navigation testing
- `calibration_data/` - Pre-calibrated sensor data

## Writing New Tests

### Test Structure

```python
import pytest
from tests.fixtures.mock_i2c_devices import MockHeadEarsModule

class TestProjectorControl:
    """Test projector control via Head+Ears module."""

    def setup_method(self):
        """Setup before each test."""
        self.module = MockHeadEarsModule()

    def test_projector_power_on(self):
        """Test turning projector on."""
        result = self.module.set_projector_power(True)
        assert result == True
        assert self.module.get_projector_status()["powered"] == True

    def teardown_method(self):
        """Cleanup after each test."""
        self.module.close()
```

### Best Practices

1. **Isolation:** Each test should be independent
2. **Fixtures:** Use pytest fixtures for common setup
3. **Assertions:** Use descriptive assertion messages
4. **Cleanup:** Always clean up resources in teardown
5. **Documentation:** Add docstrings to test functions

### Pytest Fixtures Example

```python
@pytest.fixture
def head_ears_module():
    """Fixture providing Head+Ears module interface."""
    module = HeadEarsDriver(i2c_address=0x08)
    yield module
    module.close()

def test_projector_focus(head_ears_module):
    """Test projector focus adjustment."""
    head_ears_module.set_projector_focus(128)
    assert head_ears_module.get_projector_focus() == 128
```

## Continuous Integration

Tests run automatically on:
- Pull requests
- Commits to main branch
- Nightly builds

See `.github/workflows/test.yml` for CI configuration.

## Test Coverage

Check test coverage:

```bash
pytest tests/integration/ --cov=ros2/src --cov-report=html
open htmlcov/index.html
```

**Coverage Goals:**
- Critical paths: >90%
- Driver nodes: >80%
- Overall: >70%

## Performance Testing

For performance-critical tests (e.g., I2C latency):

```bash
pytest tests/integration/test_i2c_communication.py --benchmark
```

Uses `pytest-benchmark` for timing measurements.

## Requirements

Install test dependencies:

```bash
pip install pytest pytest-cov pytest-benchmark
```

Or via Poetry:

```bash
poetry install --with dev
```

## Troubleshooting

### Tests Fail: "I2C Device Not Found"

Ensure modules are powered and connected:
```bash
python3 tools/diagnostics/i2c_scanner.py
```

### Tests Hang: ROS2 Communication Timeout

Check ROS2 nodes are running:
```bash
ros2 node list
```

### Flaky Tests

Use `pytest-repeat` to catch intermittent failures:
```bash
pytest tests/integration/test_i2c_communication.py --count=10
```

## Resources

- [Pytest Documentation](https://docs.pytest.org/)
- [ROS2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Testing.html)
