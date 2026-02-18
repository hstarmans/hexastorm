# Test Patterns

The `hexastorm` package provides utilities for generating and visualizing calibration patterns. These patterns are essential for both mechanical alignment and algorithmic validation.

## Pattern Types

| Category | Purpose |
| --- | --- |
| **Machine Patterns** | Used to physically align the machine hardware and offsets between subsequent lanes. |
| **Camera Patterns** | Used to verify the accuracy of the facet correction algorithms using a camera. |

## Generation

To generate patterns, ensure your environment is synced with the required dependency groups, then use the `uv run` command:

```console
# Ensure dependencies are present
uv sync --system --group camera --group desktop

# Run the pattern generator
uv run python -m hexastorm.interpolator.patterns.machine

```