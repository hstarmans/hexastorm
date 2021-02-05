""" debug spi

Only got it working for older version of luna
git checkout hw-r0.2-154-ge49508f
"""
from examples.hw_features.debug_spi import DebugSPIExample

from luna import top_level_cli

if __name__ == "__main__":
    top_level_cli(DebugSPIExample)
