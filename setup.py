import sys

from setuptools import Extension, setup

extra_compile_args = (
    ["/O2"] if sys.platform == "win32" else ["-O3", "-std=c99", "-Wall"]
)

vector_core_ext = Extension(
    "hexastorm.exposure._vector_core",
    sources=[
        "src/hexastorm/exposure/csrc/vector_core.c",
        "src/hexastorm/exposure/csrc/_vector_ext.c",
    ],
    include_dirs=["src/hexastorm/exposure/csrc"],
    extra_compile_args=extra_compile_args,
    optional=True,
)

setup(ext_modules=[vector_core_ext])
