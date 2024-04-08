from setuptools import setup, Extension
from Cython.Build import cythonize

extensions = [
    Extension("gain_evaluator", ["gain_evaluator.pyx"],
              language="c++",
              include_dirs=["~/voxblox/devel/include", "~/traj_gen/devel/include"],
              library_dirs=["~/voxblox/devel/lib", "~/traj_gen/devel/lib"],
              libraries=["voxblox", "trajectory_generation"])
]

setup(
    ext_modules = cythonize(extensions)
)
