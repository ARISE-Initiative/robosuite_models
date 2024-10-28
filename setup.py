# read the contents of your README file
from os import path

from setuptools import find_packages, setup

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, "README.md"), encoding="utf-8") as f:
    lines = f.readlines()

# remove images from README
lines = [x for x in lines if ".png" not in x]
long_description = "".join(lines)

setup(
    name="robosuite_models",
    packages=[package for package in find_packages() if package.startswith("robosuite_models")],
    install_requires=["mujoco>=3.2.0", "robosuite>=1.5.0"],
    eager_resources=["*"],
    include_package_data=True,
    python_requires=">=3",
    description="robosuite models: collection of robot models for robosuite usage.",
    author="Yifeng Zhu",
    url="https://github.com/ARISE-Initiative/robosuite_models",
    author_email="yifeng.zhu@utexas.edu",
    version="1.0.0",
    long_description=long_description,
    long_description_content_type="text/markdown",
)
