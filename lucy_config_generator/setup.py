from setuptools import find_packages, setup

package_name = "lucy_config_generator"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["templates/*.j2"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "Jinja2", "PyYAML"],
    zip_safe=True,
    maintainer="Sentience Robotics Team",
    maintainer_email="contact@sentience-robotics.fr",
    description="Generate RP2040 and ros2_control artifacts from thais_urdf hardware YAML (#96).",
    license="GPL-3.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "generate_config = lucy_config_generator.process:main",
        ],
    },
)
