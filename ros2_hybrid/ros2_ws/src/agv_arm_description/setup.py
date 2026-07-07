from glob import glob
from setuptools import setup

package_name = "agv_arm_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGV Project",
    maintainer_email="vuducdu2k5@gmail.com",
    description="Configurable dual-arm AGV robot description.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "generate_agv_arm_urdf = agv_arm_description.generate_urdf:main",
        ],
    },
)
