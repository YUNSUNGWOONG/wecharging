from setuptools import setup

package_name = "skyvolt_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Skyvolt RE",
    maintainer_email="re@skyvolt.example",
    description="Track-arclength KF + two-stage docking speed policy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localizer_node = skyvolt_localization.localizer_node:main",
            "speed_policy_node = skyvolt_localization.speed_policy_node:main",
        ],
    },
)
