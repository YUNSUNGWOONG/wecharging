from setuptools import setup

package_name = "skyvolt_fleet"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Skyvolt RE",
    maintainer_email="re@skyvolt.example",
    description="Greedy fleet scheduler + segment reservation + REST API.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fleet_manager_node = skyvolt_fleet.fleet_manager_node:main",
            "fleet_api = skyvolt_fleet.api_server:main",
        ],
    },
)
