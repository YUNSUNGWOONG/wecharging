from setuptools import setup

package_name = "skyvolt_eval"

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
    description="Regression eval harness.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "docking_eval = skyvolt_eval.docking_eval:main",
        ],
    },
)
