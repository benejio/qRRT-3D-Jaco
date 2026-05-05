from setuptools import setup

package_name = "qrrt_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jesse",
    maintainer_email="jesse@example.com",
    description="Joint-space qRRT experiments with MoveIt collision checking.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "add_box_obstacle = qrrt_planner.add_box_obstacle:main",
            "run_grid_astar = qrrt_planner.run_grid_astar:main",
            "benchmark_grid_astar = qrrt_planner.benchmark_grid_astar:main",
            "run_qrrt_grid = qrrt_planner.run_qrrt_grid:main",
            "run_crrt_grid = qrrt_planner.run_crrt_grid:main",
        ],
    },
)   
