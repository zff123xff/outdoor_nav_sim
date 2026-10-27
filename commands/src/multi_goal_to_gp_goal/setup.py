from setuptools import find_packages, setup

package_name = "multi_goal_to_gp_goal"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/multi_goal_to_gp_goal.launch.py"]),
        (f"share/{package_name}/config", ["config/params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description=(
        "Bridge msg_set_msgs/MultiGoal -> sensor_msgs/NavSatFix on /gp_goal"
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"multi_goal_to_gp_goal = {package_name}.multi_goal_to_gp_goal:main",
        ],
    },
)