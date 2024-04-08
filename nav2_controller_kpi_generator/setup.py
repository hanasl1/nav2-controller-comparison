from setuptools import find_packages, setup

package_name = 'nav2_controller_kpi_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanaslipogor',
    maintainer_email='hana.slipogor@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kpi_generator = nav2_controller_kpi_generator.kpi_generator:main',
            'goal_checking = nav2_controller_kpi_generator.goal_checking:main',
            'cmd_vel_formatter = nav2_controller_kpi_generator.cmd_vel_formatter:main'

        ],
    },
)
