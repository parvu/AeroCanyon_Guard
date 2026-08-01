from setuptools import setup

package_name = 'px4_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Conf. Petrisor Parvu',
    maintainer_email='petrisor.parvu@upb.ro',
    description='Keyboard teleop for PX4 SITL over the uXRCE-DDS bridge (offboard velocity control)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = px4_teleop.teleop_keyboard:main',
        ],
    },
)
