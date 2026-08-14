from glob import glob
from setuptools import setup

package_name = 'aerocanyon'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/data', glob('data/*')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'torch', 'pandas', 'matplotlib'],
    zip_safe=True,
    maintainer='Conf. Petrisor Parvu',
    maintainer_email='petrisor.parvu@upb.ro',
    description='Urban canyon FO-PINN/CBF simulation for PX4 tilt-rotor VTOL',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wind_field_node = aerocanyon.wind_field_node:main',
            'controller_node = aerocanyon.controller_node:main',
            'fo_pinn_node = aerocanyon.fo_pinn_node:main',
            'trial_logger = aerocanyon.trial_logger:main',
        ],
    },
)
