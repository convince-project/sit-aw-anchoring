from setuptools import find_packages, setup

package_name = 'get_pybullet_data'

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
    maintainer='jb282126',
    maintainer_email='jeremie.bodin@cea.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_data = get_pybullet_data.get_pybullet_data:main',
            'client = get_pybullet_data.client:main',
        ],
    },
)
