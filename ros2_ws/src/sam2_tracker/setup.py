from setuptools import find_packages, setup

package_name = 'sam2_tracker'

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
    maintainer='ecervera',
    maintainer_email='ecervera@uji.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sam2_tracker = sam2_tracker.sam2_tracker:main',
            'polygon_to_array = sam2_tracker.polygon_to_array:main',
        ],
    },
)
