from setuptools import setup

package_name = 'terrain_nav_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Benchmark default Nav2 vs terrain-aware navigation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'run_benchmark = terrain_nav_benchmark.run_benchmark:main',
        ],
    },
)
