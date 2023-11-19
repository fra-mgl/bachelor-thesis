from setuptools import setup

package_name = 'doretta'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, "doretta/controllers", "doretta/environment", "doretta/models", "doretta/utils"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover',
    maintainer_email='trackedRobot@mail.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_stanley = doretta.sim_stanley:main',
            'sim_lyapunov = doretta.sim_lyapunov:main',
            'real_stanley.yaml = doretta.real_stanley.yaml:main',
            'real_lyapunov = doretta.real_lyapunov:main',
            'bag = doretta.bag_node:main'
        ],
    },
)
