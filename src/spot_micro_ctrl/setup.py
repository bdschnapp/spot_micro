from setuptools import setup

package_name = 'spot_micro_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ben Schnapp',
    maintainer_email='bdschnapp@gmail.com',
    description='A package for controlling the movement of the spot micro robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = spot_micro_ctrl.spot_micro_ctrl:main'
        ],
    },
)
