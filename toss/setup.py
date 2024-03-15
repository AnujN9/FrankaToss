from setuptools import find_packages, setup

package_name = 'toss'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/toss.launch.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anujn',
    maintainer_email='natraj.anuj@gmail.com',
    description='Main package to throw objects',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tossing = toss.tossing:main'
        ],
    },
)
