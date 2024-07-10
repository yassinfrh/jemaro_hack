from setuptools import find_packages, setup

package_name = 'hack'

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
    maintainer='student-en',
    maintainer_email='yassin.frh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obj_detection = hack.obj_detection:main',
            'shapes = hack.shapes:main',
            'logic = hack.logic:main',
        ],
    },
)
