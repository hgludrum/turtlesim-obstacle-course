from setuptools import find_packages, setup

package_name = 'obstacle_course'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ludrum',
    maintainer_email='ludvig.rummelhoff@hitachigymnasiet.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate = obstacle_course.obstacle_course_generator:main',
            'bot = obstacle_course.bot:main',
            'launch = obstacle_course.main:main'
        ],
    },
)
