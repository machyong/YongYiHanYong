from setuptools import find_packages, setup

package_name = 'my_service_pkg'

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
    maintainer='jw',
    maintainer_email='jw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'add_two_ints_server = my_service_pkg.add_two_ints_server:main',
            'add_two_ints_client = my_service_pkg.add_two_ints_client:main',
            'text_echo_server = my_service_pkg.text_echo_server:main',
            'text_echo_client = my_service_pkg.text_echo_client:main',
        ],
    },
)
