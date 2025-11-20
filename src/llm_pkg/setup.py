from setuptools import find_packages, setup

package_name = 'llm_pkg'

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
    maintainer='up',
    maintainer_email='up@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_trigger_server = llm_pkg.llm_trigger_server:main',
            'llm_trigger_client = llm_pkg.llm_trigger_client:main',
        ],
    },
)
