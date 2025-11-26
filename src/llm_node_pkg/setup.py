from setuptools import find_packages, setup

package_name = 'llm_node_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # .env 파일을 package data에 포함
        ('share/' + package_name + '/resource', ['resource/.env']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hun',
    maintainer_email='jihun3333@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trigger_server = llm_node_pkg.trigger_server:main',
            'trigger_client = llm_node_pkg.trigger_client:main'
        ],
    },
)
