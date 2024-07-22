from setuptools import find_packages, setup

package_name = 'parc24_agribot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RS2LAB',
    maintainer_email='rs2lab@unicv.cv',
    description='Smart UniCV PARC 2024 Engineers League Agribot Agent',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = parc24_agribot.agent:main'
        ],
    },
)
