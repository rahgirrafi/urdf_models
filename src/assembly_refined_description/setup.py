from setuptools import setup
import os
from glob import glob

package_name = 'assembly_refined_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],



    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RahgirRafi',
    maintainer_email='rahgirrafi@gmail.com',
    description='The ' + package_name + ' package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'openCV_img_processing = assembly_refined_description.openCV_img_processing:main',
    ],
},
)
