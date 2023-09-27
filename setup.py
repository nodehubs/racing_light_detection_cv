from setuptools import setup

package_name = 'racing_light_detection_cv'

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
    maintainer='nuo.wu',
    maintainer_email='nuo.wu@horzion.cc',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'racing_traffic_light_detection = racing_light_detection_cv.racing_traffic_light_detection:main',
        'racing_image_color_analysis = racing_light_detection_cv.racing_image_color_analysis:main',
    ],
},
)
