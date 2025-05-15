from setuptools import find_packages, setup

package_name = 'happy_tts'

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
    maintainer='daniil',
    maintainer_email='daniil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts = happy_tts.text_to_speech:main',
            'launch_tts_server = happy_tts.launch_tts_server:main'
        ],
    },
)
