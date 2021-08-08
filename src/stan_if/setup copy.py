from setuptools import setup

package_name = 'stan_if'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['scripts/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/stan_if_widget.ui']),
        ('share/' + package_name, 
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='beaudette.david@gmail.com',
    description='Contains interface elements to interact with Stan.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['resource/stan_if'],
    },
)
