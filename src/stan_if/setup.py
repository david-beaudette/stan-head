from setuptools import setup

package_name = 'stan_if'

setup(
    name=package_name,
    version='0.11.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/stan_if_widget.ui']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='david',
    author_email='beaudette.david@gmail.com',
    maintainer='david',
    maintainer_email='beaudette.david@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Contains interface elements to interact with Stan.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'resource/stan_if',
        ],
    },
)