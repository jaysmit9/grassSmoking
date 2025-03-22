from setuptools import setup, find_packages

setup(
    name='realsense-wrapper',
    version='0.1.0',
    author='Your Name',
    author_email='your.email@example.com',
    description='A wrapper for RealSense camera to visualize distances in the field of view.',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'pyrealsense2',
        'opencv-python',
        'matplotlib'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)