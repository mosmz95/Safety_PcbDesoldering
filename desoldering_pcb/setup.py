#!/usr/bin/env python

from setuptools import setup

setup(
    name='desoldering_pcb',
    version='0.0.0',
    packages=['desoldering_pcb'],
    package_dir={'': 'src'},
    install_requires=[
        'numpy',
        'rospy',
        'mediapipe',
        'tkinter',
        'rtde',
        'opencv-python',
        'pyrealsense2'
    ]
    
)