#!/usr/bin/env python3
"""
Setup script for robotiq_2f85_modbus_tcp package.
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="robotiq-2f85-modbus-tcp",
    version="1.0.0",
    author="Vibe Code",
    author_email="",
    description="Modbus-TCP controller for Robotiq 2F-85 and 2F-140 grippers",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/robotiq/robotiq-ros",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Human Machine Interfaces",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Hardware :: Hardware Drivers",
    ],
    python_requires=">=3.8",
    install_requires=[
        "pymodbus>=3.0.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "black>=23.0.0",
            "flake8>=6.0.0",
            "mypy>=1.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "robotiq-test=robotiq_2f85_modbus_tcp.cli:test_connection",
            "robotiq-demo=robotiq_2f85_modbus_tcp.cli:demo",
        ],
    },
)
