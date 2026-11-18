from setuptools import setup, find_packages

setup(
    name="motion-client",
    version="0.2.0",
    description="Python Host Client SDK and CLI for stepper-motion-core and High-Detail 3D Scanner",
    author="stepper-motion-core team",
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    install_requires=[
        "click>=8.0.0",
    ],
    extras_require={
        "serial": ["pyserial>=3.5"],
        "websocket": ["websockets>=11.0.0"],
        "all": ["pyserial>=3.5", "websockets>=11.0.0", "numpy>=1.20.0"],
    },
    entry_points={
        "console_scripts": [
            "mc-cli=cli.mc_cli:cli",
        ],
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
)
