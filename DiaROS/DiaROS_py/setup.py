from setuptools import setup

setup(
    name='diaros',
    version='0.0.1',
    packages=['diaros'],
    package_data={
        'diaros': ['prompts/*'],
    },
    include_package_data=True,
)
