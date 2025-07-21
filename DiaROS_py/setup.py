from setuptools import setup, find_packages

setup(
    name='diaros',
    version='0.0.1',
    packages=find_packages(),
    package_data={
        'diaros': ['prompts/*.txt'],
    },
    include_package_data=True,
)
