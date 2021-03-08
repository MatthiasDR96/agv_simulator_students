from setuptools import setup

setup(
    name='agv_simulator_students',
    version='1.0',
    packages=['src'],
    url='https://github.com/MatthiasDR96/agv_simulator_students.git',
    license='BSD',
    author='Matthias De Ryck',
    author_email='matthias.deryck@kuleuven.be',
    description='Python AGV simulator for research purposes.',
    install_requires=['numpy', 'matplotlib', 'simpy', 'ortools', 'configparser']
)
