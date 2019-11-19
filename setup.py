from setuptools import setup

setup(
    name='agv_simulator',
    version='1.0',
    packages=['src'],
    url='https://github.com/MatthiasDR96/agv_simulator.git',
    license='BSD',
    author='Matthias De Ryck',
    author_email='matthias.deryck@kuleuven.be',
    description='Python AGV simulator for research purposes.',
    install_requires=['math', 'numpy', 'matplotlib', 'simpy', 'random', 'ortools', 'configparser']
)
