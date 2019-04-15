from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name='esiaf_wav_player',
    version='0.0.1',
    description='A component that feeds the esiaf_ros pipeline audio from a wav file',
    url='---none---',
    author='rfeldhans',
    author_email='rfeldh@gmail.com',
    license='---none---',
    packages=['esiaf_wav_player']

)

setup(**setup_args)