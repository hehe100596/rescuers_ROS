from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['xbacov04', 'xbacov04.helpers'],
    package_dir={
        'xbacov04': 'src/xbacov04',
        'xbacov04.helpers': 'src/xbacov04/helpers'
        })

setup(**setup_args)
