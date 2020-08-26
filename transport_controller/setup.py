from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['transport_controller'],
    package_dir={'': 'src'},
    skripts=["scripts/transport_state_machine_node"]
)

setup(**setup_args)
