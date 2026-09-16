#!/usr/bin/env python3
# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# Fetches maintainer/license/version etc. from package.xml so they stay in
# one place; catkin_python_setup() in CMakeLists.txt invokes this at build
# time to install the graph_dashboard/ Python package into devel/.
setup_args = generate_distutils_setup(
    packages=["graph_dashboard"],
    package_dir={"": "."},
    # Web assets (index.html + vendored vis-network) ship inside the
    # Python package so server.py can locate them via __file__ regardless
    # of how the package was installed.
    package_data={"graph_dashboard": ["web/*"]},
)

setup(**setup_args)
