#!/usr/bin/python3

import os
import sys


def _prepend_source_path():
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src_path = os.path.join(pkg_root, "src")
    if os.path.isdir(src_path) and src_path not in sys.path:
        sys.path.insert(0, src_path)


_prepend_source_path()

from airgrasp_minco_irl.fd_optimizer import main


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)

