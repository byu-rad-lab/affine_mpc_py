import argparse
import os
from pathlib import Path
import re
import tempfile
import sys

# assert False

parser = argparse.ArgumentParser(description="")
# parser.add_argument(
#     "dir",
#     nargs="?",
#     type=str,
#     default=tmp_dir,
#     help="Directory in which to create generated 'stubs' dir",
# )
parser.add_argument(
    "dir",
    nargs="?",
    type=str,
    default="",
    help="Directory in which to load affine_mpc",
)
args = parser.parse_args()
if args.dir != "":
    sys.path.insert(0, args.dir)


try:
    import pybind11_stubgen
except ImportError:
    print("pybind11_stubgen not found")
    # exit(1)

try:
    import affine_mpc as ampc
except ImportError:
    print("Could not find affine_mpc!")
    raise FileNotFoundError(f"{os.listdir(args.dir)}")
    raise FileNotFoundError(f"{os.listdir(args.dir+'/affine_mpc')}")

# script_dir = Path(__file__).parent.resolve()
tmp_dir = tempfile.gettempdir()
# stub_dir = os.path.join(args.dir, "stubs")
stub_dir = os.path.join(tmp_dir, "stubs")
# stub_dir = re.sub("\\", "/", stub_dir)
print("Stub Dir:", stub_dir)

########

pybind11_stubgen.main(
    [
        "-o",
        stub_dir,
        "affine_mpc",
        "--numpy-array-use-type-var",
        # "--numpy-array-wrap-with-annotated",
    ]
)

stub_file = os.path.join(stub_dir, "affine_mpc", "_bindings.pyi")

with open(stub_file, "r", encoding="utf-8") as f:
    content = f.read()
content = re.sub(r"from __future__ import annotations", "", content)
# content = re.sub(
#     r"import numpy",
#     "import numpy as np\nfrom numpy.typing import NDArray",
#     content,
#     count=1,
# )
content = re.sub("typing.SupportsInt", "int", content)
content = re.sub("typing.SupportsFloat", "float", content)
content = re.sub(r"M = .*\nN = .*", "", content)
content = re.sub(
    r"tuple\[M,[^,]*, numpy\.dtype\[numpy\.float64\]", "numpy.float64", content
)
# content = re.sub(r"numpy\.ndarray", "NDArray", content)
content = re.sub(r" = \.\.\.", " = numpy.empty(0)", content)
content = re.sub(r"OSQPSettings = None", "OSQPSettings|None = None", content)

try:
    import black

    formatted = black.format_file_contents(
        content, fast=True, mode=black.FileMode(is_pyi=True)
    )
except ImportError:
    print("black not found!")
    formatted = content

with open(stub_file, "w", encoding="utf-8") as f:
    f.write(formatted)

assert os.path.exists(stub_file)
assert os.path.exist("/tmp/stubs/affine_mpc")
