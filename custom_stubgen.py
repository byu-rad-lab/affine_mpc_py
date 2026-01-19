import argparse
import os
from pathlib import Path
import re
import pybind11_stubgen
import affine_mpc as ampc


parser = argparse.ArgumentParser(description="")
parser.add_argument(
    "dir",
    type=str,
    help="Directory in which to load affine_mpc",
)
args = parser.parse_args()

# try:
#     import affine_mpc as ampc
# except ImportError:
#     print("Could not find affine_mpc!")
#     raise FileNotFoundError(f"{os.listdir(args.dir)}")
#     raise FileNotFoundError(f"{os.listdir(args.dir+'/affine_mpc')}")

pkg = "affine_mpc"
out_dir = Path(args.dir).resolve()
# print("Out Dir:", out_dir)
stub_dir = out_dir / "stubs"
# print("Stub Dir:", stub_dir)


pybind11_stubgen.main(
    [
        "-o",
        stub_dir.as_posix(),
        pkg,
        "--numpy-array-use-type-var",
        # "--numpy-array-wrap-with-annotated",
    ]
)

stub_file = stub_dir / pkg / "_bindings.pyi"

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

# assert os.path.exists(stub_file)
