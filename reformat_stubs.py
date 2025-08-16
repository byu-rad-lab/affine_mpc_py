import argparse
import re

import os
import re
import tempfile


tmp_dir = tempfile.gettempdir()
# parser = argparse.ArgumentParser(description="")
# parser.add_argument(
#     "dir",
#     nargs="?",
#     type=str,
#     default=tmp_dir,
#     help="Directory in which to create generated 'stubs' dir",
# )
# args = parser.parse_args()

stub_dir = os.path.join(tmp_dir, "stubs")
stub_file = os.path.join(stub_dir, "affine_mpc", "_bindings.pyi")

with open(stub_file, "r", encoding="utf-8") as f:
    content = f.read()

assert content != ""

content = re.sub(r"from __future__ import annotations", "", content)
# content = re.sub(
#     r"import numpy",
#     "import numpy as np\nfrom numpy.typing import NDArray",
#     content,
#     count=1,
# )
content = re.sub("typing.SupportsInt", "int", content)
content = re.sub("typing.SupportsFloat", "float", content)
# content = re.sub(r"M = .*\nN = .*", "", content)
content = re.sub(
    r"tuple\[M,[^,]*, numpy\.dtype\[numpy\.float64\]", "np.float64", content
)
# content = re.sub(r"numpy\.ndarray", "NDArray", content)
content = re.sub(r" = \.\.\.", " = numpy.empty(0)", content)
content = re.sub(r"OSQPSettings = None", "OSQPSettings|None = None", content)

with open(stub_file, "w", encoding="utf-8") as f:
    f.write(content)
