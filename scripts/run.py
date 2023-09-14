#!/usr/bin/python3
import subprocess
from pathlib import Path
import os
import shutil
import time
import re
from concurrent.futures import ThreadPoolExecutor, as_completed

file_path = os.path.realpath(__file__)
base_dir = Path(file_path).parent
RV_ROOT = Path(os.environ["RV_ROOT"])
veer_exec = RV_ROOT / "obj_dir" / "Vtb_top"
tmpdir = base_dir / "tmp_run"

def run_test(test, is_intr):
    print("Running", test.name)
    tname = test.name
    if is_intr:
        tname += "_intr"
    mydir = tmpdir / tname
    mydir.mkdir()

    shutil.copy(test, mydir/"program.hex")
    tstart = time.time()
    res = subprocess.run([veer_exec, "+hash_seed=0", "+way_seed=0"], cwd=mydir, stdout=subprocess.PIPE, universal_newlines=True)
    elapsed = time.time() - tstart
    with open(mydir/"stdout.log", "w") as f:
        f.write(res.stdout)
    print(f"Test {tname} took {elapsed}s", end=" ")
    if res.returncode != 0:
        print("FAILED")
        return res.returncode

    if re.search(r"DONE!", res.stdout):
        cycles = re.search(r", mcycle = (\d+)", res.stdout)
        print("PASS, cycles=", cycles.groups(1))



# Load the tests and execute them

intrusive_dir = base_dir / "eembc_intrusive_crc"
non_intrusive_dir = base_dir / "eembc_non_intrusive_crc"

intr_tests = intrusive_dir.glob("*.hex")
non_intr_tests = non_intrusive_dir.glob("*.hex")

if tmpdir.is_dir():
    shutil.rmtree(tmpdir)
tmpdir.mkdir()

print("Running Non-Intrusive")

gtstart = time.time()
with ThreadPoolExecutor(max_workers=7) as executor:
    results = []
    for test in intr_tests:
        results.append(executor.submit(run_test, test, True))
    for test in non_intr_tests:
        results.append(executor.submit(run_test, test))

    for future in as_completed(results):
        pass
gelapsed = time.time()-gtstart
print("Total:", gelapsed, "s")
