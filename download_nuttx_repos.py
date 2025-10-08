import yaml
import subprocess
import os

REPO_FILE = "nuttx.repos"
TARGET_DIR = "joint_unit_mcu_nuttx"

os.makedirs(TARGET_DIR, exist_ok=True)

with open(REPO_FILE, "r") as f:
    data = yaml.safe_load(f)

repos = data.get("repositories", {})

for name, info in repos.items():
    url = info.get("url")
    version = info.get("version", "master")
    repo_path = os.path.join(TARGET_DIR, name)
    print(f"Cloning {name} from {url} (version: {version}) into {repo_path}...")
    subprocess.run([
        "git", "clone", "--branch", version, "--depth", "1", url, repo_path
    ], check=True)