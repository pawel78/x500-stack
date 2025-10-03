#!/usr/bin/env python3
import os
import subprocess
import zipfile

# Caltech Labeled RGB-Thermal Dataset link
URL_IMAGES = [
    "https://data.caltech.edu/records/cks6g-ps927/files/labeled_rgbt_pairs.zip?download=1"
]
URL_FLIGHT_LOGS = [
    "https://sdsc.osn.xsede.org/ini210004tommorrell/cks6g-ps927/data/"
]

def get_repo_root():
    """Ask git for the top-level repo path"""
    try:
        repo_root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], text=True
        ).strip()
        return repo_root
    except subprocess.CalledProcessError:
        raise RuntimeError("Not inside a git repository!")

REPO_ROOT = get_repo_root()
OUTPUT_DIR = os.path.join(REPO_ROOT, "data", "caltech_cart")

def download_file(url, out_dir):
    filename = url.split("/")[-1].split("?")[0]
    local_path = os.path.join(out_dir, filename)
    if os.path.exists(local_path):
        print(f"Skipping download, {local_path} already exists.")
        return local_path
    os.makedirs(out_dir, exist_ok=True)
    print(f"Downloading {url} -> {local_path}")
    subprocess.run(["curl", "-L", "-o", local_path, url], check=True)
    return local_path

def unzip_file(zip_path):
    extract_dir = os.path.splitext(zip_path)[0]
    if os.path.exists(extract_dir):
        print(f"Skipping unzip, {extract_dir} already exists.")
        return extract_dir
    print(f"Unzipping {zip_path} -> {extract_dir}")
    with zipfile.ZipFile(zip_path, 'r') as zf:
        zf.extractall(extract_dir)
    return extract_dir

def download_fmu_data(url, out_dir):
    pass # Placeholder for future implementation

if __name__ == "__main__":
    for url in URL_IMAGES:
        try:
            zip_path = download_file(url, OUTPUT_DIR)
            unzip_file(zip_path)
        except Exception as e:
            print(f"Error handling {url}: {e}")




