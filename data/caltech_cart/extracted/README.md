# Caltech RGB-T Dataset (Extracted)

This folder contains extracted portions of the [Caltech Aerial RGB-Thermal Dataset](https://data.caltech.edu/records/cks6g-ps927).

Because GitHub enforces a **2 GB per-file limit** in Git LFS, large dataset files (ROS bags, ZIP archives) are split into multiple smaller chunks before committing.

---

## 🔖 Chunking Strategy

- Any `.bag` or `.zip` larger than 2 GB is split into **1.9 GB chunks**.
- Split files follow the naming convention:
```
filename.ext.part-000
filename.ext.part-001
filename.ext.part-002
```

- Each `.part-*` file is tracked in **Git LFS**, so the dataset can be stored and shared without hitting the GitHub size limit.
- Helper scripts are included in the repo for consistent splitting and joining.

---

## 🔧 How to Split Large Files (with script)

Use the `split_large_file.sh` script provided in the repo:

```bash
# Usage: ./split_large_file.sh <file> [chunk_size_MB]
# Example: split a ROS bag into 1900 MB parts
./split_large_file.sh fmu_logs/2022-05-15-06-00-09.bag 1900
fmu_logs/2022-05-15-06-00-09.bag.part-000
fmu_logs/2022-05-15-06-00-09.bag.part-001
```

## 🔗 How to Join Split Files (with script)

Use the join_large_file.sh script:

# Usage: ./join_large_file.sh <first_part_file> [output_file]
# Example: reconstruct the original bag file
./join_large_file.sh fmu_logs/2022-05-15-06-00-09.bag.part-000