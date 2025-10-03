# Mapping Benchmark Workflow

This workflow demonstrates how to benchmark mapping pipelines using **ROS1 bags** and **OpenDroneMap (ODM)**.  
It covers extracting frames from recorded video topics into geotag-ready images, then processing them in ODM to produce mapping products such as orthophotos, DEMs, and dense point clouds.

---

## 1. Extract Images from ROS1 Bag

We use the `extract_compressed_images.py` script to extract JPEG frames from a compressed image topic (e.g. `/eo/color/image_color/compressed`).

### Usage
```bash
python3 extract_compressed_images.py \
  --bag bags/2022-12-20-12-48-59.bag \
  --topic /eo/color/image_color/compressed \
  --outdir images \
  --start 100 \
  --end 300 \
  --rate 2.0
```

#### Arguments
- `--bag` : Path to `.bag` file (ROS1 format).  
- `--topic` : ROS topic containing `sensor_msgs/CompressedImage` (default: `/eo/color/image_color/compressed`).  
- `--outdir` : Output folder where JPEGs will be stored.  
- `--start` : Start time in seconds (relative to bag start).  
- `--end` : End time in seconds (relative to bag start).  
- `--rate` : Sampling rate in Hz (frames per second).  
- `--prefix` : Filename prefix for output images (default: `frame_`).  
- `--csv` : Output CSV (filename + ROS timestamp).  

Example output:
```
images/
  frame_000000.jpg
  frame_000001.jpg
  frame_000002.jpg
  ...
images/frames.csv
```

The `frames.csv` file can be used later for geotagging if GPS/IMU logs are available.

---

## 2. Prepare Data for ODM

ODM expects the following structure:
```
dataset_root/
└── project_name/
    └── images/
        ├── frame_000000.jpg
        ├── frame_000001.jpg
        └── ...
```

For example:
```bash
mkdir -p /datasets/project/images
cp images/*.jpg /datasets/project/images/
```

---

## 3. Run OpenDroneMap (ODM)

ODM processes the extracted frames into mapping products.

### Example (Docker)
```bash
docker run -ti --rm \
  -v /absolute/path/to/dataset_root:/datasets \
  opendronemap/odm \
  --project-path /datasets project
```

### Output Products
ODM will generate outputs in:
```
project/odm_orthophoto/odm_orthophoto.tif   # Orthomosaic
project/odm_dem/dsm.tif                     # Digital Surface Model
project/odm_dem/dtm.tif                     # Digital Terrain Model
project/odm_filterpoints/point_cloud.ply    # Dense point cloud
project/odm_mesh/odm_mesh.ply               # 3D mesh
project/opensfm/reconstruction.ply          # Sparse reconstruction
project/report.pdf                          # Processing report
```

---

## 4. Viewing Results

- **Orthophoto / DEM / DTM**: Open with [QGIS](https://qgis.org) or GDAL.  
- **Point Cloud / Mesh**: Open with [CloudCompare](https://www.cloudcompare.org/) or MeshLab.  
- **Processing Report**: View `report.pdf` for stats and pipeline info.

---

## 5. Notes & Tips

- Ensure good overlap in the source imagery (70–80% forward/side).  
- Use `--start` and `--end` to trim unneeded parts of the bag for faster runs.  
- Tune ODM options (e.g., `--orthophoto-resolution`, `--dsm`) for your benchmarks.  
- For repeatability, consider using a `Makefile` or scripts to automate the steps.  

---

## License
This workflow is for internal benchmarking and research.  
See [OpenDroneMap license](https://github.com/OpenDroneMap/ODM/blob/master/LICENSE) for ODM usage terms.  
