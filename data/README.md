# data

Generated maps, logs, results, datasets, and recordings live here.

- `maps/raw`: baseline or unprocessed saved maps.
- `maps/before_strip`: semantic mapping maps before person-region stripping.
- `maps/after_strip`: maps after strip post-processing.
- `maps/semantic`: semantic overlays and future semantic map products.
- `logs`: runtime logs from scripts and experiments.
- `results`: metrics and experiment reports.
- `datasets`: reusable offline inputs.
- `recordings`: rosbag or media recordings.

Large generated files are ignored by git; each directory keeps only `.gitkeep`.
