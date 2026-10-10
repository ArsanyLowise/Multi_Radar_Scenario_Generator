# Multi_Radar_Scenario_Generator

MATLAB framework I built for my PhD research on accident prediction. It simulates driving scenarios with 6 radars and exports the data in a format that can be fed into sequence models.

## Demo

[ Watch the demo (main_display_view.mp4)](main_display_view.mp4)

![Demo]([https://github.com/user-attachments/assets/40cae582-d619-4f35-bae4-8d917f3b88f8](https://github.com/user-attachments/assets/1b563e30-4241-4ed0-a67e-a616887b2273))

## Why I built this

The goal of my research is to predict vehicle accidents a few seconds *before* they happen using a transformer model. The problem is that real-world accident data is rare and hard to label. So I built this simulator to generate as many labeled scenarios as I need.

The idea is simple: if a transformer can learn the patterns that precede accidents, it can warn the driver (or take over) in time. To get there, I need:

1. Realistic multi-radar data
2. Ground truth labels for every scenario
3. Enough variety to cover edge cases

This repo handles the data generation part. The transformer model itself is a separate project (coming later).

## What it does

The pipeline goes through four stages:

- **Scenario setup** — define roads, vehicles, and sensors
- **Radar simulation** — generate detections with noise, false alarms, occlusion
- **Signal processing** — ego-motion compensation, clutter removal, DBSCAN clustering
- **Tracking + export** — grid-based RFS tracker, save to `.mat` for ML

Here's a quick look at the architecture:

Scenario → Radar sim → Signal processing → Tracker → Data export



## Sensors

Six radars total, mounted around the ego vehicle:

| Position | Azimuth | Notes |
|----------|---------|-------|
| Front | 0° | Long range |
| Front-left | 75° | Corner |
| Front-right | -75° | Corner |
| Rear | 180° | Long range |
| Rear-left | 135° | Corner |
| Rear-right | -135° | Corner |

All simulated with 90° FOV, 100 m range, and realistic noise parameters.

## Getting started

You'll need MATLAB R2023b or later, plus:

- Automated Driving Toolbox
- Sensor Fusion and Tracking Toolbox
- Computer Vision Toolbox
- Statistics and Machine Learning Toolbox

Clone and run:

```matlab
git clone https://github.com/ArsanyLowise/Multi_Radar_Scenario_Generator.git
cd Multi_Radar_Scenario_Generator
addpath(genpath('src'));
main

## Usage

Basic run with default parameters:


main
```

To try a curved road with oncoming traffic:

```matlab
params = getCurrentOptimization();
params.roadCurvature = -10;
params.targetDirection = -1;
main
```

To generate a dataset for training (multiple scenarios at once):

```matlab
run('examples/generate_dataset.m')
```

Data ends up in `Results/Dataset/` as `.mat` files.

## Data format

Each time step produces a struct with these fields:

```matlab
Matrix.radarData.x              % detection X positions (m)
Matrix.radarData.y              % detection Y positions (m)
Matrix.radarData.v              % raw radial velocity
Matrix.radarData.v_compensated  % ego-compensated velocity
Matrix.radarData.rcs            % radar cross section
Matrix.radarData.x_nonstatic    % dynamic detections only
Matrix.scenarioData             % ego + target ground truth
```

For transformer training, I take sequences of these over 2–5 second windows.

## Project structure

```
src/
  core/         — scenario + sensor setup
  tracking/     — DBSCAN, ego-compensation, RFS tracker
  visualization/— all the plotting functions
  recording/    — video/data recording
  utils/        — helpers
examples/       — standalone runnable scripts
docs/           — methodology, architecture notes
config/         — parameter files
```

## Some implementation notes

A few things that took longer than expected:

- **Ego-motion compensation** is more subtle than it looks. I initially forgot to account for the cosine dependence on azimuth — everything looked static until I fixed it.
- **DBSCAN on radar data** doesn't work well with the default parameters. I had to tune epsilon separately in velocity and range, then merge clusters that had similar mean velocities.
- **Grid-based RFS tracking** is heavy. 200k particles per update is fine on desktop but slow on a laptop. I added a `UseGPU` flag but haven't tested it much.

## Status

What works:
- Straight and curved roads
- 1 target + ego vehicle
- Full tracking pipeline
- Video recording
- Data export for ML

What's still TODO:
- Intersection scenarios
- Pedestrians / cyclists
- Multiple simultaneous targets
- GPU acceleration (partially tested)
- Link to transformer training repo

## Acknowledgments

This project builds upon example code and documentation provided by MathWorks, specifically from the **Automated Driving Toolbox** and **Sensor Fusion and Tracking Toolbox**. The grid-based radar tracking display and associated helper functions were adapted from MathWorks examples.

Per the MathWorks license, the original helper files are not redistributed here. To run this code, users need a valid MATLAB license with the toolboxes listed above.

Many thanks to MathWorks for providing these examples and tools.

## Citation

If you use this in academic work, please cite:

```bibtex
@misc{lowise2026radar,
  title  = {Multi-Radar Scenario Generator},
  author = {Lowise, Arsany},
  year   = {2026},
  url    = {https://github.com/ArsanyLowise/Multi_Radar_Scenario_Generator}
}
```

## License

MIT. See [LICENSE](LICENSE).

## Contact

- GitHub: [@ArsanyLowise](https://github.com/ArsanyLowise)
- LinkedIn: https://www.linkedin.com/in/arsany-lowise-00759a2ba/    

