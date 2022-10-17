# TriFinger workshop example package

## Installation

### Testing in simulation

To test your solution in simulation before submitting it (highly recommended), create a virtual environment
```bash
python3.8 -m venv trifinger_venv
```
activate it
```bash
. trifinger_venv/bin/activate
```
update pip
```bash
pip install -U pip
```
and install the trifinger example package
```bash
pip install -e ./trifinger_example
```

### Apptainer

If you want to 

i) view videos of the real-robot episodes, or
ii) add dependencies to the code running on the real robot,

then please install apptainer: https://github.com/apptainer/apptainer/releases

Creating videos of real-robot episodes requires the 'video_creation_image.sif' image.
Dependencies should be added to the 'workshop_image.def' image and a new images can then be build based on 'rrc2021.sif'.

## Usage

### Test locally in simulation

To test your solution in simulation run
```bash
python trifinger_example/scripts/run_robot.py --simulate
```

### Submit to the real robot cluster

To submit to the real robot cluster, make sure you have set up the `roboch.json`
correctly and copied it to your user account via
```bash
scp roboch.json username@robots.real-robot-challenge.com:
```
You can then run the bash script to automatically submit, download and create the video:
```bash
bash scripts/submit_and_download.sh /path/to/output/dir/ 1 /path/to/video_creation_image.sif
```

Alternatively, you can also login directly
```bash
ssh annoyediguana@robots.real-robot-challenge.com`
```
which allows you to `check` your configuration, check the `status` of your job,
and `cancel` a job (as long as it's not already running).



