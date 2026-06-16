This folder contains all the code required to record data for EMG from myoarmband for classification.
emg_experiment.py: opens a GUI that currently involves collecting data for mild and full extension/flexion classes.
This data would be used to train the model (it could be used to validate offline accuracy)
For future it could be modified to remove mild movements data collection. It currently saves every session in emg_sessions folder under the Participant name. Under each Participant folder, you have folders of each session labeled like "YYYYMMDD_HHMMSS" so year,month,day, hour,minute,second. Under each session folder you would have csv files of the recordings.
You have:
-samples.csv: recordings (raw data) over time. It is organized like this:
    timestamp_us;participant;state;active_label_id;active_label_name;stream_index;ch0;ch1;ch2;ch3;ch4;ch5;ch6;ch7
-events.csv: All labels with timestamps of markers and classes. It is organized like this:
    timestamp_us;participant;event;detail

This folder also includes media folder which includes images used by emg_experiment.py to guide the participants in the GUI

Additionally there is a jupyter notebook: Data_analysis.ipynb. This notebook is used a workspace to do whatever analysis related to the collected data. It is recommended to start a new fresh notebook and do whatever analysis is needed for specific use case. Notebook contains examples of what can be done.