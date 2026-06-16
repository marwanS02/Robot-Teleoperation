This folder contains 3 python scripts:
1. compile_utils.py: all the preprocessing and windowing (check thesis for extra knowledge on each preprocessing step)
2. base_dataset.py: abstract class to call compile_utils.py and handle the compilation
3. EMG_dataset.py: this is the class to create the dataset object.
    Useful utils from this script:
    len(dataset): gives you how many samples in the dataset
    dataset[i]: give me the ith sample in the dataset
    dataset.visualize(i): give me a visualization of the preprocessed ith sample 
    dataset.visualize_raw(i): give me a visualization of the ith raw sample

e.g. how to create a dataset
    train_dataset = EMGDataset(
    transform = transforms.Compose([
        transforms.ToTensor()
    ]),
    root="compiled_data", 
    filename = "train_data.npz",
    participant_path = "C:\\Users\\Marwa\\Documents\\GitHub\\myo-armband-ble\\record\\emg_sessions\\train",
    force_compile = False,
)
    