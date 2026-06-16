This folder is used to train and deploy the Deep Learning model.

It contains:
1. data folder: Includes all code and scripts to preprocess, prepare and clean the data, as well to create the dataset and function utilities to handle it
2. taining_utils: for real time plotting during training and folders handling
3. Data_testing: for checking how your dataset looks like and testing difeferent preprocessing parameters

e.g. how to create a dataset


from data.EMG_dataset import EMGDataset
import torchvision
from torchvision import transforms


train_dataset = EMGDataset(
    transform = transforms.Compose([
        transforms.ToTensor()
    ]),
    root="compiled_data", 
    filename = "train_data.npz",
    participant_path = "C:\\Users\\Marwa\\Documents\\GitHub\\myo-armband-ble\\record\\emg_sessions\\train",
    force_compile = False,
)

val_dataset = EMGDataset(
    transform = transforms.Compose([
        transforms.ToTensor()
    ]),
    root="compiled_data", 
    filename = "val_data.npz",
    participant_path = "C:\\Users\\Marwa\\Documents\\GitHub\\myo-armband-ble\\record\\emg_sessions\\val",
    force_compile = True,
)


Worth noting that you can create a folder called train, val or test and put all the sessions you want manually there and use it when creating new dataset as participant_path

So you need change dataset name , filename and participant_path according to what data you want and if you want to redo preprocessing (in case you changed parameters or you added new sessions for example) simply set force_compile = True

4. networks.py: This is where you put your AI models. Currently the best tested AI model is called EMGModel. You can create whaetever model you want here.

e.g. to create a model based on EMGModel (the following example uses the bets found hyperparameters to date)
In the example device means if you want to use gpu or cpu (keep as is)
--------------------------------------------------------------------------------
from networks import EMGModel
import torch
# Hyperparameters
hparams = {'D': 16, # architecture
 'F1': 128, # architecture
 'F2': 128, # architecture
 'channels': 8,
 'dropout': 0.138, # training
 'early_stopping': {'burn_in': 4,
                    'cooldown': 4,
                    'min_delta': 0.00047912070773434816,
                    'mode': 'min',
                    'monitor': 'val_loss',
                    'patience': 15,
                    'restore_best': True},
 'extra_blocks': 1, # architecture
 'extra_dilation': 1, # architecture
 'extra_dropout': 0.138, # training
 'extra_kernel': 7, # architecture
 'kernel_length': 24, # architecture
 'num_classes': 9, # In case you want to remove mild classes it would become 5 classes only
 'optimization': {'batch_size': 24, # training
                  'criterion': 'nn.CrossEntropyLoss',
                  'epochs': 100, # training
                  'grad_clip': 2.0, # training
                  'lr': 0.00011396995431147184, # training
                  'optimizer': 'optim.AdamW'},
 'pool1_time': 3, # architecture
 'pool2_time': 8, # architecture
 'scheduler': {'factor': 0.649590522222718,
               'min_lr': 3.0570412165705003e-07,
               'mode': 'min',
               'monitor': 'val_loss',
               'name': 'torch.optim.lr_scheduler.ReduceLROnPlateau',
               'patience': 6,
               'step_mode': 'plateau',
               'verbose': True}}


# Check if CUDA is available and assign device
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# Create an instance of the model
model = EMGModel(hparams).to(device)
---------------------------------------------------------------------------

After creating the datasets and the model, we need to load the data to create batches to train several samples in parallel. To do this you can also implement an optional sampler to make sure that you have equal samples from all classes. 

e.g.
----------------------------------------------------------------------------
import numpy as np
import torch
from torch.utils.data import DataLoader
from torch.utils.data.sampler import WeightedRandomSampler

def make_weighted_sampler(dataset, num_classes=9, smoothing=0.0):
    """
    Builds a WeightedRandomSampler so each class is drawn ~uniformly.
    `smoothing` in [0, 1): 0 = strict inverse-frequency; closer to 1 flattens weights.
    """
    # get labels as a 1D numpy array
    if hasattr(dataset, "y"):
        y = np.asarray(dataset.y)
    elif hasattr(dataset, "labels"):
        y = np.asarray(dataset.labels)
    else:
        raise AttributeError("Dataset must expose labels via `y` or `labels`.")

    counts = np.bincount(y, minlength=num_classes).astype(np.float64)
    counts[counts == 0] = 1.0  # avoid div-by-zero if a class is missing

    inv = 1.0 / counts
    # optional smoothing to avoid extreme weights when a class is super-rare
    w_per_class = (1 - smoothing) * inv / inv.sum() * num_classes + smoothing * 1.0

    weights = w_per_class[y]  # per-sample weight
    weights = torch.as_tensor(weights, dtype=torch.double)

    sampler = WeightedRandomSampler(
        weights=weights,
        num_samples=len(weights),    # one "epoch" roughly same number of samples
        replacement=True
    )
    return sampler

# --- usage ---
batch_size = hparams['optimization']['batch_size']
train_sampler = make_weighted_sampler(train_dataset, num_classes=9)
train_loader  = DataLoader(train_dataset, batch_size=batch_size, sampler=train_sampler, shuffle=False, drop_last=True)

# keep validation/test natural (no balancing there):
val_loader    = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, drop_last=True)
test_loader   = DataLoader(test_dataset, batch_size=batch_size, shuffle=False, drop_last=True)
--------------------------------------------------------------------------------
NEVER USE TEST DATA TO CHANGE THE PARAMETERS. Use the val data to check accuracy and change the parameters to get the maximum validation accuracy or minimum loss

5. Next you train using train_model from training.py
e.g.
--------------------------------------------------------------------------------
from training import train_model
train_model(model, train_loader, val_loader, device, hparams, plot_losses = True)
---------------------------------------------------------------------------------

Training_example.ipynb: a jupyter notebook that implements random seearch followed by hyperparameter optimization algorithm. What it does is that it randomly tries different hyperparameter combination. And best on the results it starts picking the best next combination to try using a library called optuna.

If you want to know how it works you can dive into the notebook and check optuna documentation.

When you train a model you get 2 folders 
models: a folder containing other folders with the name of all the models that have been trained and under every model folder you have all the trained instances. This folder will be used by optuna to check all of your results and suggest the next best hyperparameter combination based on it

You can refer to the thesis and the powerpoint presentation for more info about how the training process works.

6. EMG_RUN_main.py: The real time classification algorithm  with preprocessing and postprocessing (for more info check thesis) make sure the GUI that opens is always in the front (use WindowTop app to keep it there)

Note: the algorithm currently classifies the data into 9 classes and then makes all mild classes into rest and only classifies moevement if it is full movement (extension/flexion)

For future work the algorithm could be changed so that there only 5 classes in the first place.

to run it navigate to train folder using the commmand in the terminl:
cd train
and then:
python EMG_RUN_main.py