import copy

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
import tqdm
from sklearn.model_selection import train_test_split
from sklearn.datasets import fetch_california_housing
from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error

# Read data
data = fetch_california_housing()
X, y = data.data, data.target

# train-test split for model evaluation
#X_train, X_test, y_train, y_test = train_test_split(X, y, train_size=0.7, shuffle=True)

input_file = "salidaFORMC.txt"
X_train, X_test, X_valid, y_train = np.loadtxt(input_file, unpack=True)

X_train = (X_train - 3.8722e+03)/2.1336e+03
X_test = (X_test - 3.8722e+03)/2.1336e+03
X_valid = (X_valid - 3.8722e+03)/2.1336e+03

y_test = y_train

# Convert to 2D PyTorch tensors

#print(X_train)
#print(X_train.shape)

#print(y_train)
#print(y_train.shape)


X_train = torch.tensor(X_train, dtype=torch.float32).reshape(-1,1)
y_train = torch.tensor(y_train, dtype=torch.float32).reshape(-1, 1)
X_test = torch.tensor(X_test, dtype=torch.float32).reshape(-1,1)
y_test = torch.tensor(y_test, dtype=torch.float32).reshape(-1, 1)

def training(model,number):
    loss_fn = nn.MSELoss()  # mean square error
    optimizer = optim.Adam(model.parameters(), lr=0.0001)

    n_epochs = 100  # number of epochs to run
    batch_size = 10  # size of each batch
    batch_start = torch.arange(0, len(X_train), batch_size)

    # Hold the best model
    best_mse = np.inf  # init to infinity
    best_weights = None
    history = []

    for epoch in range(n_epochs):
        model.train()
        with tqdm.tqdm(batch_start, unit="batch", mininterval=0, disable=True) as bar:
            bar.set_description(f"Epoch {epoch}")
            for start in bar:
                # take a batch
                X_batch = X_train[start:start + batch_size]
                y_batch = y_train[start:start + batch_size]
                # forward pass
                y_pred = model(X_batch)
                loss = loss_fn(y_pred, y_batch)
                # backward pass
                optimizer.zero_grad()
                loss.backward()
                # update weights
                optimizer.step()
                # print progress
                bar.set_postfix(mse=float(loss))
        # evaluate accuracy at end of each epoch
        model.eval()
        y_pred = model(X_test)
        mse = loss_fn(y_pred, y_test)
        mse = float(mse)
        history.append(mse)
        if mse < best_mse:
            best_mse = mse
            best_weights = copy.deepcopy(model.state_dict())

    # restore model and return best accuracy
    model.load_state_dict(best_weights)
    #torch.save(model, "modelo_completo"+ number +".pth")

    new = np.linspace(0, 7700, num=7700)
    new = (new - 3.8722e+03) / 2.1336e+03
    X_new_tensor = torch.tensor(new, dtype=torch.float32).reshape(-1, 1)

    model.eval()
    with torch.no_grad():
        y_pred_test = model(X_new_tensor).numpy()

    r2 = r2_score(new, y_pred_test)
    print("R2: ", r2)


    input_file1 = "640-480-10m.txt"
    input_file2 = "640-480-30m.txt"
    input_file3 = "640-480-50m.txt"
    input_file4 = "640-480-70m.txt"

    x1, nothing = np.loadtxt(input_file1, unpack=True)
    x1 = (x1 - 3.8722e+03) / 2.1336e+03
    x2, nothing = np.loadtxt(input_file2, unpack=True)
    x2 = (x2 - 3.8722e+03) / 2.1336e+03
    x3, nothing = np.loadtxt(input_file3, unpack=True)
    x3 = (x3 - 3.8722e+03) / 2.1336e+03
    x4, nothing = np.loadtxt(input_file4, unpack=True)
    x4 = (x4 - 3.8722e+03) / 2.1336e+03

    X_new_tensor1 = torch.tensor(x1, dtype=torch.float32).reshape(-1, 1)
    X_new_tensor2 = torch.tensor(x2, dtype=torch.float32).reshape(-1, 1)
    X_new_tensor3 = torch.tensor(x3, dtype=torch.float32).reshape(-1, 1)
    X_new_tensor4 = torch.tensor(x4, dtype=torch.float32).reshape(-1, 1)

    model.eval()
    with torch.no_grad():
        y_pred_test1 = model(X_new_tensor1).numpy()
        y_pred_test2 = model(X_new_tensor2).numpy()
        y_pred_test3 = model(X_new_tensor3).numpy()
        y_pred_test4 = model(X_new_tensor4).numpy()

    #print(y_pred_test4)
    rmse1 = np.sqrt(mean_squared_error(np.full_like(y_pred_test1, 1000, dtype=float),
                                       y_pred_test1))
    rmse2 = np.sqrt(mean_squared_error(np.full_like(y_pred_test2, 3000, dtype=float),
                                       y_pred_test2))
    rmse3 = np.sqrt(mean_squared_error(np.full_like(y_pred_test3, 5000, dtype=float),
                                       y_pred_test3))
    rmse4 = np.sqrt(mean_squared_error(np.full_like(y_pred_test4, 7000, dtype=float),
                                       y_pred_test4))
    print("RMSE: ", rmse1)
    print("RMSE: ", rmse2)
    print("RMSE: ", rmse3)
    print("RMSE: ", rmse4)
    return history


# Define the model
model = nn.Sequential(
    nn.Linear(1, 8),
    nn.ReLU(),
    nn.Linear(8, 1)
)

model2 = nn.Sequential(
    nn.Linear(1, 16),
    nn.ReLU(),
    nn.Linear(16, 1)
)

model3 = nn.Sequential(
    nn.Linear(1, 32),
    nn.ReLU(),
    nn.Linear(32, 1)
)

model4 = nn.Sequential(
    nn.Linear(1, 64),
    nn.ReLU(),
    nn.Linear(64, 1)
)

# loss function and optimizer

modelinference1 = training(model,"1")
modelinference2 = training(model2,"2")
modelinference3 = training(model3,"3")
modelinference4 = training(model4,"4")

#print("MSE: %.2f" % best_mse)
#print("RMSE: %.2f" % np.sqrt(best_mse))
plt.plot(modelinference1, label="8 neurons")
plt.plot(modelinference2, label="16 neurons")
plt.plot(modelinference3, label="32 neurons")
plt.plot(modelinference4, label="64 neurons")
plt.xlabel("Epochs")
plt.ylabel("Mean Square Error")

plt.grid()
plt.legend()

plt.show()

