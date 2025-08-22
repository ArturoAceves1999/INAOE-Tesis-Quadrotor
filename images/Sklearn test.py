import matplotlib.pyplot as plt
import numpy as np
import time
import pandas as pd

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score

poly = PolynomialFeatures(degree=4, include_bias=True)
poly_reg_model = LinearRegression()

input_file = "salidaFORMC.txt"
x_train, x_test, x_valid, y = np.loadtxt(input_file, unpack=True)

poly_features = poly.fit_transform(x_train.reshape(-1, 1))

print(poly_features)
poly_reg_model.fit(poly_features, y)
y_predicted = poly_reg_model.predict(poly_features)
#print(y_predicted)
print("R2: ", r2_score(y,y_predicted))

features = poly.get_feature_names_out(['x'])
coefs = [poly_reg_model.intercept_] + list(poly_reg_model.coef_[1:])  # intercepto + coef
for f, c in zip(features, coefs):
    print(f"{c:.18f} * {f}")

input_file1 = "640-480-10m.txt"
input_file2 = "640-480-30m.txt"
input_file3 = "640-480-50m.txt"
input_file4 = "640-480-70m.txt"

x1, nothing = np.loadtxt(input_file1, unpack=True)
x2, nothing = np.loadtxt(input_file2, unpack=True)
x3, nothing = np.loadtxt(input_file3, unpack=True)
x4, nothing = np.loadtxt(input_file4, unpack=True)

poly_features1 = poly.fit_transform(x1.reshape(-1, 1))
poly_features2 = poly.fit_transform(x2.reshape(-1, 1))
poly_features3 = poly.fit_transform(x3.reshape(-1, 1))
poly_features4 = poly.fit_transform(x4.reshape(-1, 1))

y_predicted1 = poly_reg_model.predict(poly_features1)
y_predicted2 = poly_reg_model.predict(poly_features2)
y_predicted3 = poly_reg_model.predict(poly_features3)
y_predicted4 = poly_reg_model.predict(poly_features4)

rmse1 = np.sqrt(mean_squared_error(np.full_like(y_predicted1, 1000, dtype=float),
                          y_predicted1))
rmse2 = np.sqrt(mean_squared_error(np.full_like(y_predicted2, 3000, dtype=float),
                          y_predicted2))
rmse3 = np.sqrt(mean_squared_error(np.full_like(y_predicted3, 5000, dtype=float),
                          y_predicted3))
rmse4 = np.sqrt(mean_squared_error(np.full_like(y_predicted4, 7000, dtype=float),
                          y_predicted4))

new = np.linspace(0,20000,num = 20000)
poly_features5 = poly.fit_transform(new.reshape(-1, 1))
y_predicted5 = poly_reg_model.predict(poly_features5)

print("RMSE: ", rmse1)
print("RMSE: ", rmse2)
print("RMSE: ", rmse3)
print("RMSE: ", rmse4)
plt.figure(figsize=(10,6))
plt.scatter(x_train, y)
plt.plot(x_train, y_predicted, c="red")
plt.plot(new, y_predicted5, c="blue")
plt.show()