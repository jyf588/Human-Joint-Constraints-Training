from keras.models import Sequential
from keras.layers import Dense
import scipy.io as sio
import numpy as np

mat_contents = sio.loadmat('randomsin_arm_left.mat')
xyall = mat_contents['qTrain_ba_sin']
print(xyall.shape)
np.random.shuffle(xyall)

length = xyall.shape[0]

xy_train = xyall[0:int(length*0.8),:]
xy_test = xyall[int(length*0.8):,:]

y_train = xy_train[:,6].reshape(-1,1).astype(int)
x_train = xy_train[:,0:6]

y_test = xy_test[:,6].reshape(-1,1).astype(int)
x_test = xy_test[:,0:6]

print(x_train.shape)
print(y_train.shape)
print(x_test.shape)
print(y_test.shape)

model = Sequential()
model.add(Dense(128, input_dim=6, activation='tanh'))
model.add(Dense(128, activation='tanh'))
model.add(Dense(128, activation='tanh'))
model.add(Dense(1, activation='sigmoid'))

model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])
print(model.metrics_names)

model.fit(x_train, y_train, epochs=60, batch_size=256)
score = model.evaluate(x_test, y_test, batch_size=256)
print(score)

model.save('3-128-ran-larm.h5')
