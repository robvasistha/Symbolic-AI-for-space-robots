from dso import DeepSymbolicOptimizer
import numpy as np

np.random.seed(0)
X = np.random.random((10,2))
y = np.sin(X[:,0]) + X[:,1] **2
model = DeepSymbolicOptimizer("spacerobotconfig.json")
#model.fit(X,y)
print(model.program_.pretty())
model.predict(2*X)
#model.train()
