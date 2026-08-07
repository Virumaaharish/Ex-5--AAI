<H3>ENTER YOUR NAME: Virumaa Harish M</H3>
<H3>ENTER YOUR REGISTER NO.: 212223230246</H3>
<H3>EX. NO.5</H3>
<H3>DATE: 07.08.26</H3>
<H1 ALIGN =CENTER> Implementation of Kalman Filter</H1>
<H3>Aim:</H3> To Construct a Python Code to implement the Kalman filter to predict the position and velocity of an object.
<H3>Algorithm:</H3>
Step 1: Define the state transition model F, the observation model H, the process noise covariance Q, the measurement noise covariance R, the initial state estimate x0, and the initial error covariance P0.<BR>
Step 2:  Create a KalmanFilter object with these parameters.<BR>
Step 3: Simulate the movement of the object for a number of time steps, generating true states and measurements. <BR>
Step 3: For each measurement, predict the next state using kf.predict().<BR>
Step 4: Update the state estimate based on the measurement using kf.update().<BR>
Step 5: Store the estimated state in a list.<BR>
Step 6: Plot the true and estimated positions.<BR>
<H3>Program:</H3>

```PYTHON
import numpy as np
import matplotlib.pyplot as plt
class KalmanFilter:
    def __init__(self,F,H,Q,R,p0,x0):
      self.F=F
      self.H=H
      self.Q=Q
      self.R=R
      self.P=p0 # Initialize self.P
      self.X=x0 # Initialize self.X
    def predict(self):
      self.X=self.F@self.X
      self.P=self.F@self.P@self.F.T+self.Q
    def update(self,Z):
      y=Z-self.H@self.X # Changed 'z' to 'Z' to match parameter name
      S=self.H@self.P@self.H.T+self.R
      K=self.P@self.H.T@np.linalg.inv(S)
      self.X=self.X+K@y
      self.P=np.dot(np.eye(self.F.shape[0])-np.dot(K,self.H),self.P)
#Example
dt=0.1
F=np.array([[1,dt],[0,1]])
H=np.array([[1,0]])
Q=np.diag([0.1,0.1])
R=np.array([[1]])
p0=np.diag([1,1])
x0=np.array([0,0])
truestates=[]
measurements=[]
kf=KalmanFilter(F,H,Q,R,p0,x0)
for i in range(100):
  truestates.append([i*dt,1])
  measurements.append(i*dt+np.random.normal(scale=1))
estimatedstates=[]
for z in measurements:
  kf.predict()
  kf.update(np.array([z]))
  estimatedstates.append(kf.X)
plt.plot([s[0] for s in truestates],label="True States")
plt.plot(measurements,label="Measurements")
plt.plot([s[0] for s in estimatedstates],label="Estimated States")
plt.legend()
plt.show()
```

<H3>Output:</H3>

<img width="680" height="517" alt="image" src="https://github.com/user-attachments/assets/439cb46e-9fda-4560-b46e-7e5becebfd49" />


<H3>Results:</H3>
Thus, Kalman filter is implemented to predict the next position and   velocity in Python
