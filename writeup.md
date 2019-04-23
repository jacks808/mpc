# MPC PROJECT

---

# Implementation notes 

## Model
  
Use kinematic modelThe model have a state vector 
> \[ x, y, &psi;, v, cte, e&psi; \] 

Vehicle has two controller method:  &delta;, *a* to control
its movement. 

The the description of all varibles are the following:
  * vehicle coordinate x: (*x*)
  * vehicle coordinate y: (*y*) 
  * orientation of the vehicle: (&psi;)
  * velocity of the vehicle: (v) 
  * cross track error(CTE): (*cte*) 
  * orientation error: (*e*&psi;) 
  * xsteering angle: (*e*&psi;) 
  * acceleration*: (*a*) 
  
Predict the next state of the vehicle

predict equations:
  * *x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> \* cos(&psi;<sub>t</sub>)  \* dt<br/>*
  * *y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> \* sin(&psi;<sub>t</sub>)  \* dt<br/>*
  * *&psi;<sub>t+1</sub> = &psi;<sub>t</sub> + v<sub>t</sub> / Lf \*   &delta;<sub>t</sub> \* dt<br/>*
  * *v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> \* dt<br/>*
  * *cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> \* sin(e&psi;<sub>t</sub>) \* dt<br/>*
  * *e&psi;<sub>t+t</sub> = &psi;<sub>t</sub> - &psi;des<sub>t</sub> + v<sub>t</sub> / Lf  \* &delta;<sub>t</sub> \* dt<br/>*

The functions *f(x)* and *&psi;des* are polynomial functions calculated from the waypoints: 

![](https://ws1.sinaimg.cn/large/006tNc79ly1g2cw4t8p9cj30y00nwtel.jpg)

Cost functions: 

  * Cross track error with a weight of 1.
  * Orientation error with a weight of 100.
  * Velocity error with a weight of 10.
  * Steering angle with a weight of 5.
  * Aceleration with a weight of 5.
  * Steering angle variations with a weight of 1000.
  * Aceleration variations with a weight of 1.


## Choose N and dt

  Set N to 10 and elapsed duration *dt = 0.1*, it means that the trajectory is calculated 
  every second. I test some values, and I found several values with which the predicted trajectory fix
  quickly with the polynomial fitted trajectory and driving is smooth. finally I choose those values.
  
      
## MPC and latency

  I set the latency of 0.1s in the vehicle which can be found in project specifications. calcluate state and 
  after 0.1s, calutate the stat again, Then I'll get net state and use it to do the next work, which is as 
  input in the MPC controller.
  
