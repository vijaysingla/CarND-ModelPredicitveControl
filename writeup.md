# CarND-Controls-MPC

In this project, Model predictive control is used to drive the car around the simulator. At every time step, x,y, position of car, car orientation with x axis ,its centre of mass velocity and reference trajectory is passed to the MPC algorithm . MPC uses a cost function under system model constraints and actuator constraints, to determine the throttle and steering angle that can produce optimal path as close as possible to refernce trajectory.The initial value of throttle and steering angle is passed to simulator and the same process is repeated at every time step. This process is being repeated every time step as the model used to predict the car behaviour is approximate model. 


## Car Kinematics Model

A kinematic model is used to emulate the car behaviour. There are 6 states in the model named as x ,y (car_position in x ,y direction) , psi (car orientation with x axis), v (car centre of mass velocity) , cte( cross track error) , epsi( orientation error ) . 

Other parameters used in defining the model are dt (time step), Lf ( length from front axle to centre of mass of car ) . THe actuator inputs are throttle (acc) and steering angle(delta) . The model state equations are  :

x(t+1) = x(t) + v(t)*cos(psi)*dt

y(t+1) = y(t) + v(t)*sin(psi)*dt

psi(t+1) = psi(t) +v(t)*delta(t)/Lf*dt

v(t+1) =v(t) + acc(t)*dt

cte(t+1) =cte(t) + v(t)*sin(epsi(t))*dt

epsi(t+1) = epsi(t) + v(t)*delta(t)/Lf*dt

This model is similar to bicylce model in some context . The main difference is the way psi  is updated. In this model, for a steering angle and fixed velocity, less turn rate is produced if Lf is large. This model simplifies the psi update but captures the critical factors like velcocity, steeirng angle and Lf effecting the orientation. For the simualtor, Lf is tuned so that model can represent he simualtor car driving behavior.

More complicated models like dynamic models including tire models can be used to simulate the car driving. But these models are computationally expensive. 


## Choice of parameters like Time step Length and Elapsed duration (N and dt)

For this tuning, I intially used a low refernce velocity of car = 10 mph so that I can see whats happening clearly.
Intially , I choose N as 30 and dt as 0.001  and observed that total time horizon is .03 seconds. This was low time horizon and model did not work properly in this . I then increased dt to 0.01 seconds. 0.3 seconds as time horizon looked okay and model behaved better but car was crossing the lanes and was not driving safely  and cte was fluctating a lot. After that I picked N as 100 and model was considering too long path for algorithm and behaved terribly . After that I tried N as 10 and dt 0.01 and my car ran properly at low velocity. 
After that I made dt and weights of cost function as arguments to main function to tune effectivelya at high velocities. After going through iterations, I settled on dt as 0.05 and N as 10 , and tuned epsi cost weight and car drove perfectly at ref_v about 30 mph . 

I could have  also preprocessed the way points and tuned N and dt and cost function for the preprocessed way points. 


## Polynomial Fitting ad MPC Preprocessing. 

I recieved 6 reference points in x and y direction , car x, y , and orienataion in global coordinates from the simualtor. I first transformed the global reference points to local car coordinates. Basically, I made the origin to be at car positon and x axis to be in the direction of car orientation. THis made the   position and orientation of car in local coordinate system   that is being passed to MPC algoithm to be zero at every time step.

I fitted a 3rd order polynomial to the  transformed reference coordinates and used the polyfit to calcualte the cte error and derivate of fit to calcualte the epsi error being passed to MPC algorithm


## Model Predictive Control with latency

The simualtor has a 100 ms latency for actuator. Every actuator value has a latency of 100 ms to the commanded value . I gave this information to the state model to take care of the latency. I chose N as 10 and dt as 0.05. it means my time horizon is 0.5 seconds. and there is latency of 0.1 seconds. It means  for every 3rd  point out of N  , acceleration  is equal to acceleartion of 2 points prior to it. I gave this information to state model using the following code given in lines 132-136 in MPC.cpp file

    if (t > 2)
	        {
	        	delta0 = vars[delta_start + t -3];
	        	a0 = vars[a_start + t - 3];
	        }

I had additional latency due to VM machine as well.



