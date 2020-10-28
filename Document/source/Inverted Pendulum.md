# Inverted Pendulum Dynamics Model

## Inverted Pendulum
The inverted pendulum, shown in Fig 1. is one of the most popular laboratory  experiments used for illustrating nonlinear control theory.  We will devise the mathematical model by Euler-Lagrange's equations.
<center>
<img src="../image/inverted pendulum.jpg" alt="avatar" style="zoom:60%;"/>
<div style="font-size:14px;color:#C0C0C0;text-decoration:underline">Fig 1. Schematic of an inverted pendulum</div> 
</center>
Firstly, we present the kinetic and potential energies, which are used to compute the Lagrangian function. The kinetic energy of the cart is $K_1=\frac{1}{2}M\dot x^2$,  where $M$ is the cart mass, $x$ is the cart position. The kinetic energy of pendulum is $K_2=\frac{1}{2}m\dot P_x^2+\frac{1}{2}m\dot P_y^2+\frac{1}{2}I\dot \theta^2$, Where $P_x=x+lsin\theta$ is the pendulum horizontal position with $l$ is the pendulum length, $P_y=lcos\theta$ is the pendulum vertical position,  $\theta$ is the pendulum rotate angle, $m$ is the pendulum mass, and $I$ is the moment of inertia of the pendulum. Then, We can obtain the total kinetic energy as follows:

$K=K_1+K_2=\frac{1}{2}(m+M)\dot x^2+ml\dot x \dot \theta cos\theta+\frac{1}{2}(I+ml^2)\dot \theta^2$

The total potential energy is $P=mglcos\theta$. The Lagrangian function is given by

$$
\begin{align}
L &= K - P \\
&= \frac{1}{2}(m+M)\dot x^2+ml\dot x \dot \theta cos\theta+\frac{1}{2}(I+ml^2)\dot \theta^2-mglcos\theta
\end{align}
$$
The corresponding equations of motion are derived using Lagrange's equations
$$
\frac{d}{dt}\big (\frac{\partial L}{\partial \dot q}(q,\dot q)\big)-\frac{\partial L}{\partial q}(q,\dot q)=\tau
$$
Let $q=(x,\theta)$, and we obtain the following equations:
$$
\begin{align}
\frac{\partial L}{\partial x} &= 0\\
\frac{\partial L}{\partial \dot x} &= (m+M)\dot x+ml\dot \theta cos\theta\\
\frac{\partial L}{\partial \theta} &= mglsin\theta-ml\dot x\dot \theta sin\theta\\
\frac{\partial L}{\partial \dot\theta} &= ml\dot x cos\theta+(I+ml^2)\dot\theta
\end{align}
$$
Assume the cart friction is $F_d=k_c\dot x$ and pendulum friction is $\tau_d=k_p\dot \theta$. Then we obtain the following equations:

$$
\begin{align}
(m+M)\ddot x+ml\ddot \theta cos\theta-ml\dot\theta sin\theta&=f-k_c\dot x\\
ml\ddot x cos\theta+(I+ml^2)\ddot \theta-mglsin\theta&=-k_p\dot\theta
\end{align}
$$

Rewriting the above equations, we obtain
$$
\begin{bmatrix}
m+M& mlcos\theta\\
mlcos\theta& I+ml^2
\end{bmatrix}
\begin{bmatrix}
\ddot x\\
\ddot \theta\\
\end{bmatrix}+
\begin{bmatrix}
0& -mlsin \theta\\
0& 0
\end{bmatrix}
\begin{bmatrix}
\dot x\\
\dot \theta\\
\end{bmatrix}+
\begin{bmatrix}
0\\
-mglsin\theta
\end{bmatrix}=
\begin{bmatrix}
f-k_c\dot x\\
-k_p\dot \theta\\
\end{bmatrix}.
$$

Let $v=\dot x$ and $\omega=\dot\theta$, we obtain the mathematical model of the inverted pendulum as follows:

$$
\begin{bmatrix}
-mlsin \theta& 0& 0& 0\\
0& m+M& 0& mlcos\theta\\
0& 0& 0& 0\\
0& mlcos\theta& 0& I+ml^2
\end{bmatrix}
\begin{bmatrix}
\dot x\\
\dot v\\
\dot \theta\\
\dot \omega
\end{bmatrix}=
\begin{bmatrix}
0\\
f-k_c\dot x\\
0\\
mglsin\theta-k_p\dot \theta\\
\end{bmatrix}.
$$


If we assumed $\theta = 0$, we could obtain the linear model as following:
$$
\begin{align}
\dot{x} &= v \\
\dot{v} &= \frac{F}{M + m}\\
\dot{\theta} &= \omega \\
\dot{\omega} &= \frac{F}{(M + m)l}
\end{align}
$$
## Double Inverted Pendulum
<center>
<img src="../image/double inverted pendulum.jpg" alt="avatar" style="zoom:60%;"/>
<div style="font-size:14px;color:#C0C0C0;text-decoration:underline">Fig 2. Schematic of a double inverted pendulum</div> 
</center>
We can obtain the initial mathematical model of double inverted pendulum[2] as following:

## References
[1] [Inverted Pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum)
[2] [Srikanth, Kavirayani, and Nagesh Kumar G V. “Stabilization At Upright Equilibrium Position of a Double Inverted Pendulum With Unconstrained Bat Optimization.” International Journal on Computational Science & Applications 5.5 (2015): 87–101. Crossref. Web.](https://arxiv.org/abs/1511.02318)
[3] Fantoni, I, Lozano, R, and Sinha, SC. "Non-linear Control for Underactuated Mechanical Systems." Applied Mechanics Reviews 55.4 (2002): B67. Web.