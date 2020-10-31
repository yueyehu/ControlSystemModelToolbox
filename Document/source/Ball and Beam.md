# Ball & Beam System
The ball and beam system, shown in Fig 1, is a well dynamic system to illustrate control theory.  We will devise the mathematical model by Euler-Lagrange's equations.
<center>
<img src="../image/ball and beam system.jpg" alt="avatar" style="zoom:80%;"/>
<div style="font-size:14px;color:#C0C0C0;text-decoration:underline">Fig 1. Ball and Beam System</div> 
</center>

Firstly, we present the kinetic and potential energies, which are used to compute the Lagrangian function. The kinetic energy of the ball is $K_1=\frac{1}{2}m\dot r^2+\frac{1}{2}I_{ba}\omega_{ba}^2+\frac{1}{2}J(\frac{\dot r}{R})^2$, where $m$ is the ball's mass, $r$ is the ball's position, $I_{ba}=m(L-r)^2$ is the ball's moment of inertia. The kinetic energy of the Beam is $K_2=\frac{1}{2}I_{be}\dot \alpha^2$, where $I_{be}=\frac{1}{3}ML^2$ is the moment of inertia of the beam, $\alpha$ is the beam's angle. Then, We obtain the total kinetic energy as follows:

$K=K_1+K_2+K_3=\frac{1}{2}m\dot r^2+\frac{1}{2}I_{ba}\omega_{ba}^2+\frac{1}{2}J(\frac{\dot r}{R})^2+\frac{1}{2}I_{be}\dot \alpha^2$.

The potential energy of ball is $P_1=mg(L-r)sin\alpha$, where $L$ is the length of beam. The potential energy of beam is $P_2=\frac{1}{2}MgL\sin\alpha$, where $M$ is the beam's mass. Then, the total potential energy is $P=P_1+P_2=mg(L-r)\sin\alpha+\frac{1}{2}MgL\sin\alpha$.

The Lagrangian function is given by

$$
\begin{align}
L &= K - P \\
&= \frac{1}{2}m\dot r^2+\frac{1}{2}I_{ba}\omega_{ba}^2+\frac{1}{2}J(\frac{\dot r}{R})^2+\frac{1}{2}I_{be}\dot \alpha^2-mg(L-r)\sin\alpha-\frac{1}{2}MgL\sin\alpha
\end{align}
$$
The corresponding equations of motion are derived using Lagrange's equations
$$
\frac{d}{dt}\big (\frac{\partial L}{\partial \dot q}(q,\dot q)\big)-\frac{\partial L}{\partial q}(q,\dot q)=\tau
$$
Since $\omega_{ba} \approx \dot\alpha$ is too small, we obtain $\frac{1}{2}I_{ba}\omega_{ba}^2=\frac{1}{2}m(L-r)^2\dot\alpha^2$. Rewriting $L$, we get

$$
L = \frac{1}{2}(m+\frac{J}{R^2})\dot r^2+\frac{1}{2}(m(L-r)^2+\frac{1}{3}ML^2)\dot \alpha^2-\big(mg(L-r)+\frac{1}{2}MgL\big)\sin\alpha
$$

Let $q=(r,\alpha)$, and we obtain the following equations:
$$
\begin{align}
\frac{\partial L}{\partial r} &= mg\sin\alpha-m(L-r)\dot\alpha^2\\
\frac{\partial L}{\partial \dot r} &= (m+\frac{J}{R^2})\dot r\\
\frac{\partial L}{\partial \alpha} &= -\big(mg(L-r)+\frac{1}{2}MgL\big)\cos\alpha\\
\frac{\partial L}{\partial \dot\alpha} &= (m(L-r)^2+\frac{1}{3}ML^2)\dot\alpha
\end{align}
$$

Substituting above equations into Lagrange's equations, we obtain
$$
\begin{align}
(m+\frac{J}{R^2})\ddot r-mg\sin\alpha+m(L-r)\dot\alpha^2&=0\\
(m(L-r)^2+\frac{1}{3}ML^2)\ddot\alpha-2m(L-r)\dot r\dot\alpha+\big(mg(L-r)+\frac{1}{2}MgL\big)\cos\alpha&=\tau
\end{align}
$$
Let $v=\dot r$ and $\omega=\dot\alpha$, we obtain the mathematical model of the inverted pendulum as follows:
$$
\begin{align}
\dot r &= v\\
\dot v &= \frac{mg\sin\alpha-m(L-r)\dot\alpha^2}{m+\frac{J}{R^2}}\\
\dot \alpha &= \omega \\
\dot \omega &= \frac{1}{(m(L-r)^2+\frac{1}{3}ML^2)}\big(\tau+2m(L-r)\dot r\dot\alpha-\big(mg(L-r)+\frac{1}{2}MgL\big)\cos\alpha\big)
\end{align}
$$
Assume $\alpha=0$ and $\dot\alpha=0$, we simplify the model as follows:
$$
\begin{align}
\dot r &= v\\
\ddot r &= \frac{mg\sin\alpha}{m+\frac{J}{R^2}}\\
\dot \alpha &= \omega \\
\dot \omega &= \frac{1}{(m(L-r)^2+\frac{1}{3}ML^2)}\big(\tau-\big(mg(L-r)+\frac{1}{2}MgL\big)\big)
\end{align}
$$

## Controller

We use the simplified model to design controller. Our object is to let the ball at a fixed point $r_d$. Since this system has two degree $r$ and $\alpha$, and only have one input $\tau$, this is an under-actuated system. We proposed a cascaded controller, the inner controller is to track the trajectory of $\alpha$, and the outer controller is to control the ball at $r_d$. We introduce some errors as follows:
$$
\begin{align}
e_1 &= r - r_d\\
e_2 &= \dot e_1+k_1e_1\\
e_3 &= \alpha-\alpha_d\\
e_4 &= \dot e_3+k_3e_3
\end{align}
$$
where $k_1$ and $k_3$ is positive constant.

We consider a Lyapunov function $V_1=\frac{1}{2}e_1^2+\frac{1}{2}e_2^2$ for outer controller. The time derivative of it is 
$$
\begin{align}
\dot V_1 &= e_1\dot e_1+e_2\dot e_2\\
       &= e_1(e_2-k_1e_1)+e_2(\ddot e_1+k_1\dot e_1) \\
       &=-k_1e_1^2+e_2(\ddot e_1+k_1\dot e_1+e_1)
\end{align}
$$
We choose $\ddot e_1+k_1\dot e_1+e_1 = -k_2e_2$, where $k_2$ is positive constant, which make $\dot V_1=-k_1e_1^2-k_2e_2^2\leq0$. Since $r_d$ is fixed point, we obtain $\ddot r+k_1r+e_1=-k_2e_2$. Using the simplified model, we obtain 
$$
\alpha_d=-\frac{(k_1r+k_2e_2+e_1)(m+\frac{J}{R^2})}{mg\sin\alpha}
$$
Similarly, We consider a Lyapunov function $V_2=\frac{1}{2}e_3^2+\frac{1}{2}e_4^2$ for inner controller. The time derivative of it is
$$
\begin{align}
\dot V_2 &= e_3\dot e_3+e_4\dot e_4\\
         &= e_3(e_4-k_3e_3)+e_4(\ddot e_3+k_3\dot e_3)\\ 
         &=-k_3e_3^2+e_4(\ddot e_3+k_3\dot e_3+e_3)
\end{align}
$$
We choose $\ddot e_3+k_3\dot e_3+e_3 = -k_4e_4$, where $k_2$ is positive constant, which make $\dot V_1=-k_3e_3^2-k_4e_4^2\leq0$. Using the simplified model, we obtain 
$$
\frac{1}{(m(L-r)^2+\frac{1}{3}ML^2)}\big(\tau-\big(mg(L-r)+\frac{1}{2}MgL\big)\big)=-(k_3\dot e_3+k_4e_4+e_3)
$$
then
$$
\tau=-(k_3\dot e_3+k_4e_4+e_3)(m(L-r)^2+\frac{1}{3}ML^2)+\big(mg(L-r)+\frac{1}{2}MgL\big)
$$

## Appendix
We obtain the relationship between $\alpha$ and $\theta$ is

$\alpha=\arccos(\frac{L^2+l^2-L_a^2}{2Ll}) +\arcsin\frac{d\sin(\pi-\delta-\theta)}{l}-\delta$ ,

or

$\theta=\pi-\delta-\arcsin\bigg(\frac{l}{d}\sin\big(\alpha+\delta-\arccos(\frac{L^2+l^2-L_a^2}{2Ll})\big)\bigg)$,

where $l=\sqrt{L_f^2+d^2-2dL_f}$ with $L_f=\sqrt{(L-d)^2+L_a^2}$, $\delta=\arctan\frac{L_a}{L-d}$.


## Reference
[1] Ball & Beam System (https://ctms.engin.umich.edu/CTMS/index.php?example=BallBeam&section=SystemModeling)