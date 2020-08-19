# Inverted Pendulum Dynamics Model

## Inverted Pendulum
We can obtain the initial mathmatical model of inverted pendulum from the wiki [1] as following:
$$
\begin{align} \label{eq:Roa}
(M + m)\ddot{x} - ml\ddot{\theta}cos\theta + ml\dot{\theta}^2sin\theta = F \\
l\ddot{\theta}^2 - gsin\theta = \ddot{x}cos\theta
\end{align}
$$
Let $v = \dot{x}$ and $\omega = \dot{\theta}$, we can obtain the following equations from above.
$$
\begin{align} \label{eq:Roa1}
(M + m)\dot{v} - ml\dot{\omega}cos\theta + ml\dot{\theta}^2sin\theta = F \\
l\dot{\omega}^2 - gsin\theta = \dot{v}cos\theta
\end{align}
$$
Combining this equations, wo can obtain
$$
\begin{align}
\dot{x} &= v \\
\dot{v} &= \\
\dot{\theta} &= \omega \\
\dot{\omega} &= 
\end{align}
$$
## Double Inverted Pendulum

## Triple Inverted Pendulum

## References
[1] [Inverted Pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum)