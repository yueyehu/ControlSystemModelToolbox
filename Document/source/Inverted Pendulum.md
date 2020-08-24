# Inverted Pendulum Dynamics Model

## Inverted Pendulum
We can obtain the initial mathmatical model of inverted pendulum from the wiki [1] as following:
$$
\begin{align} \label{eq:Roa}
(M + m)\ddot{x} - ml\ddot{\theta}cos\theta + ml\dot{\theta}^2sin\theta = F \\
l\ddot{\theta} - gsin\theta = \ddot{x}cos\theta
\end{align}
$$
Let $v = \dot{x}$ and $\omega = \dot{\theta}$, we can obtain the following equations from above.
$$
\begin{align} \label{eq:Roa1}
(M + m)\dot{v} - ml\dot{\omega}cos\theta + ml\omega^2sin\theta = F \\
l\dot{\omega} - gsin\theta = \dot{v}cos\theta
\end{align}
$$
Combining this equations, wo can obtain
$$
\begin{align}
\dot{x} &= v \\
\dot{v} &= \frac{1}{M + msin^2\theta}\left(F-m(l\omega^2 - gcos\theta)sin\theta\right)\\
\dot{\theta} &= \omega \\
\dot{\omega} &= \frac{1}{(M + msin^2\theta)l}\left(Fcos\theta + (M + m)gsin\theta - ml\omega^2sin\theta cos\theta\right)
\end{align}
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
We can obtain the initial mathmatical model of double inverted pendulum[2] as following:

## References
[1] [Inverted Pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum)
[2] [Srikanth, Kavirayani, and Nagesh Kumar G V. “Stabilization At Upright Equilibrium Position of a Double Inverted Pendulum With Unconstrained Bat Optimization.” International Journal on Computational Science & Applications 5.5 (2015): 87–101. Crossref. Web.](https://arxiv.org/abs/1511.02318)