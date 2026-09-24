# Spacecraft Rendezvous Optimization Formulation

## 1. State and decision variables

Let the target spacecraft define a circular reference orbit and let the chaser state be expressed in the target-centered, right-handed local-vertical/local-horizontal (LVLH) frame. Define

\[
\boxed{
\mathbf e_x=\text{radially outward},\qquad
\mathbf e_z=\frac{\mathbf r_t\times\mathbf v_t}{\|\mathbf r_t\times\mathbf v_t\|}
\text{ (orbit normal)},\qquad
\mathbf e_y=\mathbf e_z\times\mathbf e_x
\text{ (along-track)} }
\tag{1}
\]

and the relative state

\[
\boxed{
\mathbf x(t)=
\begin{bmatrix}\mathbf r(t)\\ \mathbf v(t)\end{bmatrix}
=
\begin{bmatrix}
x(t)&y(t)&z(t)&\dot x(t)&\dot y(t)&\dot z(t)
\end{bmatrix}^{\!T}\in\mathbb R^6,\quad
\mathbf r=\mathbf r_c-\mathbf r_t\ [\mathrm m],\quad
\mathbf v=\left.\frac{d\mathbf r}{dt}\right|_{\mathrm{LVLH}}\ [\mathrm{m\,s^{-1}}]. }
\tag{2}
\]

The controlled input is commanded chaser acceleration resolved in LVLH coordinates:

\[
\boxed{
\mathbf u(t)=
\begin{bmatrix}u_x(t)&u_y(t)&u_z(t)\end{bmatrix}^{T}
\in\mathbb R^3\ [\mathrm{m\,s^{-2}}],\qquad
\|\mathbf u(t)\|_2\le a_{\max}. }
\tag{3}
\]

Here \(a_{\max}>0\) is **not numerically specified**; a spacecraft-specific value must be confirmed. Equation (3) models total acceleration directly, so spacecraft mass, propellant depletion, thruster minimum impulse bit, pointing, and plume constraints are absent. If independent axis limits describe the actuator more accurately, replace (3) by

\[
\boxed{-\mathbf a_{\max}^{-}\le \mathbf u(t)\le \mathbf a_{\max}^{+}}
\tag{4}
\]

componentwise, with both vectors to be supplied.

The physical model is continuous on \(t\in[0,T_f]\). For finite-dimensional optimization, use \(N\) uniform zero-order-hold intervals:

\[
\boxed{
t_k=k\Delta t,\quad k=0,\ldots,N,\qquad
\Delta t=\frac{T_f}{N},\qquad
\mathbf u(t)=\mathbf u_k\quad(t\in[t_k,t_{k+1})),\qquad T_f=T. }
\tag{5}
\]

The horizon \(T_f\), number of intervals \(N\), and hence \(\Delta t\) are all to be confirmed. The direct-transcription decision vector is

\[
\boxed{
\mathbf z=
\begin{bmatrix}
\mathbf x_1^T&\cdots&\mathbf x_N^T&
\mathbf u_0^T&\cdots&\mathbf u_{N-1}^T
\end{bmatrix}^{T}\in\mathbb R^{9N}, }
\tag{6}
\]

because \(\mathbf x_0\) is prescribed rather than optimized.

## 2. Dynamics

For a target in a circular two-body orbit, define the constant mean motion

\[
\boxed{
n=\sqrt{\frac{\mu}{r_0^3}},\qquad r_0=R_E+h_{\mathrm{ref}},\qquad
\mu=\mu_E. }
\tag{7}
\]

**Assumption requiring confirmation:** the central body is Earth and the target follows a circular orbit at an as-yet unspecified altitude \(h_{\mathrm{ref}}\); therefore no numerical value of \(n\) is assumed. The CW approximation additionally requires relative separation small compared with \(r_0\), two-body gravity, and a target orbit of negligible eccentricity.

Using the axis convention in (1), the forced Clohessy-Wiltshire equations are

\[
\boxed{
\begin{aligned}
\ddot x-2n\dot y-3n^2x&=u_x,\\
\ddot y+2n\dot x&=u_y,\\
\ddot z+n^2z&=u_z.
\end{aligned} }
\tag{8}
\]

Equivalently,

\[
\boxed{
\dot{\mathbf x}=A\mathbf x+B\mathbf u,\quad
A=
\begin{bmatrix}
0&0&0&1&0&0\\
0&0&0&0&1&0\\
0&0&0&0&0&1\\
3n^2&0&0&0&2n&0\\
0&0&0&-2n&0&0\\
0&0&-n^2&0&0&0
\end{bmatrix},\quad
B=
\begin{bmatrix}
0&0&0\\0&0&0\\0&0&0\\1&0&0\\0&1&0\\0&0&1
\end{bmatrix}. }
\tag{9}
\]

These signs and axes are the standard radial/along-track/cross-track form obtained by relabeling \((\xi,\eta,\zeta)\) in the cited CW source form. Under the zero-order hold in (5), exact propagation—not a numerical integration approximation—is

\[
\boxed{
\mathbf x_{k+1}=\Phi(h)\mathbf x_k+\Gamma(h)\mathbf u_k,qquad
h=\Delta t,qquad
\Phi(h)=e^{Ah},\qquad
\Gamma(h)=\int_0^h e^{A\tau}B\,d\tau. }
\tag{10}
\]

With \(c=\cos(nh)\) and \(s=\sin(nh)\), the complete transition matrix is

\[
\boxed{
\Phi(h)=
\begin{bmatrix}
4-3c&0&0&\dfrac{s}{n}&\dfrac{2(1-c)}{n}&0\\[4pt]
6(s-nh)&1&0&-\dfrac{2(1-c)}{n}&\dfrac{4s-3nh}{n}&0\\[4pt]
0&0&c&0&0&\dfrac{s}{n}\\[4pt]
3ns&0&0&c&2s&0\\[4pt]
-6n(1-c)&0&0&-2s&4c-3&0\\[4pt]
0&0&-ns&0&0&c
\end{bmatrix}. }
\tag{11}
\]

The exact constant-acceleration input matrix is

\[
\boxed{
\Gamma(h)=
\begin{bmatrix}
\dfrac{1-c}{n^2}&\dfrac{2(nh-s)}{n^2}&0\\[5pt]
-\dfrac{2(nh-s)}{n^2}&\dfrac{4(1-c)}{n^2}-\dfrac{3h^2}{2}&0\\[5pt]
0&0&\dfrac{1-c}{n^2}\\[5pt]
\dfrac{s}{n}&\dfrac{2(1-c)}{n}&0\\[5pt]
-\dfrac{2(1-c)}{n}&\dfrac{4s}{n}-3h&0\\[5pt]
0&0&\dfrac{s}{n}
\end{bmatrix}. }
\tag{12}
\]

Equations (8) and the unforced solution underlying (11) follow W. H. Clohessy and R. S. Wiltshire, “Terminal Guidance System for Satellite Rendezvous,” *Journal of the Aerospace Sciences*, 27(9), 1960, pp. 653–658, [doi:10.2514/8.8704](https://doi.org/10.2514/8.8704), as reproduced and derived in [MIT 16.346 Astrodynamics, Lecture 26, pp. 2–3](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/e4f0632a9f1c98f7e9b25492e1a30eb1_lec_26.pdf). Equation (12) follows exactly from the integral in (10), column by column; it is not an impulsive-control approximation.

## 3. Objective

Let the desired rendezvous state be \(\mathbf x_d=\mathbf 0_6\), let \(\mathbf x_{r,k}\) be an optional reference trajectory (normally \(\mathbf 0_6\)), and choose

\[
\boxed{
Q\succeq0,\qquad Q_f\succeq0,\qquad R\succ0,\qquad
Q,Q_f\in\mathbb S_+^6,\qquad R\in\mathbb S_{++}^3. }
\tag{13}
\]

A general continuous-time quadratic performance index is

\[
\boxed{
J_c[\mathbf x,\mathbf u]
=\frac12(\mathbf x(T_f)-\mathbf x_d)^TQ_f(\mathbf x(T_f)-\mathbf x_d)
+\frac12\int_0^{T_f}
\left[(\mathbf x(t)-\mathbf x_r(t))^TQ(\mathbf x(t)-\mathbf x_r(t))
+\mathbf u(t)^TR\mathbf u(t)\right]dt. }
\tag{14}
\]

Its rectangular-rule discrete counterpart used for optimization is

\[
\boxed{
J_d(\mathbf z)=
\frac12(\mathbf x_N-\mathbf x_d)^TQ_f(\mathbf x_N-\mathbf x_d)
+\frac{h}{2}\sum_{k=0}^{N-1}
\left[(\mathbf x_k-\mathbf x_{r,k})^TQ(\mathbf x_k-\mathbf x_{r,k})
+\mathbf u_k^TR\mathbf u_k\right]. }
\tag{15}
\]

Here \(R\) penalizes thrust effort, \(Q\) penalizes pathwise displacement and velocity error, and \(Q_f\) penalizes terminal error. Their numerical values and physical scaling are unspecified and require confirmation. If exact terminal rendezvous is imposed, the terminal term in (15) is identically zero; a minimum-energy baseline then results from \(Q=0\) and \(R\succ0\). A proposed time penalty would be

\[
\boxed{J_{d,T}=J_d+w_TT_f,\qquad w_T\ge0,}
\tag{16}
\]

but for the stated fixed horizon \(T_f\), \(w_TT_f\) is constant and cannot affect the optimizer. Making \(T_f\) a decision variable changes (11)–(12) nonlinearly and is a different, generally nonconvex problem.

## 4. Constraints

The prescribed initial condition is

\[
\boxed{\mathbf x_0=\mathbf x_{\mathrm{init}}\in\mathbb R^6,}
\tag{17}
\]

where all six numerical components remain to be supplied. The baseline exact rendezvous constraint is

\[
\boxed{\mathbf x_N=\mathbf x_d=\mathbf 0_6
\iff \mathbf r_N=\mathbf0_3,\quad\mathbf v_N=\mathbf0_3.}
\tag{18}
\]

If exact equality is not mission-appropriate, a tolerance formulation is

\[
\boxed{
\|\mathbf r_N\|_2\le\varepsilon_r,qquad
\|\mathbf v_N\|_2\le\varepsilon_v, }
\tag{19}
\]

where \(\varepsilon_r\ [\mathrm m]\) and \(\varepsilon_v\ [\mathrm{m\,s^{-1}}]\) are **not selected here** and must be confirmed. The discrete control and any state/path constraints are

\[
\boxed{
\|\mathbf u_k\|_2\le a_{\max},\quad k=0,\ldots,N-1,qquad
\mathbf x_k\in\mathcal X_k,\quad k=1,\ldots,N. }
\tag{20}
\]

No numerical control limit and no state set \(\mathcal X_k\) are known. Thus the baseline has \(\mathcal X_k=\mathbb R^6\). Approach corridors, line-of-sight cones, keep-out zones, plume impingement, and collision avoidance must not be presumed absent from a flight problem, but they cannot be written numerically until mission geometry is provided.

### 4.1 Direct transcription / multiple shooting

The sparse equality-constrained formulation is

\[
\boxed{
\begin{aligned}
\underset{\mathbf x_1,\ldots,\mathbf x_N,\,\mathbf u_0,\ldots,\mathbf u_{N-1}}{\operatorname{minimize}}\quad
&J_d(\mathbf z)\\
\operatorname{subject\ to}\quad
&\mathbf x_0=\mathbf x_{\mathrm{init}},\\
&\mathbf x_{k+1}-\Phi\mathbf x_k-\Gamma\mathbf u_k=\mathbf0_6,
&&k=0,\ldots,N-1,\\
&\mathbf x_N=\mathbf0_6,\\
&\|\mathbf u_k\|_2\le a_{\max},
&&k=0,\ldots,N-1,\\
&\mathbf x_k\in\mathcal X_k,
&&k=1,\ldots,N.
\end{aligned} }
\tag{21}
\]

States at every node are independent decision variables tied together by \(6N\) linear dynamic equalities. This multiple-shooting/direct-transcription form produces a larger but sparse KKT system and exposes the equality-constraint Jacobian for conditioning analysis.

### 4.2 Condensed single shooting

Repeated substitution of (10) gives

\[
\boxed{
\mathbf x_k=\Phi^k\mathbf x_{\mathrm{init}}
+\sum_{j=0}^{k-1}\Phi^{k-1-j}\Gamma\mathbf u_j,qquad k=1,\ldots,N. }
\tag{22}
\]

Define the stacked input and terminal controllability matrix

\[
\boxed{
\mathbf U=\begin{bmatrix}\mathbf u_0^T&\cdots&\mathbf u_{N-1}^T\end{bmatrix}^T,qquad
G_N=\begin{bmatrix}\Phi^{N-1}\Gamma&\Phi^{N-2}\Gamma&\cdots&\Gamma\end{bmatrix}
\in\mathbb R^{6\times3N}. }
\tag{23}
\]

Then exact rendezvous becomes

\[
\boxed{G_N\mathbf U=-\Phi^N\mathbf x_{\mathrm{init}},}
\tag{24}
\]

and substituting (22) into (15) yields a condensed quadratic objective

\[
\boxed{J_d(\mathbf U)=\frac12\mathbf U^TH\mathbf U+\mathbf g^T\mathbf U+c,qquad H\succeq0,}
\tag{25}
\]

where \(H\), \(\mathbf g\), and \(c\) are uniquely determined by \(\Phi,\Gamma,Q,Q_f,R,h,\mathbf x_{\mathrm{init}},\mathbf x_r\). Single shooting eliminates all dynamic equalities but not the terminal equality (24) or bounds. It creates a dense condensed Hessian whose conditioning may degrade as \(N\), \(T_f\), or weight disparities increase.

If \(G_N\) has full row rank and (24) is feasible, let \(\mathbf U_p\) satisfy (24) and let columns of \(Z\) form a basis for \(\ker(G_N)\). Then

\[
\boxed{
\mathbf U=\mathbf U_p+Z\mathbf w,qquad
\underset{\mathbf w}{\operatorname{minimize}}\quad
\frac12\mathbf w^T(Z^THZ)\mathbf w
+\bigl[Z^T(H\mathbf U_p+\mathbf g)\bigr]^T\mathbf w, }
\tag{26}
\]

which is genuinely unconstrained only if control and path inequalities are omitted. Formulations (21), (25) with (24), and (26) are mathematically related but expose different matrices—and therefore different numerical conditioning.

## 5. Problem classification

Under the fixed-horizon baseline, the classification is

\[
\boxed{
\begin{array}{ll}
\text{variables:}&\text{finite-dimensional continuous real variables},\\
\text{dynamics:}&\text{linear time invariant},\\
\text{objective:}&\text{convex quadratic if }Q,Q_f\succeq0, R\succ0,\\
\text{equalities:}&\text{affine initial, dynamics, and terminal constraints},\\
\text{thrust bound:}&\text{convex second-order cone under (3), or linear under (4)},\\
\text{baseline class:}&\text{convex QCQP/SOCP; a QP with box bounds or without (3)},\\
\text{minimum-energy, equalities only:}&\text{strictly convex equality-constrained QP in }\mathbf U
\text{ when the reduced Hessian is positive definite},\\
\text{integer variables:}&\text{none},\\
\text{stochastic quantities:}&\text{none}.
\end{array} }
\tag{27}
\]

Nonconvex keep-out constraints, nonlinear mass/thrust dynamics, variable final time, eccentric reference motion, or nonlinear orbital dynamics would invalidate parts of (27). The repository contains no Project 1 formulation or classification rubric, so (27) uses a standard optimization classification; the exact Project 1 terminology must be supplied or confirmed.

## 6. Open questions

Every unresolved modeling choice is collected below:

\[
\boxed{
\begin{aligned}
\mathcal Q=\{&
\text{Project 1 classification terminology};\\
&\text{central body, }R_E,\mu_E,\text{ circular reference altitude }h_{\mathrm{ref}}\text{ and }n;\\
&\text{initial state }\mathbf x_{\mathrm{init}}\text{ and confirmation of LVLH sign convention};\\
&\text{fixed horizon }T_f,\text{ interval count }N,\text{ and sample time }h;\\
&\text{whether final time is fixed or optimized};\\
&\text{exact terminal equality versus tolerances }\varepsilon_r,\varepsilon_v;\\
&\text{weights }Q,Q_f,R\text{, their nondimensional scaling, and optional }w_T;\\
&\text{thrust magnitude }a_{\max}\text{ versus componentwise limits};\\
&\text{whether acceleration limits vary with mass and whether mass is a state};\\
&\text{continuous-thrust assumptions: minimum thrust, duty cycle, slew/pointing, and bandwidth};\\
&\text{state/path limits }\mathcal X_k\text{: keep-out zone, approach corridor, line of sight,}\\
&\text{plume, collision, and closing-speed rules};\\
&\text{reference trajectory }\mathbf x_r(t)\text{ and whether path-state penalties are desired};\\
&\text{validity range of CW linearization and whether perturbations/eccentricity are material};\\
&\text{preferred optimization form: sparse transcription, condensed shooting, or null-space reduction};\\
&\text{quadrature choice for the continuous cost and control hold model};\\
&\text{units and scaling used for conditioning comparisons}\}.
\end{aligned} }
\tag{28}
\]
