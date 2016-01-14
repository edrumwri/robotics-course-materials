--- 
layout: post
title:  "An engineer's guide to matrices, vectors, and numerical linear algebra"
date:   2015-12-18
---

This material is not meant to replace a proper course in linear algebra, where
important concepts like vector spaces, eigenvalues/eigenvectors, and
spans are defined. My intent is to give you a practical guide to concepts
in matrix arithmetic and numerical linear algebra that I have found useful. 

### Matrix arithmetic


#### Matrix and vector addition and subtraction

Matrices and vectors can only be added or subtracted if they are of the
same dimensionality. Then, addition and subtraction are performed
elementwise:

\begin{equation*}
\begin{bmatrix}
u\_1 \\\\
u\_2 \\\\
u\_3 
\end{bmatrix} + 
\begin{bmatrix}
v\_1 \\\\
v\_2 \\\\
v\_3
\end{bmatrix} =
\begin{bmatrix}
u\_1 + v\_1 \\\\
u\_2 + v\_2 \\\\
u\_3 + v\_3 
\end{bmatrix}
\end{equation*}

\begin{equation*}
\begin{bmatrix}
m\_{11} & m\_{12} & m\_{13} \\\\
m\_{21} & m\_{22} & m\_{23} \\\\
m\_{31} & m\_{32} & m\_{33}
\end{bmatrix} + 
\begin{bmatrix}
n\_{11} & n\_{12} & n\_{13} \\\\
n\_{21} & n\_{22} & n\_{23} \\\\
n\_{31} & n\_{32} & n\_{33}
\end{bmatrix} = 
\begin{bmatrix}
m\_{11} + n\_{11} & m\_{12} + n\_{12} & m\_{13} + n\_{13} \\\\
m\_{21} + n\_{21} & m\_{22} + n\_{22} & m\_{23} + n\_{23} \\\\
m\_{31} + n\_{31} & m\_{32} + n\_{32} & m\_{33} + n\_{33}
\end{bmatrix}
\end{equation*}

#### Matrix and vector scaling

Matrix and vectors can be scaled by multiplying every element in the matrix or vector by the scalar, as seen below:

\begin{align*}
\mathbf{M} & \equiv \begin{bmatrix} 
m\_{11} & m\_{12} & m\_{13} \\\\
m\_{21} & m\_{22} & m\_{23} \\\\
m\_{31} & m\_{32} & m\_{33} \end{bmatrix} \\\\ 
s \mathbf{M} & = \begin{bmatrix}
s m\_{11} & sm\_{12} & sm\_{13} \\\\
s m\_{21} & sm\_{22} & sm\_{23} \\\\
s m\_{31} & sm\_{32} & sm\_{33} \end{bmatrix} 
\end{align*}
for \\(s \in \mathbb{R}\\)

#### Special matrices

I will point out three special types of matrices:

* \\(\mathbf{0} \in \mathbb{R}^{m \times n}\\) (the "zero matrix"): a matrix with every entry set to zero
* Diagonal matrices: a square (\\(n \times n\\)) matrix with nonzero entries placed only on the diagonal (from upper left to lower right)
* \\(\mathbf{I} \in \mathbb{R}^{n \times n}\\) (the "identity matrix"): a diagonal matrix with every entry on the diagonal set to 1 

#### The cross product

The cross product operator (\\(\times\\)) does not fit neatly into matrix/vector arithmetic
and linear algebra, because it applies only to vectors in \\(\mathbb{R}^3\\).
For two vectors \\(\mathbf{a}, \mathbf{b} \in \mathbb{R}^3\\), \\(\mathbf{a} \times \mathbf{b}\\) yields:

1. A vector orthogonal to \\(\mathbf{a}\\) and to \\(\mathbf{b}\\), assuming that \\(\mathbf{a}\\) and \\(\mathbf{b}\\) are linearly independent (otherwise, \\(\mathbf{a} \times \mathbf{b} = \mathbf{0}\\)) 
2. The negation of \\(\mathbf{b} \times \mathbf{a}\\); stated again, \\(\mathbf{a} \times \mathbf{b} = -\mathbf{b} \times \mathbf{a}\\).
3. A different vector depending on whether the cross product is a _right handed cross product_ or a _left handed cross product_

The cross product is distributive over addition:

\begin{equation*}
\mathbf{a} \times (\mathbf{b} + \mathbf{c}) = \mathbf{a} \times \mathbf{b} + \mathbf{a} \times \mathbf{c}
\end{equation*}

##### Computing the cross product

For the right handed cross product, 
$$
\mathbf{a} \times \mathbf{b} = \begin{bmatrix}
a\_2b\_3 - a\_3b\_2 \\\\
a\_3b\_1 - a\_1b\_3 \\\\
a\_1b\_2 - a\_2b\_1
\end{bmatrix}
$$

The left handed cross product is just the negation of this vector.

#### Inner products

The inner product operation (also called the _dot product_) between two vectors \\(\mathbf{a}\\) and \\(\mathbf{b}\\) is written \\(\lt \mathbf{a}, \mathbf{b}\gt \\), \\(\mathbf{a}^{\textsf{T}}\mathbf{b}\\), or \\(\mathbf{a} \cdot \mathbf{b}\\). The inner product is only defined for vectors of equal dimension and consists of the sum of the products of the corresponding elements from the two vectors. An example is given below:

\begin{align*}
\mathbf{a} & \equiv \begin{bmatrix} a\_1 \\\\ a\_2 \\\\ a\_3 \\\\ a\_4 \end{bmatrix} \\\\
\mathbf{b} & \equiv \begin{bmatrix} b\_1 \\\\ b\_2 \\\\ b\_3 \\\\ b\_4 \end{bmatrix} \\\\
\mathbf{a}^\textsf{T}\mathbf{b} & = a\_1 b\_1 + a\_2 b\_2 + a\_3 b\_3 + a\_4 b\_4
\end{align*}

Some properties of the inner product follow, for real vectors \\(\mathbf{a}, \mathbf{b}, \mathbf{c}\\):

* The dot product is commutative: \\(\mathbf{a} \cdot \mathbf{b} = \mathbf{b} \cdot \mathbf{a}\\)
* The dot product is distributive over vector addition: \\(\mathbf{a} \cdot (\mathbf{b} + \mathbf{c}) = \mathbf{a} \cdot \mathbf{b} + \mathbf{a} \cdot \mathbf{c}\\)
* If \\(\mathbf{a}, \mathbf{b} \ne \mathbf{0}\\), \\(\mathbf{a} \cdot \mathbf{b} = 0 \\) if and only if \\(\mathbf{a}\\) and \\(\mathbf{b}\\) are orthogonal
* \\(\mathbf{a} \times (\mathbf{b} \times \mathbf{c}) = \mathbf{b}(\mathbf{a} \cdot \mathbf{c}) - \mathbf{c}(\mathbf{a} \cdot \mathbf{b})\\)

{% comment %}
Problem: Convert n' * (xd + w x r) to (n' -r x n) * (xd; w). Motivate problem using velocity of a point on a rigid body.

{% endcomment %}

#### Matrix/vector multiplication 

Matrix/vector multiplication is identical to multiple inner (dot) product
operations over the rows of \\(\mathbf{A}\\). Assume we define matrix \\(\mathbf{A}\\) as follows:

\begin{equation*}
\mathbf{A} \equiv \begin{bmatrix} \mathbf{a}\_1 \\\\ \vdots \\\\ \mathbf{a}\_m \end{bmatrix}
\end{equation*}

We can then compute the matix vector product \\(\mathbf{Av}\\) using \\(m\\) inner products:

\begin{equation*}
\mathbf{Av} = \begin{bmatrix} \mathbf{a}\_1\mathbf{v} \\\\ \vdots \\\\ \mathbf{a}\_m\mathbf{v} \end{bmatrix}
\end{equation*} 

As a concrete example, we define:

\begin{align*}
\mathbf{A} & \equiv \begin{bmatrix} a & b & c \\\\ d & e & f \end{bmatrix} \\\\
\mathbf{v} & \equiv \begin{bmatrix} g \\\\ h \\\\ i \end{bmatrix} \\\\
\mathbf{Av} & = \begin{bmatrix} ag + bh + ci \\\\ dg + eh + fi \end{bmatrix}
\end{align*}

#### Matrix/matrix multiplication 

Matrix/matrix multiplication proceeds the same as matrix-vector multiplication
over multiple columns:

\begin{equation}
\mathbf{MN} \equiv \mathbf{M}\begin{bmatrix} \mathbf{n}\_1 & \ldots & \mathbf{n}\_n \end{bmatrix} \equiv \begin{bmatrix} \mathbf{Mn}\_1 & \ldots & \mathbf{Mn}\_n \end{bmatrix} 
\end{equation}

As a concrete example:

\begin{align*}
\mathbf{M} & \equiv 
\begin{bmatrix} a & b \\\\ c & d \\\\ e & f \end{bmatrix} \\\\
\mathbf{n}\_1 & \equiv \begin{bmatrix} g \\\\ j \end{bmatrix}  \\\\
\mathbf{n}\_2 & \equiv \begin{bmatrix} h \\\\ k \end{bmatrix}  \\\\
\mathbf{n}\_3 & \equiv \begin{bmatrix} i \\\\ l \end{bmatrix}  \\\\
\mathbf{N} & \equiv \begin{bmatrix} \mathbf{n}\_1 & \mathbf{n}\_2 & \mathbf{n}\_3 \end{bmatrix} \\\\
\mathbf{Mn}\_1 & \equiv \begin{bmatrix} ag + bj \\\\ cg + dj \\\\ eg + fj \end{bmatrix} \\\\
\mathbf{Mn}\_2 & \equiv \begin{bmatrix} ah + bk \\\\ ch + dk \\\\ eh + fk \end{bmatrix} \\\\
\mathbf{Mn}\_3 & \equiv \begin{bmatrix} ai + bl \\\\ ci + dl \\\\ ei + fl \end{bmatrix} \\\\
\mathbf{MN} & \equiv \begin{bmatrix} ag+bj & ah+bk & ai+bl \\\\ cg+dj & ch+dk & ci + dl\\\\ eg+fj & eh+fk & ei+fl \end{bmatrix}
\end{align*}

Note that __matrix multiplication is not commutative__: \\(\mathbf{AB} \ne \mathbf{BA}\\) (generally), even when the dimensions are compatible.

#### Outer products

The _outer product_ \\(\mathbf{ab}^\mathsf{T}\\) of two vectors \\(\mathbf{a} \in \mathbb{R}^m\\) and \\(\mathbf{b} \in \mathbb{R}^n\\) is always defined and represents a matrix/matrix multiplication operation between a \\(m \times 1\\) and a \\(1 \times n\\) matrix; in other words, normal matrix/matrix multiplication rules apply.

#### Matrix transposition

The transpose of a matrix is defined as follows:

$$
A\_{ij}^\mathsf{T} = A\_{ji}
$$

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/e/e4/Matrix_transpose.gif" description="A depiction of the matrix transpose operation." %}

where the operator \\(^\mathsf{T}\\) indicates transposition. This definition implies that an \\(m \times n\\) matrix becomes an \\(n \times m\\) matrix. If \\(\mathbf{A} = \mathbf{A}^\textsf{T}\\) we say that \\(\mathbf{A}\\) is a _symmetric matrix_. 

The following properties apply to matrix transposition for matrices \\(\mathbf{A}\\) and \\(\mathbf{B}\\):

* \\((\mathbf{A}^\textsf{T})^\mathsf{T} = \mathbf{A}\\)
* \\((\mathbf{A}+\mathbf{B})^\mathsf{T} = \mathbf{A}^\mathsf{T} + \mathbf{B}^\mathsf{T}\\)
* \\((\mathbf{AB})^\mathsf{T} = \mathbf{B}^\mathsf{T}\mathbf{A}^\mathsf{T}\\)
* If \\(\mathbf{A}\\) has only real entries, then \\(\mathbf{A}^\textsf{T}\mathbf{A}\\) is a positive semi-definite matrix (see below).
* \\(\mathbf{A}\mathbf{A}^\textsf{T}\\) is a symmetric matrix.

#### Matrix inversion

The inverse of a matrix \\(\mathbf{A}\\), written \\(\mathbf{A}^{-1}\\), is characterized by the following properties:

\begin{align}
\mathbf{AA}^{-1} & = \mathbf{I} \\\\
\mathbf{A}^{-1}\mathbf{A} & = \mathbf{I}
\end{align}

The inverse exists only if \\(\mathbf{A}\\) is square and is _non-singular_. Singularity can be determined multiple ways (only two are listed below):

* If the determinant of the matrix is zero, the matrix is singular. The determinant can be computed using LU factorization (see below).
* If one or more of the singular values of the matrix is zero, the matrix is singular. The singular values can be computed using the singular value decomposition (see below).
 
The following properties apply to matrix inversion for matrices \\(\mathbf{A}\\) and \\(\mathbf{B}\\):

* \\((\mathbf{A}^\textsf{-1})^\mathsf{-1} = \mathbf{A}\\)
* \\((\mathbf{A}^\textsf{T})^{-1} = (\mathbf{A}^{-1})^\textsf{T}\\)
* \\((\mathbf{AB})^{-1} = \mathbf{B}^{-1}\mathbf{A}^{-1}\\)

In numerical linear algebra, you almost never need to explicitly form the
inverse of a matrix and __you should avoid explicitly forming the inverse
whenever possible__: _the solution obtained by computing \\(\mathbf{x} = \mathbf{A}^{-1}\mathbf{b}\\) is considerably slower than back/forward substitution-based methods_ (using, e.g., Cholesky factorization, LU factorization, etc.) for solving \\(\mathbf{Ax} = \mathbf{b}\\).

{% comment %}
1. What is ((AB)^T)^{-1}?
2. Show that A'*inv(A*A')*A = I, for square, non-singular A
{% endcomment %}


### Empty matrices and vectors
Empty matrices (matrices with one or more dimensions equal to zero) are often useful. They allow formulas,
optimization, etc. to be used without breaking even when the inputs to the
problem are empty: it does not become necessary to use special logic to
handle such corner cases. 

Given scalar \\(s \in \mathbb{R}\\), empty matrix \\(\mathbf{M} \in \mathbb{R}^{m \times n}\\) (with one of \\(m,n\\) equal to zero), empty matrix \\(\mathbf{E} \in \mathbb{R}^{0 \times m}\\), and empty matrix \\(\mathbf{F} \in \mathbb{R}^{n \times 0}\\), we have the following rules:

1. \\(s \mathbf{M} = \mathbf{M}\\)
2. \\(\mathbf{M} + \mathbf{M} = \mathbf{M} \\)
3. \\(\mathbf{EM} = \mathbf{F}^\mathsf{T}\\)
4. \\(\mathbf{MF} = \mathbf{E}^\mathsf{T}\\)
5. \\(\mathbf{E}\mathbf{F}^\mathsf{T} = \mathbf{0}\\)

### Computational considerations for matrix-multiplication associativity

Matrix multiplication _is_ associative: \\((\mathbf{AB})\mathbf{C} = \mathbf{A}(\mathbf{BC})\\). The amount of computation required can be very different in the two cases.
Assume that \\(\mathbf{A} \in \mathbb{R}^{i \times j}, \mathbf{B} \in \mathbb{R}^{j \times k}, \mathbf{C} \in \mathbb{R}^{k \times m}\\). Depending on the order of operation, two very different flop counts are possible:

1. \\((\mathbf{A}\mathbf{B})\mathbf{C} = O(ijk) + O(ikm)\\)
2. \\(\mathbf{A}(\mathbf{B}\mathbf{C}) = O(jkm) + O(ijm)\\)

Now consider the following variable instances: \\(i = 1, j = 2, k = 3, m = 4 \\). The asymptotic number of operations in Case (1) will be on the order of 15 flops and in Case (2) will be 32 flops. Takeaway: consider your multiplication ordering. 

Note that in the case of multiplying a chain of matrices and then a vector:

\begin{equation}
\mathbf{ABv}
\end{equation}

One always wants to do the vector multiplication first:

\begin{equation}
\mathbf{ABv} = \mathbf{A}(\mathbf{Bv})
\end{equation}

In many applications, only a few matrices may be multiplied at
one time, meaning that a little logic can be used to determine the order of operations. For longer chains of matrices, one likely wants to use [dynamic programming to determine the optimal multiplication order](https://en.wikipedia.org/wiki/Matrix_chain_multiplication).

### Linear algebra

#### Orthogonality
Vectors \\(\mathbf{u}\\) and \\(\mathbf{v}\\) are orthogonal if their dot (inner) product is zero.

_Orthogonal_ matrices are very convenient because they possess the property that their inverse is equal to their transpose:

\begin{equation}
\mathbf{A}^\mathsf{T} = \mathbf{A}^{-1} \textrm{ if } \mathbf{A} \textrm{ orthogonal}
\end{equation}

 Computationally, this means that the inverse can be computed quickly and robustly. An orthogonal matrix has the following properties:

1. The determinant of the matrix is \\(\pm 1\\)
2. The dot product of any two rows \\(i \ne j\\) of the matrix is zero
3. The dot product of any two columns \\(i \ne j\\) of the matrix is zero

#### Positive and negative definiteness

A symmetric matrix \\(\mathbf{A}\\) is _positive definite_ if:

\begin{equation}
\mathbf{x}^\mathsf{T}\mathbf{A}\mathbf{x} \gt 0
\end{equation}

for any \\(\mathbf{x} \in \mathbb{R}^n\\) such that \\(\mathbf{x} \ne \mathbf{0}\\). This condition is equivalent to saying that a matrix is positive definite if all of its [eigenvalues](https://en.wikipedia.org/wiki/Eigenvalues_and_eigenvectors) are positive; eigenvalues are readily computable with GNU Octave/Matlab (using `eig`) and with most libraries for numerical linear algebra. If \\(\mathbf{A}\\) is not positive definite and instead,

\begin{equation}
\mathbf{x}^\mathsf{T}\mathbf{A}\mathbf{x} \ge 0
\end{equation}

for any \\(\mathbf{x} \in \mathbb{R}^n\\) such that \\(\mathbf{x} \ne \mathbf{0}\\), we say that the matrix is _positive semi-definite_. This condition is equivalent to saying that a matrix is positive semi-definite if all of its eigenvalues are
non-negative. 

Similarly, a matrix is _negative definite_ if all of its eigenvalues are
strictly negative and _negative semi-definite_ if all of its eigenvalues
are non-positive. If none of these conditions hold- \\(\mathbf{A}\\) has both positive and negative eigenvalues- we say that the matrix is _indefinite_.

##### Checking positive-definiteness

The fastest general way to check for positive-definiteness is using the
Cholesky factorization. If the Cholesky factorization succeeds, the matrix
is positive definite. This approach for checking positive definiteness
is a (significant) constant factor faster than approaches that compute eigenvalues.

##### Applications of definiteness

Definite matrices have many applications in engineering applications.
As one example, if the Hessian matrix of an objective function is 
positive semi-definite, the function is convex and admits solution
via robust convex optimization codes. As another example, Lyapunov
stability analysis requires negative definiteness of the time derivative
of a Lyapunov function candidate. 

We often wish to avoid indefinite matrices. For example, quadratic programming
with indefinite matrices is NP-hard, while it is polynomial time solvable
with definite matrices. 

#### Factorizations

I like to consider matrix factorizations in ascending order of computational expense. Correlated with computational expense is numerical robustness. A list of the factorizations follows:

Factorization | Flops | Applicability
---- | ---- | ----
Cholesky factorization | \\(n^3/3\\) | Positive-definite matrices only
LDL\\(^\mathsf{T}\\) factorization | \\(n^3/2 + O(n^2)\\) | Symmetric matrices only
LU factorization | \\(2n^3/3\\) | Non-singular matrices (no least squares)
QR factorization | \\(4n^3/3\\) | Singular and non-singular matrices (least squares ok) 
Singular value decomposition | \\(8n^3/3\\) | Singular and non-singular matrices (least squares ok) 

#### Nullspace

The nullspace \\(\mathbf{R}\\) of matrix \\(\mathbf{A}\\) is a nonzero matrix
such that: 

\begin{equation}
\mathbf{AR} = \mathbf{0}
\end{equation}

The nullspace of a matrix \\(\mathbf{A}\\) can be determined using the 
rightmost columns of \\(\mathbf{V}\\) (from the singular value decomposition 
of \\(\mathbf{A}\\)) that correspond to the zero singular values from 
\\(\mathbf{\Sigma}\\) (also from the SVD of \\(\mathbf{A}\\)).

Nullspaces are particularly good for optimization and least squares problems.
For example, the nullspace allows optimizing along multiple criteria in
hierarchical fashion.

### Matrix calculus

If \\(\mathbf{a}, \mathbf{b}\\) are functions, then the derivative of (denoted by prime ') the matrix multiplication operation is: \\(\mathbf{a}^\mathsf{T}\mathbf{b} = {\mathbf{a}'}^{\mathsf{T}}\mathbf{b} + \mathbf{a}^\mathsf{T} \cdot \mathbf{b}'\\)

#### Gradient

The gradient of a function \\(f(\mathbf{x})\\), where \\(\mathbf{x} \in \mathbb{R}^n\\) is the \\(n\\)-dimensional vector:

\begin{equation}
\nabla f\_{\mathbf{x}} \equiv
\begin{bmatrix}
\frac{\partial f}{\partial x\_1} \\\\
\vdots \\\\
\frac{\partial f}{\partial x\_n} \\\\
\end{bmatrix}
\end{equation}

#### Hessian
The Hessian matrix of the same function is the \\(n \times n\\) matrix of second order partial derivatives:

\begin{equation}
\nabla f\_{\mathbf{xx}} \equiv 
\begin{bmatrix}
\frac{\partial^2 f}{\partial x\_1^2} & \ldots & \frac{\partial^2 f}{\partial x\_1 \partial x\_n}\\\\
\vdots \\\\
\frac{\partial^2 f}{\partial x\_n \partial x\_1} & \ldots & \frac{\partial^2 f}{\partial x\_n^2}\\\\
\end{bmatrix}
\end{equation}

Given that \\(\frac{\partial^2 f}{\partial x\_i \partial x\_j} = \frac{\partial^2 f}{\partial x\_j \partial x\_i}\\), the Hessian matrix is symmetric (this helps with debugging and can reduce computation when constructing the matrix).

{% comment %}
problem: compute derivative of x'*A*x with respect to function x
1) what should be the dimension of the derivative
2) compute the second derivative of x'*A*x with respect to function x
{% endcomment %} 


#### Jacobian

The Jacobian matrix of a function with vector outputs is the partial derivative
of each function output dimension taken with respect to the partial derivative of each function input dimension. Let us examine a contrived function, \\(f : \mathbb{R}^3 \to \mathbb{R}^2\\); \\(f(.)\\) might represent a vector flow for
points in three dimensional Cartesian space. The Jacobian of \\(f(.)\\) is then:

\begin{equation*}
\frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} 
\frac{\partial f\_1}{\partial x\_1} & \frac{\partial f\_1}{\partial x\_2} & \frac{\partial f\_1}{\partial x\_3} \\\\
\frac{\partial f\_2}{\partial x\_1} & \frac{\partial f\_2}{\partial x\_2} & \frac{\partial f\_2}{\partial x\_3} 
\end{bmatrix} 
\end{equation*}

Just like the standard derivative, the Jacobian matrix gives the instantaneous change in \\(f(.)\\) at \\(\mathbf{x}\\). 

### Least squares problems

Least squares problems are ubiquitous in science and engineering applications.
Solving a least squares problem finds a line (or plane or hyperplane,
in higher dimensions) that minimizes the sum of the squared distance from a
set of points to the line/plane/hyperplane. Another way of saying this is
that least squares seeks to minimize the residual error, i.e., \\(||\mathbf{A}\mathbf{x} - \mathbf{b}||\\). 

{% include image.html url="http://www.datavis.ca/papers/koln/figs/grsp2.gif" description="A depiction of least squares as a mechanical device. Springs are attached from each point to a rod. The amount of force increases quadratically with the distance of each point to the rod." %}

Clearly, if \\(\mathbf{A}\\) is square and non-singular, the solution is \\(\mathbf{x} = \mathbf{A}^{-1}\mathbf{b}\\). What if \\(\mathbf{A} \in \mathbb{R}^{m \times n}\\), where \\(m \neq n\\)? Assume that each row of \\(\mathbf{A}\\) is linearly independent (i.e., \\(\mathbf{A}\\) has full row rank) for now. If \\(m \gt n\\), then there are more
equations than variables and the problem is _overdetermined_; we expect \\(||\mathbf{Ax} - \mathbf{b}||\\) to be nonzero. If \\(m \lt n\\), then there are more
variables than unknowns and the problem is _underdetermined_; we expect there
to be multiple (infinite) assignments to \\(\mathbf{x}\\) that make \\(||\mathbf{Ax} - \mathbf{b}|| = 0\\).

##### The Moore-Penrose pseudo-inverse

_Two_ least squares problems become evident: (1) select \\(\mathbf{x}\\) such that  \\(||\mathbf{Ax} - \mathbf{b}||\\) is minimized. If \\(||\mathbf{Ax} - \mathbf{b}|| = 0\\), then (2) select the solution that minimizes \\(||\mathbf{x}||\\). Both solutions can be obtained using the Moore-Penrose pseudo-inverse (which is denoted using the operator \\(\ ^+\\), which has some of the properties of the inverse (I'll only list a few below):

1. \\(\mathbf{A}\mathbf{A}^+ = \mathbf{I}\\)
2. \\(\mathbf{A}^+\mathbf{A} = \mathbf{I}\\)
3. If \\(\mathbf{A}\\) is invertible, then \\(\mathbf{A}^+ = \mathbf{A}^{-1}\\).
4. \\(\mathbf{0}^{-1} = \mathbf{0}^\mathsf{T}\\).
5. \\((\mathbf{A}^+)^+ = \mathbf{A}\\).

The _left pseudo-inverse_ follows:

\begin{equation}
\mathbf{A}^+ = {(\mathbf{A}^\mathsf{T}\mathbf{A})}^{-1}\mathbf{A}^\mathsf{T}
\end{equation}

This pseudo-inverse constitutes a _left inverse_ (because \\(\mathbf{A}^+\mathbf{A} = \mathbf{I}\\)), while the _right pseudo-inverse_ (below)
constitutes a _right inverse_ (because \\(\mathbf{A}\mathbf{A}^+ = \mathbf{I}\\)). See [Wikipedia](https://en.wikipedia.org/wiki/Inverse_function#Left_and_right_inverses) for an accessible description of left and right inverses. 

\begin{equation}
\mathbf{A}^+ = \mathbf{A}^\mathsf{T}{(\mathbf{A}\mathbf{A}^\textsf{T})}^{-1}
\end{equation}

{% comment %}
Derivation 1:

Ax = b
A'Ax = A'*b
x = inv(A*A')*A'*b

Derivation 2:
x'*A' = b'
x'*A'*A = b'*A
x' = b'*A*inv(A'*A)

x = A'*inv(AA')*b
{% endcomment %}

##### Solving least squares problems using the singular value decomposition

Still working under the assumption that \\(\mathbf{A}\\) is of full row rank,
we can also use the singular value decomposition to solve least squares 
problems.

Recall that the singular value decomposition of \\(\mathbf{A} = \mathbf{U}\Sigma^{-1}\mathbf{V}^{\mathsf{T}}\\). \\(\mathbf{U}\\) and \\(\mathbf{V}\\) are both orthogonal, which means that \\(\mathbf{U}^{-1} = \mathbf{U}^{\mathsf{T}}\\) and \\(\mathbf{V}^{-1} = \mathbf{V}^{\mathsf{T}}\\). From the identity above, \\(\mathbf{A}^{-1} = \mathbf{V}\Sigma^{-1}\mathbf{U}^{\mathsf{T}}\\). For non-square \\(\mathbf{A}\\), \\(\mathbf{A}^+ = \mathbf{V}\mathbf{\Sigma}^+{\mathbf{U}}^\mathsf{T}\\), where \\(\Sigma^+\\) will be defined as follows:

\begin{equation}
\Sigma\_{ij}^+ \leftarrow \begin{cases}\frac{1}{\Sigma\_{ij}} & \textrm{if } i = j, \\\\
0 & \textrm{if } i \ne j. \end{cases}
\end{equation}

#### Regularization

It's too much to ask that \\(\mathbf{A}\\), \\(\mathbf{A}^\mathsf{T}\mathbf{A}\\), or \\(\mathbf{A}\mathbf{A}^\textsf{T}\\) be always invertible. Even if you 
_know_ the matrix is invertible (based on some theoretical properties), this does not mean that \\(\mathbf{A}\\) et al. will be well conditioned. And singular
matrices arise in least squares applications when two or more equations are 
linearly dependent (in other words, regularly).

{% comment %}
Challenge: how much computation does it take to determine a linearly 
independent set of equations for least squares?
{% endcomment %}

_Regularization_ allows one to compute least squares solutions with numerical
robustness.

##### Using regularization with the singular value decomposition

The formula for computing \\(\mathbf{\Sigma}^+\\) can be updated for a numerically robust solution:

\begin{equation}
\Sigma\_{ij}^+ \leftarrow \begin{cases}\frac{1}{\Sigma\_{ij}} & \textrm{if } i = j \textrm{ and } \Sigma\_{ij} > \epsilon, \\\\
0 & \textrm{if } i \ne j \textrm{ or } \Sigma\_{ij} \le \epsilon. \end{cases}
\end{equation}

\\(\epsilon\\) can be set using _a priori_ knowledge or using the strategy
used in [LAPACK](http://www.netlib.org/lapack/)'s library:

\begin{equation}
\epsilon = \epsilon_{\textrm{mach}} \cdot \max{(m,n)} \cdot \max\_{i,j} \Sigma\_{ij} 
\end{equation} 


##### Tikhonov regularization

A very simple form of regularization is known as _Tikhonov Regularization_
and, in statistics applications, as _Ridge Regression_. Consider the system of linear equations:

\begin{equation}
\mathbf{A}\mathbf{x} = \mathbf{b}
\end{equation}

for \\(\mathbf{A} \in \mathbb{R}^{n \times n}\\) and \\(\mathbf{x}, \mathbf{b} \in \mathbb{R}^{n \times 1}\\). The
matrix \\(\mathbf{A} + \epsilon\mathbf{I}\\), where \\(\mathbf{I}\\) is the identity matrix will always be invertible for sufficiently large \\(\epsilon \ge 0\\). Tikhonov Regularization solves the nearby system \\((\mathbf{A} + \epsilon\mathbf{I})\mathbf{x} = \mathbf{b}\\) for \\(\mathbf{x}\\).

The obvious question at this point: what is the optimal value of \\(\epsilon\\)? If \\(\epsilon\\) is too small, the relevant factorization algorithm (e.g., Cholesky factorization, LU factorization) will fail with an error (at best). If \\(epsilon\\) is too large, the residual error \\(||\mathbf{Ax} - \mathbf{b}||\\) will be greater than is necessary. The optimal value of \\(\epsilon\\) can be
computed with a singular value decomposition, but- one might as well just use the result from the SVD to compute a regularization solution (as described above)- if one goes down this computationally expensive route.

An effective \\(\epsilon\\) will keep the condition number (the ratio of
largest to smallest singular values) relatively small: \\(10^6\\) or so. An 
example of a quick and dirty way to compute \\(\epsilon\\) is:

\begin{equation}
\epsilon = \epsilon_{\textrm{mach}} \cdot \max{(m,n)} \cdot ||\mathbf{A}||\_\infty
\end{equation} 

where \\(\epsilon_{\textrm{mach}}\\) is machine epsilon and

\begin{equation}
||\mathbf{A}||\_\infty = \max_{i \in m, j \in n} |A_{ij}|
\end{equation} 

