Why numerical methods?

Analytical and numerical methods are two primary approaches to solving inverse kinematics (IK) problems, each with distinct advantages and use cases. Analytical methods provide exact solutions through closed-form equations, 
making them highly efficient and fast. They are best suited for simpler robotic systems with fewer degrees of freedom,
where the mathematical models are straightforward. However, they struggle with complex, redundant, or highly non-linear systems, 
as deriving explicit solutions can be exceedingly difficult or impossible.

In contrast, numerical methods offer a more flexible and robust approach, 
capable of handling a wide variety of robotic configurations, 
including those with redundancy and non-linear constraints. 
These methods iteratively approximate solutions, making them suitable for complex systems where analytical solutions are impractical.
Although typically slower due to their iterative nature, numerical methods excel in adaptability, generality, and their ability to incorporate additional constraints and optimization criteria, 
such as minimizing energy use or avoiding obstacles. This makes them indispensable for modern, sophisticated robotic applications where precision and adaptability are paramount.

Gradient Descent Parameter 

![Parameter for Gradient Descent](https://github.com/user-attachments/assets/aa20a266-c3f5-4081-91c9-fa37658f4733)

Muti-goals Based Optimization

![Muti-goals Based Optimization](https://github.com/user-attachments/assets/5136a2fc-a27e-477b-b181-089ae7e1b060)
