# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- svg - Handles drawing objects created in geometry2d and se2d into an svg file
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Create a differential drive robot with 2 wheels and varying properties
- circle - Creates a circle object, with the ability to create a circle based on given points along the circle

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
    - Propose three different designs for implementing the ~normalize~ functionality
        - To make normalize a funciton of the Vector struct and have it return a new vector
        - To make normalize a function that takes in a vector and returns a Vector2D
        - To make normalize a function that takes in a vector and returns an array of doubles

    - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
        - Making it a member function
            - Pro: Allows for function chaining if other functions were added
            - Con: Runs a higher risk of accidentally manipulating the struct (violates *C.4*)
        - Making it a non-member function and return a Vector2d
            - Pro: No risk of modifying the struct
            - Pro: Still allows for chaining of functions
        - Making it a non-member function that returns an array
            - Pro: Doesn't allocate memory for a vector automatically
            - Pro: No risk of modifying the struct
            - Con: Doesn't allow for function chaining

    - Which of the methods would you implement and why?
        &ensp; I implimented the second option because it allows for function chaining if more functions are added and removes the risk of modifying the struct. Both making it the safest option and the most future-proof option.

2. What is the difference between a class and a struct in C++?
    &ensp; A class has a default access level of private for each member, and a struct has a default access level of public

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

    &ensp; We made Transform2D a class because we needed its member variables to be private and can't allowthe user to modify its members at will, and because we needed at least one member was non-public, *C.8* states that we should use a class instead of a struct. *C.2* is another reason why Transform2D should be a class instead of a struct, because the transformation matrix **T** varies based on the x, y, and omega input.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?  
    &ensp; ES:23 states that one should prefer the `{}` syntax, this is beacuse the `{}` syntax tends to be "simpler, more general, less ambiguous, and safer than than for other forms of initialization."

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
   &ensp; `inv()` is declared const because it doesn't modify anything in a Transform2D object, it just creates another object based on the Transform2D it's called on. Though `*=` does modify the Transform 2D object it's called on so it's declared as const. These declarations follow rule *Con.2*.