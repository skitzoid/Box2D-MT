
# Box2D-MT
Box2D-MT is an experimental library that adds multi-threading to Box2D.

It has no external dependencies and aims to be no less deterministic than Box2D.

Before using Box2D-MT you should be familiar with Box2D and multi-threading.

# Quick Start
To enable multi-threading, create a thread-pool and pass it to your world's constructor.

```
b2ThreadPool tp;
b2World world(gravity, &tp);
```

Now when you step the world, the work will be split among all of your cpu's cores.
The thread pool must remain in scope until the world is destroyed.

# Callback Thread-Safety
Box2D uses various callback functions to let the user respond to events. In Box2D-MT, some of these functions
are called from multiple threads at the same time, so your callback implementations must practice thread-safety.

Here are some guidelines for relevant callbacks (these are a work in progress and probably not 100% accurate):

b2ContactFilter::ShouldCollide: Treat all bodies and joints as read-only. Don't access any contacts.

b2ContactListener::BeginContact, b2ContactListener::EndContact, b2ContactListener::Presolve:
Treat all bodies and joints as read-only. The provided contact is safe for reads and writes. Don't access any
other contacts.

b2ContactListener::PostSolve: The provided contact is safe for reads and writes. A dynamic body that is part of
the provided contact is safe for reads and writes, except for the body's flags, which are read-only. Joints
attached to the dynamic body are safe for reads and writes. A static body that is part of the provided contact
is read-only, except for it's flags, which must not be accessed at all. Don't access any other bodies, joints,
or contacts.

A simple way of avoiding these dangers is to queue up events and process them on a single thread when the world
is done stepping. There's no need for thread-safe data-structures or mutexes (aqcuiring a mutex in a callback can
kill performance <sup>[*citation needed*]</sup>). You can use an array of per thread data of size b2_maxThreads, and use b2GetThreadId() to index
the array.

A later version of Box2D-MT should provide an option to automatically defer callbacks.



![Box2D Logo](http://box2d.org/images/icon.gif)

# Box2D 

**Box2D is a 2D physics engine for games.**

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

Please see `Building.txt` to learn how to build Box2D and run the testbed.

## Demos

To run the demos, set `Testbed` as your startup project and press <kbd>F5</kbd>. Some test bed commands are:

- <kbd>r</kbd> to reset the current test
- <kbd>SPACE</kbd> to launch a bomb
- <kbd>&larr;</kbd> <kbd>&rarr;</kbd> keys to pan
- <kbd>x</kbd> and <kbd>z</kbd> to zoom in/out
- use the mouse to click and drag objects

## Contributing

Please do not submit pull requests with new features. Instead, please file an issue first for discussion. For bugs, I prefer detailed bug reports over pull requests.

## Features

### Collision
- Continuous collision detection
- Contact callbacks: begin, end, pre-solve, post-solve
- Convex polygons and circles
- Multiple shapes per body
- One-shot contact manifolds
- Dynamic tree broadphase
- Efficient pair management
- Fast broadphase AABB queries
- Collision groups and categories

### Physics
- Continuous physics with time of impact solver
- Persistent body-joint-contact graph
- Island solution and sleep management
- Contact, friction, and restitution
- Stable stacking with a linear-time solver
- Revolute, prismatic, distance, pulley, gear, mouse joint, and other joint types
- Joint limits, motors, and friction
- Momentum decoupled position correction
- Fairly accurate reaction forces/impulses

### System
- Small block and stack allocators
- Centralized tuning parameters
- Highly portable C++ with no use of STL containers

### Testbed
- OpenGL with GLFW
- Graphical user interface with imgui
- Easily switch between tests using GUI
- Test framework for easily adding new tests
- Mouse picking and the bomb!
- CMake build system files

### Documentation
- User manual
- Doxygen document with code comments
- Active user forum

## License

Box2D is developed by Erin Catto, and has the [zlib license](http://en.wikipedia.org/wiki/Zlib_License). While the zlib license does not require acknowledgement, we encourage you to give credit to Box2D in your product.
