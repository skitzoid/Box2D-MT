
# Box2D-MT

**Box2D with multithreading.**

This project improves performance of Box2D by executing the world's step function on multiple threads.

## Usage

Familiarity with Box2D is assumed.

### Task Executor

A task executor runs the tasks submitted to it by the world. You can use your game engine's thread pool by
implementing b2TaskExecutor, or you can use the provided b2ThreadPoolTaskExecutor.

```
// Create a task executor
b2ThreadPoolTaskExecutor executor;

// Pass the executor to the world's step function
world.Step(timeStep, velocityIterations, positionIterations, executor);
```

### Multithreaded Callbacks

Box2D-MT adds 4 pure virtual functions to b2ContactListener, which correspond to the Box2D functions.

```
virtual bool BeginContactImmediate(b2Contact* contact, uint32 threadIndex) = 0;
virtual bool EndContactImmediate(b2Contact* contact, uint32 threadIndex) = 0;
virtual bool PreSolveImmediate(b2Contact* contact, const b2Manifold* oldManifold, uint32 threadIndex) = 0;
virtual bool PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse, uint32 threadIndex) = 0;
```

These are called by multiple threads simultaneously, so your implementation must be thread-safe. If your function
returns true then the arguments will be saved and used to invoke the corresponding non-immediate function after the
multithreaded work is finished.

Immediate functions can filter events to avoid the overhead of a deferred call. This is more important for PreSolve
and PostSolve because more contacts receive those calls every step. However, you should start with an implementation
that returns true for callbacks that you use and false for callbacks you don't use. For example:

```
virtual bool BeginContactImmediate(b2Contact* contact, uint32 threadIndex) override
{
    // Call BeginContact for every contact.
    return true;
}
virtual bool EndContactImmediate(b2Contact* contact, uint32 threadIndex) override
{
    // Call EndContact for every contact.
    return true;
}
virtual bool PreSolveImmediate(b2Contact* contact, const b2Manifold* oldManifold, uint32 threadIndex) override
{
    // Never call PreSolve
    return false;
}
virtual bool PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse, uint32 threadIndex) override
{
    // Never call PostSolve
    return false;
}
```

If your performance is acceptable then you should leave it like that. Otherwise, if profiling shows significant time
spent in deferred callbacks then you can move work into immediate callbacks, but only if you can do so without causing
data races. Read the function documentation to see what data is safe to access.

## Reproducibility

With Box2D, running the same build on the same machine with the same floating point environment will produce the
same results every time. This is also true for Box2D-MT, but the order of multithreaded callbacks is indeterminate.
If you rely on reproducibility for features like game replays or multiplayer, then you must ensure that your
immediate callbacks produce consistent results regardless of the order in which they're called. An easy solution
is to do all order-dependent work in deferred callbacks.

Box2D-MT is tested for reproducibility on every test in the testbed, but you still might discover inconsistencies;
if so, please file an issue.

## Thread Error Detection

I use valgrind DRD to test for data races. These kind of tools can't detect atomic variables so they generate false
positives on conflicting atomic loads and stores. Box2D-MT can be configured to make DRD ignore specific atomic
variables, which eliminates the false positives. This can help when testing the thread-safety of your program with
DRD while using b2ThreadPoolTaskExecutor. The DRD configuration also enables symbols, to identify the source of any
problems in Box2D-MT.

See [Building.md](https://github.com/jhoffman0x/Box2D-MT/blob/master/Building.md) for details on using the DRD configurtion.

Please file an issue if DRD detects any errors in Box2D-MT while running a DRD build, even if they're false positives.


![Box2D Logo](http://box2d.org/images/icon.gif)

# Box2D

**Box2D is a 2D physics engine for games.**

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

Please see [Building.md](https://github.com/erincatto/Box2D/blob/master/Building.md) to learn how to build Box2D and run the testbed.

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

## Documentation
You can find documentation related to the project in the [documentation page](http://box2d.org/documentation/) and in the [documentation folder](https://github.com/erincatto/Box2D/tree/master/Box2D/Documentation) in GitHub
- [User manual](http://box2d.org/manual.pdf)
- [Doxygen document](https://github.com/erincatto/Box2D/blob/master/Box2D/Documentation/Doxyfile) with code comments
- [subreddit](https://www.reddit.com/r/box2d/)
- [Discord server](https://discord.gg/NKYgCBP)
- [User forum (legacy)](http://box2d.org/forum/)

You can also visit the project wiki where you will find the [FAQ](https://github.com/erincatto/Box2D/wiki/FAQ)'s page

## License

Box2D is developed by Erin Catto, and has the [zlib license](http://en.wikipedia.org/wiki/Zlib_License). While the zlib license does not require acknowledgement, we encourage you to give credit to Box2D in your product.
