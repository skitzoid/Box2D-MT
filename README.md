
# Box2D-MT

**Box2D with multithreading.**

This project improves performance of Box2D by executing the world's step
function on multiple threads.

## Usage

Familiarity with Box2D is assumed. These are the main differences.

### Task Executor

A task executor runs the tasks submitted to it by the world.

```
// Create a task executor
b2ThreadPoolTaskExecutor executor;

// Pass the executor to the world's step function
world.Step(timeStep, velocityIterations, positionIterations, executor);
```

Box2D-MT comes with a thread pool task executor, but you could use your
game engine's thread pool instead by implementing b2TaskExecutor.

### Multithreaded Callbacks

Box2D-MT adds 4 pure virtual functions to b2ContactListener, which correspond to
the Box2D functions.

```
/// This lets you process and filter BeginContact callbacks as they arise from multiple threads.
virtual bool BeginContactImmediate(b2Contact* contact, uint32 threadId) = 0;

/// This lets you process and filter EndContact callbacks as they arise from multiple threads.
virtual bool EndContactImmediate(b2Contact* contact, uint32 threadId) = 0;

/// This lets you process and filter PreSolve callbacks as they arise from multiple threads.
virtual bool PreSolveImmediate(b2Contact* contact, const b2Manifold* oldManifold, uint32 threadId) = 0;

/// This lets you process and filter PostSolve callbacks as they arise from multiple threads.
virtual bool PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse, uint32 threadId) = 0;
```

(See b2WorldCallbacks.h for more detailed comments.)

These are called by multiple threads simultaneously, so your implementation must
be thread-safe.

If your function returns true then the arguments will be saved and used to invoke the
corresponding non-immediate function after the multithreaded work is finished.

Immediate functions can filter events to avoid the overhead of a deferred call.
This is more important for PreSolve and PostSolve because more contacts receive
those calls every step.

You should start with an implementation that simply returns true for callbacks that
you use and false for callbacks you don't use.

For example:

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

If your performance is acceptable then you should do all work in deferred
callbacks for the sake of simplicity.

Otherwise, if profiling shows significant time spent in deferred callbacks,
or in Box2D-MT's processing of deferred callback arguments, then it could help
to move work into immediate callbacks (while being careful about thread safety).

I'm open to suggestions on changes to the contact listener interface, so create
an issue if you want to discuss it.

## Time of Impact (TOI) Changes

Since TOI is still processed on a single thread, it's important that it doesn't
do more work than necessary. I found two ways to address that.

### Finer-grained Continuous Collision Detection (CCD) Control

In Box2D, enabling CCD means you get TOI events for all contacts between dynamic
and non-dynamic bodies, which can be expensive.

If your static body only has a few thin shapes that are subject to tunneling, and
many thick shapes that are not, then most of that expense is unnecessary.

Box2D-MT lets you avoid that expense by marking specific fixtures as thick walls.

```
/// Set whether this fixture is treated like a thick wall for continuous collision detection.
/// Note: thick walls only get TOI events for contacts with bullet bodies.
void SetThickWall(bool flag);

/// Is this fixture treated like a thick wall for continuous collision detection?
bool IsThickWall() const;
```

You can also set isThickWall in the fixture def before you create the fixture.

### Partitioning Contacts Based on TOI Eligibility

Box2D's SolveTOI iterates over every contact in the world to find a TOI event,
possibly visiting many contacts that are not eligible for TOI (because, for example,
they are between two non-bullet dynamic bodies, or one of the fixtures is a sensor).

Visiting these contacts can take up a significant portion of the step.

Box2D-MT avoids this overhead by evaluating TOI eligibility when the contact is
created, and partitioning the contact list so that TOI eligible contacts come
before ineligible contacts. Then SolveTOI only iterates over eligible contacts.

This shifts the cost into functions that are called infrequently;
b2Fixture::SetSensor, b2Fixture::SetThickWall, and b2Body::SetBullet must
traverse the body's contacts to reevaluate TOI eligibility.

## Thread Error Detection

I test for data races with valgrind DRD, which generates false positives
on conflicting atomic loads and stores because it can't detect atomics.

Box2D-MT can be configured to make DRD ignore specific atomic variables, which
eliminates the false positives. This can help when testing the thread-safety of
your program with DRD while using b2ThreadPoolTaskExecutor.

The DRD configuration also enables symbols, to identify the source of any problems
in Box2D-MT. See [Building.md](https://github.com/jhoffman0x/Box2D-MT/blob/master/Building.md) for details on using the DRD configurtion.

## Reproducibility

With Box2D, running the same build on the same machine with the same floating
point environment will produce the same results every time. This is also true
for Box2D-MT*, but the order of multithreaded callbacks is indeterminate.

If you rely on reproducibility for features like game replays or multiplayer, then
you must ensure that the effects of your immediate callbacks don't depend on the
order in which they're called. An easy solution is to do all order-dependent work
in deferred callbacks.

*It's tested for reproducibility on every test in the testbed, but that's still
a small subset of use cases. Create an issue if you identify any inconsistencies.

## License

Box2D-MT uses the same permissive license as Box2D.


**Note: the following is from Box2D, as is all included documentation.**


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
