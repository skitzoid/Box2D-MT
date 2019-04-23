
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
game engine's thread pool instead by implementing b2TaskExecutor

### Multithreaded Callbacks

Box2D-MT adds 4 pure virtual functions to b2ContactListener, which correspond to
the Box2D functions.

```
/// This lets you process and filter BeginContact callbacks as they arise from multiple threads.
/// Within this callback, bodies and joints must not be modified. It's safe to read and modify
/// the provided contact. Other contacts must not be accessed. Unmentioned Box2d objects probably
/// shouldn't be accessed.
/// Note: threadId is unique per thread and less than the number of threads.
/// @return true if BeginContact must be called for the contact.
/// @warning this function is called from multiple threads.
virtual bool BeginContactImmediate(b2Contact* contact, uint32 threadId) = 0;

/// This lets you process and filter EndContact callbacks as they arise from multiple threads.
/// Within this callback, bodies and joints must not be modified. It's safe to read and modify
/// the provided contact. Other contacts must not be accessed. Unmentioned Box2d objects probably
/// shouldn't be accessed.
/// Note: threadId is unique per thread and less than the number of threads.
/// @return true if EndContact must be called for the contact.
/// @warning this function is called from multiple threads.
virtual bool EndContactImmediate(b2Contact* contact, uint32 threadId) = 0;

/// This lets you process and filter PreSolve callbacks as they arise from multiple threads.
/// Within this callback, it's safe to read and modify the provided contact. A non-static body that is
/// part of the provided contact is also safe to modify, except for the body's flags, which must be
/// treated as read-only. Joints attached to a non-static body are safe to modify. A static body that
/// is part of the provided contact must be treated as read-only, except for its flags, which must
/// not be accessed. Other bodies, joints, and contacts must not be accessed. Unmentioned Box2d objects
/// probably shouldn't be accessed.
/// Note: threadId is unique per thread and less than the number of threads.
/// @return true if PreSolve must be called for the contact.
/// @warning this function is called from multiple threads.
virtual bool PreSolveImmediate(b2Contact* contact, const b2Manifold* oldManifold, uint32 threadId) = 0;

/// This lets you process and filter PostSolve callbacks as they arise from multiple threads.
/// Within this callback, it's safe to read and modify the provided contact. Other contacts must not
/// be accessed. It's safe to read or modify a non-static body that is part of the provided contact.
/// A static body that is part of the provided contact must be treated as read-only. Unmentioned
/// Box2d objects probably shouldn't be accessed.
/// Note: threadId is unique per thread and less than the number of threads.
/// @return true if PostSolve must be called for the contact.
/// @warning this function is called from multiple threads.
virtual bool PostSolveImmediate(b2Contact* contact, const b2ContactImpulse* impulse, uint32 threadId) = 0;
```

These are called by multiple threads simultaneously, so your implementation must
be thread-safe. If your function returns true then the arguments will be saved
and used to invoke the corresponding non-immediate function after the
multithreaded work is finished.

Immediate functions can filter events to avoid the overhead of a deferred call.
This is more important for PreSolve and PostSolve because more contacts receive
those calls every step. You should start with an implementation that simply
returns true for callbacks that you use and false for callbacks you don't use.
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
callbacks for the sake of simplicity. Otherwise, if profiling shows significant
time spent in deferred callbacks (or in Box2D-MT's processing of deferred
callback arguments) then you can move work into immediate callbacks, but only
if you can do so without causing data races. Read the function comments in
b2WorldCallbacks.h to see what data is safe to access.

I'm open to suggestions on changes to the contact listener interface, so create
an issue if you want to discuss it.

## Time of Impact (TOI) Changes

Since TOI is still processed on a single thread, it's important that it doesn't
do more work than necessary. These changes address that.

### Finer-grained Continuous Collision Detection (CCD) Control

In Box2D, enabling TOI means you get CCD on all static geometry. However, you
may need TOI to prevent tunneling on thin objects but also have thick objects
that aren't subject to tunneling. Avoiding CCD on contacts with the thick objects
will improve performance, so Box2D-MT has a setting that you can apply to bodies
that hold thick geometry:

```
/// Should this body only use continuous collision detection when colliding with bullet bodies?
void SetPreferNoCCD(bool flag);

/// Does this body only use continuous collision detection when colliding with bullet bodies?
bool GetPreferNoCCD() const;
```

Without this you would typically have one static body that has fixtures for your
world's geometry. If you use this feature, then you'll have an extra static body
with the PreferNoCCD flag applied, and you'll add your thick geometry to this body.

The names of those functions and their exact functionality are subject to change.
Suggestions are welcome.

### Partitioning Contacts Based on TOI Eligibility

In Box2D, TOI can be expensive when there's many contacts, even when few or
none of them are actually eligible for TOI (because both bodies are non-bullet
dynamic, or one of the fixtures is a sensor). This can be seen in the Tumbler
test, where on my system SolveTOI accounts for roughly 10% of the step time
despite all contacts being between non-bullet dynamic bodies and thus ineligible
for TOI. Of course in that case you could just disable TOI, but in practice
it's common to have many ineligible contacts when you still need TOI.

In Box2D's SolveTOI, in order to determine that a contact is ineligible due to
both bodies being dynamic, you need to first read the contact pointer, then read
two fixture pointers from the contact, then read two body pointers from the
fixtures, and finally read two types from the bodies. Since this is done for
every contact in the world, the cost adds up when there's many contacts.

Box2D-MT avoids this overhead by evaluating TOI eligibility when the contact is
created, and partitioning the contact list so that TOI eligible contacts come
before ineligible contacts. This shifts the cost into functions that are called
infrequently; b2Fixture::SetSensor, b2Body::SetBullet, and b2Body::SetPreferNoCCD
must traverse the body's contacts to re-evaluate TOI eligibility, however, in
typical use cases the added cost to these functions is negligible compared to the
savings in SolveTOI.

## Thread Error Detection

I use valgrind DRD to test for data races. These kind of tools can't detect
atomic variables so they generate false positives on conflicting atomic loads
and stores. Box2D-MT can be configured to make DRD ignore specific atomic
variables, which eliminates the false positives. This can help when testing the
thread-safety of your program with DRD while using b2ThreadPoolTaskExecutor. The
DRD configuration also enables symbols, to identify the source of any problems
in Box2D-MT.

See [Building.md](https://github.com/jhoffman0x/Box2D-MT/blob/master/Building.md) for details on using the DRD configurtion.

## Reproducibility

With Box2D, running the same build on the same machine with the same floating
point environment will produce the same results every time. This is also true
for Box2D-MT*, but the order of multithreaded callbacks is indeterminate. If
you rely on reproducibility for features like game replays or multiplayer, then
you must ensure that your immediate callbacks produce consistent results
regardless of the order in which they're called. An easy solution is to do all
order-dependent work in deferred callbacks.

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
