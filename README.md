##About

Box2D-MT adds multi-threading to Erin Catto's 2D physics engine, Box2D.

It has no external dependencies and is no less deterministic than Box2D.

It requires a somewhat modern compiler, since it relies on C++11's thread support library.

Before using Box2D-MT, you should be familiar with Box2D, and multi-threading.

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

Box2D-MT is made by Justin Hoffman.

##Performance
Running the demo world on a 2 core processor gets about a 2.4x speedup compared to the original Box2D.

Compared to running Box2D-MT on a single core, you should generally get a 1.5-1.8x speedup on 2 cores, 
and a 2-3x speedup on 4 cores. However, the speedup will be worse if your world has few bodies or if most 
of the bodies are in a single cluster.

##Quick Start
To enable multi-threading, create a thread-pool and pass it to your world's constructor.

```
b2ThreadPool tp;
b2World world(gravity, &tp);
```

Now when you step the world, the work will be split among all of your cpu's cores.
The thread pool must remain in scope until the world is destroyed.

##Callback Thread-Safety
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
kill performance). You can use an array of per thread data of size b2_maxThreads, and use b2GetThreadId() to index
the array.

A later version of Box2D-MT should provide an option to automatically defer callbacks.
