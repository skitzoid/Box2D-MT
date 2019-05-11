/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "../Framework/Test.h"

#include "AddPair.h"
#include "ApplyForce.h"
#include "BasicSliderCrank.h"
#include "BodyTypes.h"
#include "Breakable.h"
#include "Bridge.h"
#include "BulletTest.h"
#include "Cantilever.h"
#include "Car.h"
#include "ContinuousTest.h"
#include "Chain.h"
#include "CharacterCollision.h"
#include "CollisionFiltering.h"
#include "CollisionProcessing.h"
#include "CompoundShapes.h"
#include "Confined.h"
#include "ConvexHull.h"
#include "ConveyorBelt.h"
#include "DistanceTest.h"
#include "Dominos.h"
#include "DumpShell.h"
#include "DynamicTreeTest.h"
#include "EdgeShapes.h"
#include "EdgeTest.h"
#include "Gears.h"
#include "HeavyOnLight.h"
#include "HeavyOnLightTwo.h"
#include "Mobile.h"
#include "MobileBalanced.h"
#include "MotorJoint.h"
#include "ManyBodies.h"
#include "MultithreadDemo.h"
#include "OneSidedPlatform.h"
#include "Pinball.h"
#include "PolyCollision.h"
#include "PolyShapes.h"
#include "Prismatic.h"
#include "Pulleys.h"
#include "Pyramid.h"
#include "RayCast.h"
#include "Revolute.h"
#include "RopeJoint.h"
#include "SensorTest.h"
#include "ShapeCast.h"
#include "ShapeEditing.h"
#include "SliderCrank.h"
#include "SphereStack.h"
#include "TheoJansen.h"
#include "Tiles.h"
#include "TimeOfImpact.h"
#include "TunnelingTest.h"
#include "Tumbler.h"
#include "VaryingFriction.h"
#include "VaryingRestitution.h"
#include "VerticalStack.h"
#include "Web.h"

TestEntry g_testEntries[] =
{
	{"Many Bodies 1", ManyBodies1::Create, 480},
	{"Many Bodies 2", ManyBodies2::Create, 480},
	{"Many Bodies 3", ManyBodies3::Create, 480},
	{"Many Bodies 4", ManyBodies4::Create, 480},
	{"Many Bodies 5", ManyBodies5::Create, 480},
	{"Multithread Demo", MultithreadDemo::Create, 1800},
	{"Tunneling Test", TunnelingTest::Create, 1800},
	{"Shape Cast", ShapeCast::Create, 60},
	{"Time of Impact", TimeOfImpact::Create, 60},
	{"Character Collision", CharacterCollision::Create, 240},
	{"Tiles", Tiles::Create, 1020},
	{"Heavy on Light", HeavyOnLight::Create, 480},
	{"Heavy on Light Two", HeavyOnLightTwo::Create, 240},
	{"Vertical Stack", VerticalStack::Create, 480},
	{"Basic Slider Crank", BasicSliderCrank::Create, 480},
	{"Slider Crank", SliderCrank::Create, 480},
	{"Sphere Stack", SphereStack::Create, 480},
	{"Convex Hull", ConvexHull::Create, 60},
	{"Tumbler", Tumbler::Create, 1800},
	{"Ray-Cast", RayCast::Create, 240},
	{"Dump Shell", DumpShell::Create, 240},
	{"Apply Force", ApplyForce::Create, 240},
	{"Continuous Test", ContinuousTest::Create, 240},
	{"Motor Joint", MotorJoint::Create, 240},
	{"One-Sided Platform", OneSidedPlatform::Create, 240},
	{"Mobile", Mobile::Create, 1800},
	{"MobileBalanced", MobileBalanced::Create, 240},
	{"Conveyor Belt", ConveyorBelt::Create, 480},
	{"Gears", Gears::Create, 240},
	{"Varying Restitution", VaryingRestitution::Create, 480},
	{"Cantilever", Cantilever::Create, 480},
	{"Edge Test", EdgeTest::Create, 240},
	{"Body Types", BodyTypes::Create, 240},
	{"Shape Editing", ShapeEditing::Create, 240},
	{"Car", Car::Create, 240},
	{"Prismatic", Prismatic::Create, 240},
	{"Revolute", Revolute::Create, 480},
	{"Pulleys", Pulleys::Create, 480},
	{"Polygon Shapes", PolyShapes::Create, 240},
	{"Web", Web::Create, 480},
	{"RopeJoint", RopeJoint::Create, 480},
	{"Pinball", Pinball::Create, 480},
	{"Bullet Test", BulletTest::Create, 240},
	{"Confined", Confined::Create, 240},
	{"Pyramid", Pyramid::Create, 480},
	{"Theo Jansen's Walker", TheoJansen::Create, 1020},
	{"Edge Shapes", EdgeShapes::Create, 480},
	{"PolyCollision", PolyCollision::Create, 240},
	{"Bridge", Bridge::Create, 1020},
	{"Breakable", Breakable::Create, 480},
	{"Chain", Chain::Create, 1020},
	{"Collision Filtering", CollisionFiltering::Create, 480},
	{"Collision Processing", CollisionProcessing::Create, 480},
	{"Compound Shapes", CompoundShapes::Create, 480},
	{"Distance Test", DistanceTest::Create, 60},
	{"Dominos", Dominos::Create, 1020},
	{"Dynamic Tree", DynamicTreeTest::Create, 60},
	{"Sensor Test", SensorTest::Create, 480},
	{"Varying Friction", VaryingFriction::Create, 1020},
	{"Add Pair Stress Test", AddPair::Create, 240},
	{NULL, NULL}
};
