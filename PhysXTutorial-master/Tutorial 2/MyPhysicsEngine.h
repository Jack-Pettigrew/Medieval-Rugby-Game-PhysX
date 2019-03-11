#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(61.f/255.f,1.f/255.f,60.f/255.f),PxVec3(96.f/255.f, 72.f/255.f, 127.f/255.f),
		PxVec3(120/255.f, 136.f/255.f, 191.0f/255.f), PxVec3(104.0f/255.f, 194.0f/255.f, 229.0f/255.f),PxVec3(74.0f/255.f, 248.0f/255.f, 255.0f/255.f)};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Box* box[20];
		Sphere* sphere;
		CompoundShape* cShape;

	public:
		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			Add(plane);

			float xPos = 0.0f;
			float zPos = 0.0f;

			for (int x = 0; x < 20; x++)
			{

				box[x] = new Box(PxTransform(PxVec3(xPos, 0.5f, zPos)));
				Add(box[x]);

				if (x == 10)
				{
					xPos = 25.0f;
					zPos = 0.0f;
				}

				if (xPos <= 10)
				{
					xPos += 1.0f;
				}
				else
				{
					xPos -= 1.0f;
				}

				zPos -= 2.0f;
			}

			cShape = new CompoundShape(PxTransform(PxVec3(12.0f, 0.5f, 10.0f)));
			Add(cShape);

			PxTransform sphereOffest = ((PxRigidBody*)cShape->Get())->getGlobalPose() * PxTransform(PxVec3(0.0f, 1.0f, -2.0f));

			sphere = new Sphere(sphereOffest, 0.5f, 1.0);
			((PxRigidBody*)sphere->Get())->setLinearVelocity(PxVec3(0.0f, 0.0f, -150.0f));
			((PxActor*)sphere->Get())->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
			
			Add(sphere);
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			//px_scene->raycast(((PxRigidBody*)sphere->Get())->getGlobalPose(), );
			

		}
	};
}
