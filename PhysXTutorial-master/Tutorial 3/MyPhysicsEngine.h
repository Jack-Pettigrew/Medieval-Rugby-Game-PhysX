#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	// A list of custom colour palette: Circus Palette
	static const PxVec3 color_palette[] = { PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	// Pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};

	// pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	// vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
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

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:

		//an example variable that will be checked in the main simulation loop
		bool trigger, playerTrigger;

		MySimulationEventCallback() : trigger(false), playerTrigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;

						// Player Trigger
						string otherActorName = pairs[i].otherActor->getName();
						string triggerActorName = pairs[i].triggerActor->getName();

						if (otherActorName == "Player" && triggerActorName == "Trebuchet Trigger")
						{
							playerTrigger = true;
							printf("Trebuchet Trigger Called\n");
						}

						if (otherActorName == "Chaser" && triggerActorName == "Player")
							printf("COLLISION PLAYER TRIGGER\n");

					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;

						// Player Trigger
						string otherActorName = pairs[i].otherActor->getName();

						if (otherActorName == "Player")
						{
							playerTrigger = false;
						}
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};


	// ============================  Rugby Scene Class  ============================


	class MyScene : public Scene
	{
		// Private Game Objects
		Plane* plane;
		Box* handle;
		Capsule* ball;
		Goal* goal;
		Sphere* weaponHead;
		Box* trebuchetTrigger;


		// Simulation Callback
		MySimulationEventCallback* my_callback;
		
	public:

		enum PlayerController  { playerControls, trebuchetControls };

		// Public Game Objects
		Player *player;
		Enemy *heavyEnemy[5], *chaserEnemy[2];
		TrebuchetBase* trebuchetBase;
		TrebuchetArm* trebuchetArm;
		RevoluteJoint* weaponJoint,* trebuchetJoint;

		PlayerController playerController; 

		bool pressed = false;

		///specify your custom filter shader here
		///PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};

		// Debug Visulisation Setup
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
		}

		// Scene Init
		virtual void CustomInit()
		{
			SetVisualisation();

			GetMaterial()->setDynamicFriction(.2f);

			// Initialise and activate Custom Event Callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);


			// Plane Setup
			plane = new Plane();
			plane->Color(PxVec3(100.0f / 255.f, 210.0f / 255.f, 100.0f / 255.f));
			plane->Name("Pitch");
			PxMaterial* planeMaterial = GetPhysics()->createMaterial(0.2f, 0.2f, 0.1f);
			plane->Material(planeMaterial);
			Add(plane);


			// Goal Setup
			goal = new Goal(PxTransform(PxVec3(0.0f, 5.0f, -400.0f)));
			///((PxRigidBody*)goal->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			Add(goal);


			// Ball Setup
			ball = new Capsule(PxTransform(PxVec3(-4.0f, 5.0f, -2.0f), PxQuat(PxIdentity)), PxVec2(0.5f, 0.5f), 2.0f);
			ball->Color(color_palette[2]);
			ball->Name("Ball");
			PxMaterial* ballMaterial = GetPhysics()->createMaterial(0.2f, 0.5f, 1.0f);
			ball->Material(ballMaterial);
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(ball);


			// Player
			playerController = playerControls;

			player = new Player(PxTransform(PxVec3(-5.0f, 0.5f, -17.0f)));
			player->Name("Player");
			///PxMaterial* playerMaterial = GetPhysics()->createMaterial(0.1f, 0.1f, 0.0f);
			///player->Material(playerMaterial);
			player->SetBallTarget(((PxRigidBody*)ball->Get()));
			Add(player);


			// Enemies

			/// Chaser Enemies
			PxVec3 pos = { -20.0f, 1.0f, -60.0f };
			for (int i = 0; i < 2; i++)
			{
				chaserEnemy[i] = new Chaser(PxTransform(PxVec3(pos)), PxVec3(1.0f, 2.0f, 1.0f), 2.0f);
				chaserEnemy[i]->Init();
				chaserEnemy[i]->SetChaseTarget(player->Get());
				Add(chaserEnemy[i]);

				pos.x += 40.0f;
			}

			/// Heavy Enemies
			pos = { -50.0f, 0.5f, -30.0f };
			for (int i = 0; i < 5; i++)
			{
				heavyEnemy[i] = new Heavy(PxTransform(PxVec3(pos)), PxVec3(1.0f, 2.0f, 1.0f), 2.0f);
				heavyEnemy[i]->Init();
				heavyEnemy[i]->SetChaseTarget(player->Get());
				Add(heavyEnemy[i]);

				pos.x += 20.0f;
			}
			

			// Weapons
			
			/// Handle
			handle = new Box(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)), PxVec3(0.25f, 1.50f, 0.25f), 1.0f);
			handle->SetKinematic(true);
			Add(handle);

			/// Head
			weaponHead = new Sphere(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)), 1.0f, 200.0f);
			Add(weaponHead);

			/// Joint
			weaponJoint = new RevoluteJoint(handle, PxTransform(PxVec3(0.0f, 1.0f, 0.0f), PxQuat(PxReal(PxPi / 2), PxVec3(0.0f, 0.0f, 1.0f))), weaponHead, PxTransform(PxVec3(0.0f, 1.0f, 4.0f)));
			weaponJoint->DriveVelocity(20.0f);


			// Trebuchet

			/// Base
			PxVec3 trebuchetOffset = { -1.0f, -4.0f, 100.0f };
			trebuchetBase = new TrebuchetBase(PxTransform(((PxRigidBody*)goal->Get())->getGlobalPose().p + trebuchetOffset));
			trebuchetBase->SetKinematic(true);
			Add(trebuchetBase);

			/// Arm
			trebuchetArm = new TrebuchetArm(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p, PxQuat(PxPi / 2, PxVec3(1.0f, 0.0f, 0.0f))));
			Add(trebuchetArm);

			/// Joint
			trebuchetJoint = new RevoluteJoint(trebuchetBase, PxTransform(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxReal(2 * PxPi), PxVec3(1.0f, 0.0f, 0.0f))), trebuchetArm, PxTransform(PxVec3(0.0f, -0.3f, -2.5f)));
			trebuchetJoint->SetLimitsByDegrees(0.0f, 180.0f);
			((PxRevoluteJoint*)trebuchetJoint->Get())->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
			trebuchetJoint->Get()->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);

			/// Trigger Object
			trebuchetTrigger = new Box(PxTransform(PxVec3(0.0f, 2.0f, -10.0f)));
			trebuchetTrigger->Name("Trebuchet Trigger");
			trebuchetTrigger->SetTrigger(true);
			trebuchetTrigger->SetKinematic(true);
			Add(trebuchetTrigger);

		}

		// Custom update function
		virtual void CustomUpdate() 
		{
			// Player Update
			player->Update();
			trebuchetBase->Update();

			//Enemy Updates
			//for (int i = 0; i < 5; i++)
			//{
			//	heavyEnemy[i]->Update();

			//if(i < 2)
			//	chaserEnemy[i]->Update();
			//}

			PxVec3 offset = { 0.0f, 1.0f, 1.5f };
			((PxRigidBody*)handle->Get())->setGlobalPose(PxTransform(((PxRigidBody*)heavyEnemy[0]->Get())->getGlobalPose().p + offset, PxQuat(PxIdentity)));

			if (my_callback->playerTrigger)
				PlayerToTrebuchet();

		}

		// Switch to Trebuchet Controls
		void PlayerToTrebuchet()
		{
			playerController = trebuchetControls;

			player->SetKinematic(true);
			PxVec3 playerOffset = { 3.0f, 1.0f, 0.0f };
			((PxRigidBody*)player->Get())->setGlobalPose(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + playerOffset));
			my_callback->playerTrigger = false;
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}

	};
}
