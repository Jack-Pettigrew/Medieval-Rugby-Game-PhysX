#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>
#include <time.h>

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
		bool trigger, playerTrigger, scoreTrigger;

		MySimulationEventCallback() : trigger(false), playerTrigger(false), scoreTrigger(false) {}

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
						{
							printf("COLLISION PLAYER TRIGGER\n");
							
						}

						if (otherActorName == "Ball" && triggerActorName == "Score Trigger")
						{
							printf("Score!!!!!\n");
							scoreTrigger = true;
						}
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

						if (otherActorName == "Ball")
						{
							scoreTrigger = false;
							printf("Disabled Score!\n");
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
		Box* handle[4];
		Sphere* weaponHead[4];
		Goal* goal;
		Box* trebuchetTrigger, *scoreTrigger;
		Bleachers* bleachers;
		RotatingObstacle* rotatingObstacle[8];
		Box* obstacleStands[8];
		Wall* wall[6], *wall2;
		Castle* castle;

		// Simulation Callback
		MySimulationEventCallback* my_callback;
		
	public:

		enum PlayerController  { playerControls, trebuchetControls };

		// Public Game Objects
		Player *player;
		Capsule* ball;
		Enemy *heavyEnemy[4], *chaserEnemy[2];
		TrebuchetBase* trebuchetBase;
		TrebuchetArm* trebuchetArm;
		RevoluteJoint* weaponJoint[4],* trebuchetJoint, *obstacleJoint[8];

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
			plane->Color(PxVec3(50.0f / 255.f, 210.0f / 255.f, 50.0f / 255.f));
			plane->Name("Pitch");
			PxMaterial* planeMaterial = GetPhysics()->createMaterial(0.2f, 0.2f, 0.1f);
			plane->Material(planeMaterial);
			Add(plane);


			// Goal Setup
			goal = new Goal(PxTransform(PxVec3(0.0f, 5.0f, -400.0f)));
			///((PxRigidBody*)goal->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			Add(goal);

			/// Score Trigger
			scoreTrigger = new Box(PxTransform(((PxRigidBody*)goal->Get())->getGlobalPose().p + PxVec3(0.0f, 95.0f, -0.5f)), PxVec3(10.0f, 100.0f, 0.5f));
			scoreTrigger->Color(PxVec3(0.0f, 0.0f, 0.0f));
			scoreTrigger->Name("Score Trigger");
			scoreTrigger->SetKinematic(true);
			scoreTrigger->SetTrigger(true);
			Add(scoreTrigger);

			// Bleachers
			bleachers = new Bleachers(PxTransform(PxVec3(0.0f, 1.0f, -200.0f)));
			bleachers->Color(PxVec3(50.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f));
			bleachers->Name("Right Bleachers");
			bleachers->SetKinematic(true);
			Add(bleachers);

			// Ball Setup
			ball = new Capsule(PxTransform(PxVec3(-4.0f, 5.0f, -2.0f), PxQuat(PxIdentity)), PxVec2(0.5f, 0.5f), 2.0f);
			ball->Color(color_palette[2]);
			ball->Name("Ball");
			PxMaterial* ballMaterial = GetPhysics()->createMaterial(0.2f, 0.5f, 1.0f);
			ball->Material(ballMaterial);
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			ball->GetShape()->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(ball);


			// Player
			playerController = playerControls;

			player = new Player(PxTransform(PxVec3(-5.0f, 0.5f, -300.0f))); //125.0f
			player->Name("Player");
			player->Color(PxVec3(200.0f / 255.f, 50.0f / 255.f, 200.0f / 255.f));
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
			pos = { -75.0f, 0.5f, 15.0f };
			for (int i = 0; i < 4; i++)
			{
				heavyEnemy[i] = new Heavy(PxTransform(PxVec3(pos)), PxVec3(1.0f, 2.0f, 1.0f), 2.0f);
				heavyEnemy[i]->Init();
				heavyEnemy[i]->SetChaseTarget(player->Get());
				Add(heavyEnemy[i]);

				pos.x += 20.0f;
				if (i == 1)
					pos.x = 50.0f;
			}
			

			// Weapons
			
			/// Handle
			for (int i = 0; i < 5; i++)
			{
				handle[i] = new Box(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)), PxVec3(0.25f, 1.50f, 0.25f), 1.0f);
				handle[i]->SetKinematic(true);
				Add(handle[i]);

				/// Head
				weaponHead[i] = new Sphere(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)), 1.0f, 100.0f);
				weaponHead[i]->Material(CreateMaterial(0.5f, 0.5f, 2.0f));
				Add(weaponHead[i]);

				/// Joint
				weaponJoint[i] = new RevoluteJoint(handle[i], PxTransform(PxVec3(0.0f, 1.0f, 0.0f), PxQuat(PxReal(PxPi / 2), PxVec3(0.0f, 0.0f, 1.0f))), weaponHead[i], PxTransform(PxVec3(0.0f, 1.0f, 4.0f)));
				weaponJoint[i]->DriveVelocity(20.0f);
			}


			// Trebuchet

			/// Base
			PxVec3 trebuchetOffset = { -1.0f, -4.0f, -250.0f };
			trebuchetBase = new TrebuchetBase(PxTransform(((PxRigidBody*)goal->Get())->getGlobalPose().p + trebuchetOffset));
			trebuchetBase->SetKinematic(true);
			Add(trebuchetBase);

			/// Arm
			trebuchetArm = new TrebuchetArm(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p));
			Add(trebuchetArm);

			/// Joint
			trebuchetJoint = new RevoluteJoint(trebuchetBase, PxTransform(PxVec3(0.0f, 1.0f, 1.0f), PxQuat(PxReal(2 * PxPi), PxVec3(1.0f, 0.0f, 0.0f))), trebuchetArm, PxTransform(PxVec3(0.0f, -0.3f, -2.5f)));

			trebuchetJoint->SetLimitsByDegrees(0.0f, 180.0f);
			trebuchetJoint->Get()->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);

			/// Trigger Object
			trebuchetTrigger = new Box(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + PxVec3(0.0f, 0.0f, 10.0f)), PxVec3(100.0f, 0.5f, 0.5f));
			trebuchetTrigger->Name("Trebuchet Trigger");
			trebuchetTrigger->SetTrigger(true);
			trebuchetTrigger->SetKinematic(true);
			Add(trebuchetTrigger);


			// Obstacles
			PxVec3 obstacleOffset = { -30.0f, 0.5f, 50.0f };
			for (int i = 0; i < 8; i++)
			{
				obstacleStands[i] = new Box(PxTransform(obstacleOffset));
				obstacleStands[i]->Name("Obstacle Stand");
				obstacleStands[i]->SetKinematic(true);
				Add(obstacleStands[i]);

				rotatingObstacle[i] = new RotatingObstacle(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)));
				rotatingObstacle[i]->Name("Rotating Obstacle");
				Add(rotatingObstacle[i]);

				obstacleJoint[i] = new RevoluteJoint(obstacleStands[i], PxTransform(PxVec3(0.0f, 2.0f, 0.0f), PxQuat(PxReal(PxPi / 2), PxVec3(0.0f, 0.0f, 1.0f))), rotatingObstacle[i], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

				obstacleJoint[i]->DriveVelocity(1.0f);

				obstacleOffset.x += 30.0f;

				if (i == 2)
				{
					obstacleOffset.x = -15.0f;
					obstacleOffset.z -= 90.0f;
				}
				else if (i == 4)
				{
					obstacleOffset.x = -30.0f;
					obstacleOffset.z -= 90.0f;
				}
			}

			// Wall
			wall[0] = new Wall(PxTransform(PxVec3(70.0f, 1.0f, 50.0f)));
			wall[0]->ChangeWallSize(PxVec3(25.0f, 1.0f, 0.5f));
			wall[1] = new Wall(PxTransform(PxVec3(-70.0f, 1.0f, 50.0f)));
			wall[1]->ChangeWallSize(PxVec3(25.0f, 1.0f, 0.5f));
			wall[2] = new Wall(PxTransform(PxVec3(70.0f, 1.0f, -40.0f)));
			wall[2]->ChangeWallSize(PxVec3(40.0f, 1.0f, 0.5f));
			wall[3] = new Wall(PxTransform(PxVec3(-70.0f, 1.0f, -40.0f)));
			wall[3]->ChangeWallSize(PxVec3(40.0f, 1.0f, 0.5f));
			wall[4] = new Wall(PxTransform(PxVec3(-70.0f, 1.0f, -130.0f)));
			wall[4]->ChangeWallSize(PxVec3(25.0f, 1.0f, 0.5f));
			wall[5] = new Wall(PxTransform(PxVec3(-70.0f, 1.0f, -130.0f)));
			wall[5]->ChangeWallSize(PxVec3(25.0f, 1.0f, 0.5f));

			for (int i = 0; i < 6; i++)
			{
				wall[i]->SetKinematic(true);
				Add(wall[i]);
			}

			wall2 = new Wall(PxTransform(PxVec3(70.0f, 1.0f, -130.0f)));
			wall2->ChangeWallSize(PxVec3(25.0f, 1.0f, 0.5f));
			wall2->SetKinematic(true);
			Add(wall2);


			// Castle
			castle = new Castle(PxTransform(PxVec3(0.0f, 0.5f, -400.0f)));
			castle->SetKinematic(true);
			Add(castle);

			/// Castle Cloth


			// Destructable Flags + Cloths

		}

		// Custom update function
		virtual void CustomUpdate() 
		{
			// Player Update
			player->Update();
			trebuchetBase->Update();

			//Enemy Updates
			for (int i = 0; i < 4; i++)
			{
				heavyEnemy[i]->Update();

			if(i < 2)
				chaserEnemy[i]->Update();
			}


			// Morning Star Handles
			PxVec3 offset = { 0.0f, 1.0f, 1.5f };
			for (int i = 0; i < 5; i++)
			{
				((PxRigidBody*)handle[i]->Get())->setGlobalPose(PxTransform(((PxRigidBody*)heavyEnemy[i]->Get())->getGlobalPose().p + offset, PxQuat(PxIdentity)));
			}

			if (my_callback->playerTrigger)
				PlayerToTrebuchet();

			if (my_callback->scoreTrigger)
				GoalTrigger();
		}

		// Switch to Trebuchet Controls
		void PlayerToTrebuchet()
		{
			playerController = trebuchetControls;

			player->SetKinematic(true);
			PxVec3 playerOffset = { 5.0f, 1.0f, 0.0f };
			((PxRigidBody*)player->Get())->setGlobalPose(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + playerOffset));
			my_callback->playerTrigger = false;

			player->SetBallTarget(nullptr);
			PxVec3 ballOffset = { 0.1f, 0.0f, 7.5f };
			((PxRigidBody*)ball->Get())->setGlobalPose(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + ballOffset));
			ball->GetShape()->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);

			for (int i = 0; i < 5; i++)
			{
				heavyEnemy[i]->SetChaseTarget(nullptr);

				if (i < 2)
					chaserEnemy[i]->SetChaseTarget(nullptr);
			}
		}

		// Goal Score Reaction
		void GoalTrigger()
		{
			my_callback->scoreTrigger = false;

			// Create Random Balls
			for (int i = 0; i < 200; i++)
			{
				PxVec3 ballPositions(0.0f, 30.0f, -350.0f);

				Box* box = new Box(PxTransform(ballPositions), PxVec3(1.0f, 1.0f, 1.0f), 100.0f);
				box->Material(CreateMaterial(0.5f, 0.5f, 1.5f));
				box->Name("Score Ball");
				box->Color(PxVec3(0.7f, 0.7f, 0.2f));
				Add(box);

			}
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
