#pragma once

#include "BasicActors.h"
#include "SZ_ChronoTimer.h"
#include <iostream>
#include <iomanip>
#include <ctime>
#include <cstdlib>

namespace PhysicsEngine
{
	using namespace std;

	// A list of custom colour palette: Circus Palette
	static const PxVec3 color_palette[] = { PxVec3(46.f / 255.f,9.f / 255.f,39.f / 255.f),PxVec3(217.f / 255.f,0.f / 255.f,0.f / 255.f),
		PxVec3(255.f / 255.f,45.f / 255.f,0.f / 255.f),PxVec3(255.f / 255.f,140.f / 255.f,54.f / 255.f),PxVec3(4.f / 255.f,117.f / 255.f,111.f / 255.f) };

	static const PxVec3 scoreColors[] = { PxVec3(0.7f, 0.7f, 0.2f), PxVec3(0.7f, 0.2f, 0.7f), PxVec3(0.2f, 0.7f, 0.7f), PxVec3(0.7f, 0.0f, 1.0f) };

	// Pyramid vertices
	static PxVec3 pyramid_verts[] = { PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1) };

	// pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	// vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = { 1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1 };

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose = PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts), end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs), end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0 = (1 << 0),
			ACTOR1 = (1 << 1),
			ACTOR2 = (1 << 2),
			PLAYER = (1 << 3),
			WEAPON = (1 << 4)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxTransform& pos, const PxVec3& dimensions = PxVec3(1.f, 1.f, 1.f), PxReal stiffness = 1.f, PxReal damping = 1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(pos.p.x, thickness, pos.p.z)), PxVec3(dimensions.x, thickness, dimensions.z));
			top = new Box(PxTransform(PxVec3(pos.p.x, dimensions.y + thickness, pos.p.z)), PxVec3(dimensions.x, thickness, dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x, thickness, dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -dimensions.y, dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x, thickness, -dimensions.z)), top, PxTransform(PxVec3(dimensions.x, -dimensions.y, -dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x, thickness, dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -dimensions.y, dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x, thickness, -dimensions.z)), top, PxTransform(PxVec3(-dimensions.x, -dimensions.y, -dimensions.z)));

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
		bool trigger, playerTrigger, scoreTrigger, restart, playerHit;

		MySimulationEventCallback() : trigger(false), playerTrigger(false), scoreTrigger(false), restart(false), playerHit(false) {}

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
						//cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;

						// Player Trigger
						string otherActorName = pairs[i].otherActor->getName();
						string triggerActorName = pairs[i].triggerActor->getName();

						if (otherActorName == "Player" && triggerActorName == "Trebuchet Trigger")
						{
							playerTrigger = true;
							printf("COLLISION: %s triggered %s \n", otherActorName.c_str(), triggerActorName.c_str());
						}

						if (otherActorName == "Chaser" && triggerActorName == "Player")
						{
							printf("COLLISION: %s triggered %s \n RESTARTING GAME...\n", otherActorName.c_str(), triggerActorName.c_str());
							restart = true;
						}

						if (otherActorName == "Head" && triggerActorName == "Player")
						{
							printf("COLLISION: %s triggered %s \n RESTARTING GAME...\n", otherActorName.c_str(), triggerActorName.c_str());
							restart = true;
						}

						if (otherActorName == "Ball" && triggerActorName == "Score Trigger")
						{
							printf("COLLISION: %s triggered %s \n PLAYER SCORED - RELEASING THE CUBES!\n", otherActorName.c_str(), triggerActorName.c_str());
							scoreTrigger = true;
						}
						if (otherActorName == "Arrow" && triggerActorName == "Player")
						{
							printf("COLLISION: %s hit and pushed %s \n", otherActorName.c_str(), triggerActorName.c_str());
							((PxRigidBody*)pairs[i].triggerActor)->addForce(PxVec3(((PxRigidBody*)pairs[i].otherActor)->getLinearVelocity() * 1000.0f));
						}
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						//cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
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
						}
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs)
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			// Set Player 
			playerHit = true;

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
	static PxFilterFlags CustomFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		// let triggers through
		if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
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
		if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
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
		Box* trebuchetTrigger, *scoreTrigger, *flagPole;
		Bleachers* bleachers, * bleachersBehind;
		RotatingObstacle* rotatingObstacle[8];
		Box* obstacleStands[8];
		Wall* wall[6], *wall2;
		Castle* castle;
		std::vector<Ball*> spawnBalls;
		Trampoline* tramopline;

		// Timer Instance
		SZ_ChronoTimer chronoRestart;

		// Simulation Callback
		MySimulationEventCallback* my_callback;

		const float BALL_TIMER_CONST = 12.0f, RESTART_TIMER_CONST = 3000.0f;
		float ballTimer = BALL_TIMER_CONST, restartTimer = RESTART_TIMER_CONST;

	public:

		int clothCount;
		enum PlayerController { playerControls, trebuchetControls };

		// Public Game Objects
		Player *player;
		Ball* ball;
		Enemy *heavyEnemy[4], *chaserEnemy[2];
		TrebuchetBase* trebuchetBase;
		TrebuchetArm* trebuchetArm;
		RevoluteJoint* weaponJoint[4], *trebuchetJoint, *obstacleJoint[8];
		Cloth* cloth[4], *flag;
		Towers* tower[2];

		// Player Controls Modifier
		PlayerController playerController;

		bool testCase = false;
		bool kickTime = false;
		bool pressed = false;

		///specify your custom filter shader here
		///PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

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
			PxMaterial* planeMaterial = GetPhysics()->createMaterial(0.2f, 0.25f, 0.1f);
			plane->Material(planeMaterial);
			Add(plane);


			// Goal Setup
			goal = new Goal(PxTransform(PxVec3(0.0f, 5.0f, -410.0f)));
			///((PxRigidBody*)goal->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
			Add(goal);

			/// Score Trigger
			scoreTrigger = new Box(PxTransform(((PxRigidBody*)goal->Get())->getGlobalPose().p + PxVec3(0.0f, 13.0f, -0.5f)), PxVec3(10.0f, 9.0f, 0.01f));
			scoreTrigger->Color(PxVec3(0.5f, 0.5f, 0.5f));
			scoreTrigger->Name("Score Trigger");
			scoreTrigger->SetKinematic(true);
			scoreTrigger->SetTrigger(true);
			Add(scoreTrigger);

			// Bleachers
			bleachers = new Bleachers(false, PxTransform(PxVec3(0.0f, 1.0f, -200.0f)));
			bleachers->Color(PxVec3(50.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f));
			bleachers->Name("Right Bleachers");
			bleachers->SetKinematic(true);
			Add(bleachers);

			bleachersBehind = new Bleachers(true, PxTransform(PxVec3(0.0f, 1.0f, 75.0f), PxQuat(3 * (PxPi / 2), PxVec3(0.0f, 1.0f, 0.0f))));
			bleachers->Color(PxVec3(50.0f / 255.0f, 50.0f / 255.0f, 50.0f / 255.0f));
			bleachersBehind->SetKinematic(true);
			Add(bleachersBehind);

			// Ball Setup
			ball = new Ball(PxTransform(PxVec3(-4.0f, 5.0f, -2.0f)), 1.0f);
			ball->Color(color_palette[2]);
			ball->Name("Ball");
			PxMaterial* ballMaterial = GetPhysics()->createMaterial(0.2f, 0.5f, 1.3f);
			ball->Material(ballMaterial);
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

			/// Simulation Flag acting as Collision Filter 
			for each (PxShape* shape in ball->GetShapes())					/// Ball does not need to interact with world at this time!
				shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);

			/// Accurate Collision Detection for ball
			((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			Add(ball);


			// Player
			playerController = playerControls;

			player = new Player(PxTransform(PxVec3(-5.0f, 0.5f, -350.0f))); //125.0f
			player->Name("Player");
			player->Color(PxVec3(200.0f / 255.f, 50.0f / 255.f, 200.0f / 255.f));
			player->SetBallTarget(((PxRigidBody*)ball->Get()));
			player->SetupFiltering(FilterGroup::PLAYER, FilterGroup::WEAPON);
			Add(player);


			// Enemies
#pragma region Enemies Init


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

#pragma endregion

			// Weapons
#pragma region Weapons Init


			/// Handle
			for (int i = 0; i <= 4; i++)
			{

				handle[i] = new Box(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)), PxVec3(0.25f, 1.50f, 0.25f), 1.0f);
				handle[i]->SetKinematic(true);
				Add(handle[i]);

				/// Head
				weaponHead[i] = new Sphere(PxTransform(PxVec3(0.0f, 6.0f, 0.0f)), 1.0f, 100.0f);
				weaponHead[i]->Name("Head");
				weaponHead[i]->Material(CreateMaterial(0.5f, 0.5f, 2.0f));
				weaponHead[i]->CreateShape(PxSphereGeometry(1.0f), 0.0001f);
				weaponHead[i]->SetupFiltering(FilterGroup::WEAPON, FilterGroup::PLAYER);
				Add(weaponHead[i]);


				/// Joint
				weaponJoint[i] = new RevoluteJoint(handle[i], PxTransform(PxVec3(0.0f, 1.0f, 0.0f), PxQuat(PxReal(PxPi / 2), PxVec3(0.0f, 0.0f, 1.0f))), weaponHead[i], PxTransform(PxVec3(0.0f, 1.0f, 4.0f)));
				weaponJoint[i]->DriveVelocity(20.0f);

				/// Weapon Aggregate
				PxAggregate* weaponAggregate = GetPhysics()->createAggregate(2, false);
				weaponAggregate->addActor(*handle[i]->Get());
				weaponAggregate->addActor(*weaponHead[i]->Get());

			}

#pragma endregion


			// Trebuchet
#pragma region Trebuchet Init

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

			/// Trebuchet Trigger Object
			trebuchetTrigger = new Box(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + PxVec3(0.0f, -2.40f, 10.0f)), PxVec3(100.0f, 0.5f, 0.5f));
			trebuchetTrigger->Name("Trebuchet Trigger");
			trebuchetTrigger->SetTrigger(true);
			trebuchetTrigger->SetKinematic(true);
			Add(trebuchetTrigger);

			/// Trebuchet Aggregate (collection of shapes creating a single entity)
			PxU32 numActors = 2;
			bool selfCollisions = true;

			PxAggregate* trebuchetAggregate = GetPhysics()->createAggregate(numActors, selfCollisions);
			trebuchetAggregate->addActor(*trebuchetBase->Get());
			trebuchetAggregate->addActor(*trebuchetArm->Get());

#pragma endregion


			// Obstacles
#pragma region Obstacles Init

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

				obstacleJoint[i]->DriveVelocity(2.0f);

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

			/// Wall
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
			
			/// Arrow Towers
			PxVec3 tower1 = { 85.0f, 4.0f, 0.0f }, tower2 = { -85.0f, 4.0f, -100.0f }, temp = tower1;
			for (int i = 0; i < 2; i++)
			{
				tower[i] = new Towers(PxTransform(temp));
				tower[i]->Color(PxVec3(255.0f / 255.0f, 25.0f / 255.0f, 0.0f));
				tower[i]->SetPlayerTarget(player);
				tower[i]->Enabled(true);
				tower[i]->getArrow()->Name("Arrow");
				tower[i]->SetKinematic(true);
				tower[i]->arrow->SetupFiltering(FilterGroup::WEAPON, FilterGroup::PLAYER);
				Add(tower[i]);
				Add(tower[i]->getArrow());

				temp = tower2;
			}

			PxVec3 endChaserPos = { -80.0f, 0.5f, -325.0f };
			srand(time(0));
			for (int i = 0; i < 35; i++)
			{
				float z = (rand() % -25) + -325;

				Enemy* endChaser = new Chaser(PxTransform(PxVec3(endChaserPos.x, endChaserPos.y, z)), PxVec3(1.0f, 2.0f, 1.0f), 2.0f);
				endChaser->Name("End Chaser" + i);
				endChaser->Color(PxVec3(1.0f, 0.0f, 0.0f));
				Add(endChaser);

				endChaserPos.x += 5.0f;

			}

#pragma endregion


			// Castle
			castle = new Castle(PxTransform(PxVec3(0.0f, 0.5f, -400.0f)));
			castle->SetKinematic(true);
			Add(castle);

			// Cloth

			/// Cloth on Cloth Collision
			px_scene->setClothInterCollisionDistance(0.5f);
			px_scene->setClothInterCollisionStiffness(1.0f);

			/// Init Cloths
			PxVec3 clothOffset = { 25.0f, 49.0f, -397.0f };
			for (int i = 0; i < 4; i++)
			{
				clothCount++;

				cloth[i] = new Cloth(PxTransform(clothOffset), PxVec2(25.0f, 45.0f), PxU32(10), PxU32(10), true);
				((PxCloth*)cloth[i]->Get())->setClothFlags(PxClothFlag::eGPU | PxClothFlag::eSCENE_COLLISION | PxClothFlag::eSWEPT_CONTACT);		// Swept = Computationally Expensive
				((PxCloth*)cloth[i]->Get())->setName("Tappastry" + i);
				cloth[i]->Color(PxVec3(175.0f / 255.0f, 50.0f / 255.0f, 175.0f / 255.0f));
				((PxCloth*)cloth[i]->Get())->setExternalAcceleration(PxVec3(5.0f, 0.0f, 5.0f));
				((PxCloth*)cloth[i]->Get())->setSelfCollisionStiffness(1.0f);
				((PxCloth*)cloth[i]->Get())->setSelfCollisionDistance(0.1f);
				Add(cloth[i]);

				if (i == 1)
				{
					clothOffset.x -= 150.0f;
					continue;
				}

				clothOffset.x += 35.0f;

			}

			// Flags
			flag = new Cloth(PxTransform(PxVec3(0.0f, 75.0f, -402.0f), PxQuat(3 * (PxPi / 2), PxVec3(0.0f, 0.0f, 1.0f))), PxVec2(15.0f, 25.0f), PxU32(10), PxU32(10), true);
			flag->Name("Flag");
			((PxCloth*)flag->Get())->setExternalAcceleration(PxVec3(8.0f, 1.0f, 2.5f));
			flag->Color(PxVec3(175.0f / 255.0f, 50.0f / 255.0f, 175.0f / 255.0f));
			Add(flag);

			flagPole = new Box(PxTransform(PxVec3(0.0f, 63.0f, -402.0f)), PxVec3(1.0f, 12.0f, 1.0f));
			flagPole->Color(PxVec3(1.0f, 0.2f, 0.8f));
			flagPole->SetKinematic(true);
			flagPole->Name("Flag Pole");
			Add(flagPole);


		}

		// Custom update function
		virtual void CustomUpdate(PxReal dt)
		{

			// Player Update
			player->Update();
			trebuchetBase->Update();
			for (int i = 0; i < 2; i++)
			{
				tower[i]->Update(dt);
			}
			
			//Enemy Updates
			for (int i = 0; i < 4; i++)
			{
				heavyEnemy[i]->Update();

				if (i < 2)
					chaserEnemy[i]->Update();
			}

			// Morning Star Handles
			PxVec3 offset = { 0.0f, 1.0f, 1.5f };
			for (int i = 0; i < 5; i++)
			{
				((PxRigidBody*)handle[i]->Get())->setGlobalPose(PxTransform(((PxRigidBody*)heavyEnemy[i]->Get())->getGlobalPose().p + offset, PxQuat(PxIdentity)));
			}

			///printf("Timer: %f \n", ballTimer);

			// Timer Checking
			if (kickTime)
			{
				ballTimer -= dt;
			}


			// Reset Trebuchet on Timer
			if (ballTimer <= 0.0f)
			{
				printf("TREBUCHET: Resetting Trebuchet\n");
				
				trebuchetJoint->DriveVelocity(40.0f);

				PxVec3 ballOffset = { 0.1f, 0.0f, 7.5f };
				((PxRigidBody*)ball->Get())->setGlobalPose(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + ballOffset));
				((PxRigidBody*)ball->Get())->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
				((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

				kickTime = false;
				ballTimer = BALL_TIMER_CONST;
			}

			// Collision Triggers

			if (my_callback->playerHit)
				SpawnPlayerBlood();

			if (my_callback->playerTrigger)
			{
				PlayerToTrebuchet();
				my_callback->playerTrigger = false;
			}

			if (my_callback->scoreTrigger)
				GoalTrigger(testCase);

			if(!my_callback->restart)
				chronoRestart.resetChronoTimer();

			if (my_callback->restart && playerController != PlayerController::trebuchetControls)
			{
				player->done = true;
				restartTimer -= chronoRestart.getChronoTime() * dt;
				
				if (restartTimer <= 0.0f)
				{
					restartTimer = RESTART_TIMER_CONST;
					PhysicsEngine::DynamicActor::dynamicCount = 0;
					clothCount = 0;
					this->Reset();
				}
			}

			//printf("Reset Timer: %f \n", restartTimer);
		}

		// Switch to Trebuchet Controls
		void PlayerToTrebuchet()
		{
			// Change Control Scheme
			playerController = trebuchetControls;
			printf("PLAYER CONTROLS: Control scheme to Trebuchet\n");

			// Setup Player for Trebuchet
			((PxRigidBody*)player->Get())->clearForce();
			player->SetKinematic(true);
			PxVec3 playerOffset = { 5.0f, 1.0f, 0.0f };
			((PxRigidBody*)player->Get())->setGlobalPose(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + playerOffset));

			// Setup Ball for Trebuchet
			player->SetBallTarget(nullptr);
			((PxRigidBody*)ball->Get())->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
			PxVec3 ballOffset = { 0.0f, 0.0f, 7.5f };
			((PxRigidBody*)ball->Get())->setGlobalPose(PxTransform(((PxRigidBody*)trebuchetBase->Get())->getGlobalPose().p + ballOffset)); /// 90 DEGREE ROTATION: PxQuat(PxPi / 2, PxVec3(0.0f, 1.0f, 0.0f))

			// Simulation Flag Active >> Acting as Collision Filter
			for each (PxShape* shape in ball->GetShapes())					/// Ball needs to interact with world at this time
				shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);		

			// Disable all Enemies
			for (int i = 0; i < 5; i++)
			{
				heavyEnemy[i]->SetChaseTarget(nullptr);
				((PxRigidBody*)heavyEnemy[i]->Get())->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
				((PxRigidBody*)heavyEnemy[i]->Get())->setGlobalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

				if (i < 2)
				{
					chaserEnemy[i]->SetChaseTarget(nullptr);
					((PxRigidBody*)chaserEnemy[i]->Get())->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
					((PxRigidBody*)chaserEnemy[i]->Get())->setGlobalPose(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

					tower[i]->Enabled(false);
				}
			}

			printf("ENEMIES: Enemies Disabled -> Trebuchet\n");
		}


		// Goal Score Reaction
		void GoalTrigger(bool testCase)
		{
			my_callback->scoreTrigger = false;

			int colorIndex = 0;
			PxVec3 ballPositions = { 0.0f, 2.0f, -350.0f };

			if (testCase)
			{
				// Create Random Balls
				for (int i = 0; i < 100; i++)
				{
					Box* box = new Box(PxTransform(ballPositions), PxVec3(1.0f, 1.0f, 1.0f), 100.0f);
					box->Material(CreateMaterial(0.5f, 0.5f, 1.5f));
					box->Name("Score Ball\n");
					box->Color(scoreColors[colorIndex++]);
					Add(box);

					if (colorIndex > 3)
						colorIndex = 0;

					if(i % 25 == 0)
						ballPositions.y += 10.0f;
				}
			}

			PxVec3 ballPositionsCastle = { -75.0f, 75.0f, -410.0f };
			// Create Random Balls
			for (int i = 0; i < 100; i++)
			{
				Box* box = new Box(PxTransform(ballPositionsCastle), PxVec3(1.0f, 1.0f, 1.0f), 100.0f);
				box->Material(CreateMaterial(0.5f, 0.5f, 1.5f));
				box->Name("Score Ball\n");
				box->Color(scoreColors[colorIndex++]);
				Add(box);

				if (colorIndex > 3)
					colorIndex = 0;

				if (i % 50 == 0)
					ballPositionsCastle.x = -ballPositionsCastle.x;
			}

		}

		// Spawn Number of Balls Specified
		void SpawnBalls(int numberofBalls, bool inFormation)
		{
			PxVec3 pos = { 0.0f, 20.0f, -50.0f };

			for (int i = 0; i < numberofBalls; i++)
			{
				if (i % 10 == 0 && inFormation)
				{
					pos.y += 5.0f;
					pos.x = 0.0f;
				}
				Ball* ball = new Ball(PxTransform(((PxRigidBody*)player->Get())->getGlobalPose().p + pos));
				ball->Color(color_palette[2]);
				ball->Name("Test Ball" + i);
				PxMaterial* ballMaterial = GetPhysics()->createMaterial(0.2f, 0.5f, 1.25f);
				ball->Material(ballMaterial);
				Add(ball);

				spawnBalls.push_back(ball);

				if(inFormation)
					pos.x += 5.0f;

			}
		}

		void SpawnCloth(int numberofCloth)
		{
			PxVec3 pos = { -10.0f, 20.0f, -50.0f };

			for (int i = 0; i < numberofCloth; i++)
			{
				Cloth* cloth = new Cloth(PxTransform(((PxRigidBody*)player->Get())->getGlobalPose().p + pos), PxVec2(50.0f, 50.0f), PxU32(10), PxU32(10), false);
				((PxCloth*)cloth->Get())->setClothFlags(PxClothFlag::eGPU | PxClothFlag::eSCENE_COLLISION);
				((PxCloth*)cloth->Get())->setName("Test Cloth" + i);
				cloth->Color(PxVec3(200.0f / 255.0f, 175.0f / 255.0f, 50.0f / 255.0f));
				((PxCloth*)cloth->Get())->setExternalAcceleration(PxVec3(0.25f, 8.0f, -0.5f));
				((PxCloth*)cloth->Get())->setSelfCollisionStiffness(1.0f);
				((PxCloth*)cloth->Get())->setSelfCollisionDistance(0.1f);
				Add(cloth);

				clothCount++;
			}
		}

		// Spawns blood at point of collision between Player and Arrow
		void SpawnPlayerBlood()
		{
			my_callback->playerHit = false;

			PxVec3 playerPos = ((PxRigidBody*)player->Get())->getGlobalPose().p;

			for (int i = 0; i < 3; i++)
			{
				Box* bloodBox = new Box(PxTransform(playerPos + PxVec3(0.0f, 0.5f, 0.25f)), PxVec3(0.25f, 0.25f, 0.25f), 0.01f);
				bloodBox->Color(PxVec3(1.0f, 0.0f, 0.0f));
				Add(bloodBox);

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
