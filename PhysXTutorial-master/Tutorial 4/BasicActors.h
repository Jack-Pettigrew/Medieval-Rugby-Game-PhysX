#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	///Plane class
	class Plane : public StaticActor
	{
	public:
		//A plane with default paramters: XZ plane centred at (0,0,0)
		Plane(PxVec3 normal=PxVec3(0.f, 1.f, 0.f), PxReal distance=0.f) 
			: StaticActor(PxTransformFromPlaneEquation(PxPlane(normal, distance)))
		{
			CreateShape(PxPlaneGeometry());
		}
	};

	///Sphere class
	class Sphere : public DynamicActor
	{
	public:
		//a sphere with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m
		// - denisty: 1kg/m^3
		Sphere(const PxTransform& pose=PxTransform(PxIdentity), PxReal radius=1.f, PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxSphereGeometry(radius), density);
		}
	};

	///Box class
	class Box : public DynamicActor
	{
	public:
		//a Box with default parameters:
		// - pose in 0,0,0
		// - dimensions: 1m x 1m x 1m
		// - denisty: 1kg/m^3
		Box(const PxTransform& pose=PxTransform(PxIdentity), PxVec3 dimensions=PxVec3(.5f,.5f,.5f), PxReal density=1.f) 
			: DynamicActor(pose)
		{ 
			CreateShape(PxBoxGeometry(dimensions), density);
		}
	};

	class Capsule : public DynamicActor
	{
	public:
		Capsule(const PxTransform& pose=PxTransform(PxIdentity), PxVec2 dimensions=PxVec2(1.f,1.f), PxReal density=1.f) 
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	// Ball Class
	class Ball : public DynamicActor
	{
	public:
		Ball(const PxTransform& pose = PxTransform(PxIdentity), PxReal radius = 1.f, PxReal density = 1.f)
			:DynamicActor(pose)
		{
			// Ball Shape
			CreateShape(PxSphereGeometry(radius), density);		   /// 0
			CreateShape(PxSphereGeometry(radius * 0.75), density); /// 1
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.5f, 0.0f, 0.0f)));
			CreateShape(PxSphereGeometry(radius * 0.75), density); /// 2
			GetShape(2)->setLocalPose(PxTransform(PxVec3(-0.5f, 0.0f, 0.0f)));
			CreateShape(PxSphereGeometry(radius * 0.50), density); /// 3
			GetShape(3)->setLocalPose(PxTransform(PxVec3(1.0f, 0.0f, 0.0f)));
			CreateShape(PxSphereGeometry(radius * 0.50), density); /// 4
			GetShape(4)->setLocalPose(PxTransform(PxVec3(-1.0f, 0.0f, 0.0f)));
			CreateShape(PxSphereGeometry(radius * 0.25), density); /// 5
			GetShape(5)->setLocalPose(PxTransform(PxVec3(1.4f, 0.0f, 0.0f)));
			CreateShape(PxSphereGeometry(radius * 0.25), density); /// 6 
			GetShape(6)->setLocalPose(PxTransform(PxVec3(-1.4f, 0.0f, 0.0f)));
		}

		void Update()
		{
			
		}
	};
	// Player Class
	class Player : public DynamicActor
	{
	private:

		PxRigidBody* ballTarget;
		float maxSpeed = 19.0f;
		float speed = 50.0f;
		PxVec3 ballOffset = { -3.0f, -0.5f, 0.0f };

	public:

		bool forward = false, back = false, left = false, right = false;
		bool follow = true, done = false;

		Player(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(1.0f, 2.0f, 1.0f), PxReal density = 2.0f)
			: DynamicActor(pose)
		{

			Create(dimensions, density);

		}

		// Creates Player Shape
		void Create(PxVec3 dimensions, PxReal density)
		{

			CreateShape(PxBoxGeometry(dimensions), density);

			// CollisionShape (1)
			CreateShape(PxBoxGeometry(dimensions), density);
			GetShape(1)->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			GetShape(1)->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);

		}

		// Player Update
		void Update()
		{

			Movement();

			PxVec3 position = ((PxRigidBody*)this->Get())->getGlobalPose().p;

			// Ball follow Trebuchet
			if (ballTarget != nullptr && follow)
				ballTarget->setGlobalPose(PxTransform(position - ballOffset, PxQuat(PxIdentity)));

		}

		// Handles Player Movement
		void Movement()
		{
			// Limit Player Speed (Check Magnitude)
			if (((PxRigidBody*)this->Get())->getLinearVelocity().normalize() > maxSpeed)
				((PxRigidBody*)this->Get())->setLinearVelocity(((PxRigidBody*)this->Get())->getLinearVelocity().getNormalized() * maxSpeed);

			if (forward)
				((PxRigidBody*)this->Get())->addForce(PxVec3(0.0f, 0.0f, -1.0f) * speed, PxForceMode::eIMPULSE);
			else if (back)
				((PxRigidBody*)this->Get())->addForce(PxVec3(0.0f, 0.0f, 1.0f) * speed, PxForceMode::eIMPULSE);

			if (left)
				((PxRigidBody*)this->Get())->addForce(PxVec3(-1.0f, 0.0f, 0.0f) * speed, PxForceMode::eIMPULSE);
			else if (right)
				((PxRigidBody*)this->Get())->addForce(PxVec3(1.0f, 0.0f, 0.0f) * speed, PxForceMode::eIMPULSE);

		}

		// Set Ball Follow Target
		void SetBallTarget(PxRigidBody* ball)
		{
			ballTarget = ball;
		}
	};

	// Enemy Base Class
	class Enemy : public DynamicActor
	{
	public:

		// Enemy Done Flag
		bool done = false;

		float maxSpeed = 22.0f;
		float speed = 44.0f;

		PxRigidBody* targetToChase;

		Enemy(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = PxReal(1.0f))
			: DynamicActor(pose)
		{
			//CreateShape(PxBoxGeometry(dimensions), density);
			//this->Name("Default Enemy");
			///printf("* Default Enemy Created, base properies in use! *\n");
		}

		// Base Init
		virtual void Init()
		{
			this->Color(PxVec3(61.0f / 255.0f, 61.0f / 255.0f, 61.0f / 255.0f));
		}

		// Enemy Update Class
		void Update()
		{
			if (done)
				return;

			Movement();
			LimitSpeed();
			Attack();
		}

		// Base Movement
		virtual void Movement()
		{
			///printf("Default Movement method called \n");
		}

		// Limit Speed
		void LimitSpeed()
		{

			if (((PxRigidBody*)this->Get())->getLinearVelocity().normalize() > maxSpeed)
				((PxRigidBody*)this->Get())->setLinearVelocity(((PxRigidBody*)this->Get())->getLinearVelocity().getNormalized() * maxSpeed);
		}

		// Set Current Target
		void SetChaseTarget(PxActor* target)
		{
			targetToChase = ((PxRigidBody*)target);

			if (!targetToChase)
			{
				printf("No Target Assigned\n");
				return;
			}

			printf("%s's target changed to %s \n", this->Get()->getName(), target->getName());
		}

		// Base Attack
		virtual void Attack()
		{

		}

		virtual void SetSpeed()
		{

		}

	};

	// Chaser Enemy Class
	class Chaser : public Enemy
	{
	private:

		float attackForce = 400.0f;

	public:

		Chaser(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = PxReal(2.0f))
			: Enemy(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			this->Name("Chaser");

			maxSpeed = 20.0f;
			speed = 100.0f;
		}

		void Init()
		{

			this->Color(PxVec3(100.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f));

		}

		void Movement()
		{

			///printf("Chaser Movement called!\n");

			if (targetToChase == nullptr)
				return;

			PxVec3 a = ((PxRigidActor*)this->Get())->getGlobalPose().p;
			PxVec3 b = targetToChase->getGlobalPose().p;

			PxVec3 direction = b - a;

			((PxRigidBody*)this->Get())->addForce(direction * speed);

		}

		// Chaser Attack
		void Attack()
		{
			if (!targetToChase)
				return;

			PxVec3 a = ((PxRigidActor*)this->Get())->getGlobalPose().p;
			PxVec3 b = targetToChase->getGlobalPose().p;
			PxVec3 targetDirection = (b - a);

			// Stop force when hit

			if (targetDirection.magnitude() < 7.0f)
			{
				((PxRigidBody*)this->Get())->addForce(targetDirection * attackForce, PxForceMode::eIMPULSE);
				((PxRigidBody*)this->Get())->addTorque(PxVec3(20.0f, 0.0f, 0.0f) * 125.0f, PxForceMode::eIMPULSE);
				done = true;
			}
			else if (!done)
				((PxRigidBody*)this->Get())->setAngularVelocity(PxZero);

		}
	};

	// Heavy Enemy Class
	class Heavy : public Enemy
	{
	public:

		Heavy(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = PxReal(2.0f))
			: Enemy(pose)
		{
			// Body
			CreateShape(PxBoxGeometry(dimensions), density);
			this->Name("Heavy");

			maxSpeed = 12.0f;
			speed = 20.0f;
		}

		void Init(PxVec3 color = PxVec3(61.0f / 255.0f, 61.0f / 255.0f, 61.0f / 255.0f))
		{
			this->Color(color);
		}

		void Movement()
		{
			///printf("Heavy Movement called!\n");

			if (!targetToChase)
				return;

			PxVec3 c = ((PxRigidActor*)this->Get())->getGlobalPose().p;
			PxVec3 d = targetToChase->getGlobalPose().p;
			PxVec3 direction2 = (d - c);
			PxQuat rotation = ((PxRigidBody*)this->Get())->getGlobalPose().q;

			((PxRigidBody*)this->Get())->addForce(direction2 * speed);
		}

	};

	// Weapon Class
	class MorningStar : public DynamicActor
	{
	public:

		MorningStar(const PxTransform& pose = PxTransform(PxIdentity), PxReal ballRadius = PxReal(1.0f), PxReal ballDensity = PxReal(1.0f))
			: DynamicActor(pose)
		{
			// Handle
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 0.5f, 0.5f)), ballDensity);
			this->Name("Morning Star");

			// Ball
			CreateShape(PxSphereGeometry(ballRadius), ballDensity);
		}

	};

	// Trebuchet Component Class
	class TrebuchetBase : public DynamicActor
	{
	private:

		float turnForce = 20.0f;

	public:
		PxVec3 position, ballOffset = { -0.3f, -0.5f, -3.5f };
		PxRigidBody* ballTarget;
		bool follow = true;

		float kickLength = 55.0f;
		float kickHeight = 80.0f;

		bool left = false, right = false;

		TrebuchetBase(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = PxReal(1.0f))
			: DynamicActor(pose)
		{

			CreateShape(PxBoxGeometry(PxVec3(3.0f, 0.4f, 1.0f)), density);
			this->Name("Trebuchet Base");

			CreateShape(PxBoxGeometry(PxVec3(0.2f, 1.0f, 2.0f)), density);
			GetShape(1)->setLocalPose(PxTransform(PxVec3(3.0f, 0.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(0.2f, 1.0f, 2.0f)), density);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(-3.0f, 0.0f, 0.0f)));

		}

		void Update()
		{
			PxVec3 position = ((PxRigidBody*)this->Get())->getGlobalPose().p;

			if (left)
				((PxRigidBody*)this->Get())->addTorque(PxVec3(0.0f, -1.0f, 0.0f) * turnForce);
			else if (right)
				((PxRigidBody*)this->Get())->addTorque(PxVec3(0.0f, 1.0f, 0.0f) * turnForce);


			// Ball follow Trebuchet
			if (ballTarget != nullptr && follow)
				ballTarget->setGlobalPose(PxTransform(position - ballOffset, PxQuat(PxIdentity)));

		}

		// Releases Ball and Applies Force
		void Kick()
		{
			// If no target to follow...
			if (ballTarget == nullptr)
				return;

			follow = false;

			ballTarget->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);

			ballTarget = nullptr;
		}

		void SetBallTarget(PxRigidBody* ball)
		{
			ballTarget = ball;
		}
	};

	// Trebuchet Component Class
	class TrebuchetArm : public DynamicActor
	{
	public:

		TrebuchetArm(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = PxReal(1.0f))
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 0.1f, 3.0f)), density);

			CreateShape(PxBoxGeometry(PxVec3(1.0f, 0.1f, 1.0f)), density);

			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.0f, 0.0f, 3.5f)));

			// Angle Barrier
			CreateShape(PxBoxGeometry(PxVec3(1.0f, 0.5f, 0.1f)), density);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(0.0f, 0.5f, 5.0f), PxQuat(PxPi / 4, PxVec3(1.0f, 0.0f, 0.0f))));
		}

	};

	// Goal Class
	class Goal : public StaticActor
	{
	public:
		Goal(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 100.0f)
			: StaticActor(pose)
		{
			Init(density);
		}

		void Init(PxReal density)
		{
			// Post
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 5.0f, 0.5f)), density);

			// Post Arms
			CreateShape(PxBoxGeometry(PxVec3(10.0f, 0.5f, 0.5f)), density);
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.0f, 5.0f, 0.0f), PxQuat(PxIdentity)));

			// Post Left
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 8.0f, 0.5f)), density);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(9.5f, 13.0f, 0.0f), PxQuat(PxIdentity)));

			// Post Right
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 8.0f, 0.5f)), density);
			GetShape(3)->setLocalPose(PxTransform(PxVec3(-9.5f, 13.0f, 0.0f), PxQuat(PxIdentity)));

			// Stand
			CreateShape(PxBoxGeometry(PxVec3(2.0f, 2.0f, 2.0f)), density);
			GetShape(4)->setLocalPose(PxTransform(PxVec3(0.0f, -3.0f, 0.0f)));
		}
	};

	// Bleachers Class
	class Bleachers : public DynamicActor
	{
	public:
		Bleachers(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 100.0f)
			: DynamicActor(pose)
		{
			Init(density);
		}

		void Init(PxReal density)
		{
			// Right
			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(0)->setLocalPose(PxTransform(PxVec3(100.0f, 0.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(1)->setLocalPose(PxTransform(PxVec3(105.0f, 3.5f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(110.0f, 7.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(3)->setLocalPose(PxTransform(PxVec3(115.0f, 10.5f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(4)->setLocalPose(PxTransform(PxVec3(120.0f, 14.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(0.5f, 4.0f, 400.0f)), density);
			GetShape(5)->setLocalPose(PxTransform(PxVec3(94.0f, 0.0f, 0.0f)));

			// Left
			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(6)->setLocalPose(PxTransform(PxVec3(-100.0f, 0.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(7)->setLocalPose(PxTransform(PxVec3(-105.0f, 3.5f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(8)->setLocalPose(PxTransform(PxVec3(-110.0f, 7.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(9)->setLocalPose(PxTransform(PxVec3(-115.0f, 10.5f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(6.0f, 2.0f, 400.0f)), density);
			GetShape(10)->setLocalPose(PxTransform(PxVec3(-120.0f, 14.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(0.5f, 4.0f, 400.0f)), density);
			GetShape(11)->setLocalPose(PxTransform(PxVec3(-94.0f, 0.0f, 0.0f)));

		}
	};

	class Wall : public DynamicActor
	{
	public:
		Wall(const PxTransform &pose = PxTransform(PxIdentity), PxVec3 wallSize = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(wallSize), density);
		}

		void ChangeWallSize(PxVec3 newWallSize)
		{
			GetShape(0)->setGeometry(PxBoxGeometry(newWallSize));
		}
	};

	class Castle : public DynamicActor
	{
	public:

		Castle(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 1.0f)
			: DynamicActor(pose)
		{
			PxVec3 castlePos = { -120.0f, 50.0f, 0.0f };
			for (int i = 0; i < 25; i++)
			{
				CreateShape(PxBoxGeometry(3.0f, 4.0f, 1.0), density);
				GetShape(i)->setLocalPose(PxTransform(castlePos));
				castlePos.x += 10.0f;
			}

			CreateShape(PxBoxGeometry(PxVec3(50.0f, 50.0f, 1.0f)), density);
			GetShape(25)->setLocalPose(PxTransform(PxVec3(-75.0f, 0.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(50.0f, 50.0f, 1.0f)), density);
			GetShape(26)->setLocalPose(PxTransform(PxVec3(75.0f, 0.0f, 0.0f)));

			CreateShape(PxBoxGeometry(PxVec3(50.0f, 2.0f, 1.0f)), density);
			GetShape(27)->setLocalPose(PxTransform(PxVec3(0.0f, 48.0f, 0.0f)));

		}
	};

	// RotatingObstacle Class
	class RotatingObstacle : public DynamicActor
	{
	public:

		RotatingObstacle(const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 20000.0f)
			: DynamicActor(pose)
		{

			CreateShape(PxBoxGeometry(PxVec3(1.0, 14.0f, 1.0f)), density);
			CreateShape(PxBoxGeometry(PxVec3(1.0f, 1.0f, 14.0f)), density);

		}
	};

	// The ConvexMesh class
	class ConvexMesh : public DynamicActor
	{
	public:
		//constructor
		ConvexMesh(const std::vector<PxVec3>& verts, const PxTransform& pose = PxTransform(PxIdentity), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			PxConvexMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
			mesh_desc.vertexLimit = 256;

			CreateShape(PxConvexMeshGeometry(CookMesh(mesh_desc)), density);
		}

		//mesh cooking (preparation)
		PxConvexMesh* CookMesh(const PxConvexMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if (!GetCooking()->cookConvexMesh(mesh_desc, stream))
				throw new Exception("ConvexMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createConvexMesh(input);
		}
	};

	// The TriangleMesh class
	class TriangleMesh : public StaticActor
	{
	public:
		//constructor
		TriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose = PxTransform(PxIdentity))
			: StaticActor(pose)
		{
			PxTriangleMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.triangles.count = (PxU32)trigs.size();
			mesh_desc.triangles.stride = 3 * sizeof(PxU32);
			mesh_desc.triangles.data = &trigs.front();

			CreateShape(PxTriangleMeshGeometry(CookMesh(mesh_desc)));
		}

		//mesh cooking (preparation)
		PxTriangleMesh* CookMesh(const PxTriangleMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if (!GetCooking()->cookTriangleMesh(mesh_desc, stream))
				throw new Exception("TriangleMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createTriangleMesh(input);
		}
	};

	//Distance joint with the springs switched on
	class DistanceJoint : public Joint
	{
	public:
		DistanceJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = (PxJoint*)PxDistanceJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
			((PxDistanceJoint*)joint)->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
			Damping(1.f);
			Stiffness(1.f);
		}

		void Stiffness(PxReal value)
		{
			((PxDistanceJoint*)joint)->setStiffness(value);
		}

		PxReal Stiffness()
		{
			return ((PxDistanceJoint*)joint)->getStiffness();		
		}

		void Damping(PxReal value)
		{
			((PxDistanceJoint*)joint)->setDamping(value);
		}

		PxReal Damping()
		{
			return ((PxDistanceJoint*)joint)->getDamping();
		}
	};

	///Revolute Joint
	class RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = PxRevoluteJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION,true);
		}

		void DriveVelocity(PxReal value)
		{
			//wake up the attached actors
			PxRigidDynamic *actor_0, *actor_1;
			((PxRevoluteJoint*)joint)->getActors((PxRigidActor*&)actor_0, (PxRigidActor*&)actor_1);
			if (actor_0)
			{
				if (actor_0->isSleeping())
					actor_0->wakeUp();
			}
			if (actor_1)
			{
				if (actor_1->isSleeping())
					actor_1->wakeUp();
			}
			((PxRevoluteJoint*)joint)->setDriveVelocity(value);
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
		}

		PxReal DriveVelocity()
		{
			return ((PxRevoluteJoint*)joint)->getDriveVelocity();
		}

		void SetLimits(PxReal lower, PxReal upper)
		{
			((PxRevoluteJoint*)joint)->setLimit(PxJointAngularLimitPair(lower, upper));
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
		}

		void SetLimitsByDegrees(PxReal lower, PxReal upper)
		{
			((PxRevoluteJoint*)joint)->setLimit(PxJointAngularLimitPair(lower * (PxPi / 180), upper * (PxPi / 180)));
			((PxRevoluteJoint*)joint)->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
		}
	};

	class Cloth : public Actor
	{
		PxClothMeshDesc mesh_desc;

	public:
		//constructor
		Cloth(PxTransform pose=PxTransform(PxIdentity), const PxVec2& size=PxVec2(1.f,1.f), PxU32 width=1, PxU32 height=1, bool fix_top = true)
		{
			//prepare vertices
			PxReal w_step = size.x/width;
			PxReal h_step = size.y/height;

			PxClothParticle* vertices = new PxClothParticle[(width+1)*(height+1)*4];
			PxU32* quads = new PxU32[width*height*4];

			for (PxU32 j = 0; j < (height+1); j++)
			{
				for (PxU32 i = 0; i < (width+1); i++)
				{
					PxU32 offset = i + j*(width+1);
					vertices[offset].pos = PxVec3(w_step*i,0.f,h_step*j);
					if (fix_top && (j == 0)) //fix the top row of vertices
						vertices[offset].invWeight = 0.f;
					else
						vertices[offset].invWeight = 1.f;
				}

				for (PxU32 j = 0; j < height; j++)
				{
					for (PxU32 i = 0; i < width; i++)
					{
						PxU32 offset = (i + j*width)*4;
						quads[offset + 0] = (i+0) + (j+0)*(width+1);
						quads[offset + 1] = (i+1) + (j+0)*(width+1);
						quads[offset + 2] = (i+1) + (j+1)*(width+1);
						quads[offset + 3] = (i+0) + (j+1)*(width+1);
					}
				}
			}

			//init cloth mesh description
			mesh_desc.points.data = vertices;
			mesh_desc.points.count = (width+1)*(height+1);
			mesh_desc.points.stride = sizeof(PxClothParticle);

			mesh_desc.invMasses.data = &vertices->invWeight;
			mesh_desc.invMasses.count = (width+1)*(height+1);
			mesh_desc.invMasses.stride = sizeof(PxClothParticle);

			mesh_desc.quads.data = quads;
			mesh_desc.quads.count = width*height;
			mesh_desc.quads.stride = sizeof(PxU32) * 4;

			//create cloth fabric (cooking)
			PxClothFabric* fabric = PxClothFabricCreate(*GetPhysics(), mesh_desc, PxVec3(0, -1, 0));

			//create cloth
			actor = (PxActor*)GetPhysics()->createCloth(pose, *fabric, vertices, PxClothFlags());
			//collisions with the scene objects
			((PxCloth*)actor)->setClothFlag(PxClothFlag::eSCENE_COLLISION, true);

			colors.push_back(default_color);
			actor->userData = new UserData(&colors.back(), &mesh_desc);
		}

		~Cloth()
		{
			delete (UserData*)actor->userData;		
		}
	};
}