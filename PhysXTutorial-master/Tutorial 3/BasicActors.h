#pragma once

#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	// Basic Actors Classes

	// Plane class
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

	// Sphere class
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

	// Box class
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

	// Create Capsule
	class Capsule : public DynamicActor
	{
	public:
		Capsule(const PxTransform& pose=PxTransform(PxIdentity), PxVec2 dimensions=PxVec2(1.f,1.f), PxReal density=1.f) 
			: DynamicActor(pose)
		{
			CreateShape(PxCapsuleGeometry(dimensions.x, dimensions.y), density);
		}
	};

	// Player Class
	class Player : public DynamicActor
	{
	private:
		
		
	public:

		float speed = 50.0f;

		bool forward = false, back = false, left = false, right = false;

		Player(const PxTransform& pose = PxTransform(PxIdentity), PxVec2 dimensions = PxVec2(2.0f, 2.0f), PxReal density = 2.0f)
			: DynamicActor(pose)
		{

			Create(dimensions, density);

		}

		void Create(PxVec2 capsuleDimensions, PxReal capsuleDensity)
		{
			// Body 0
			CreateShape(PxCapsuleGeometry(capsuleDimensions.x, capsuleDimensions.y), capsuleDensity);
			GetShape(0)->setLocalPose(PxTransform(PxVec3(PxIdentity), PxQuat(PxReal(PxPi / 2), PxVec3(0.0f, 0.0f, 1.0f))));

			// Leg 1
			CreateShape(PxSphereGeometry(PxReal(1.0f)), 1.0f);
			GetShape(1)->setLocalPose(PxTransform(PxVec3(2.0f, -3.0f, 0.0f), PxQuat(PxIdentity)));
			
			// Leg 3
			CreateShape(PxSphereGeometry(PxReal(1.0f)), 1.0f);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(-2.0f, -3.0f, 0.0f), PxQuat(PxIdentity)));

			// Foot 4
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 0.5f, 0.5f)), 1.0f);
			GetShape(3)->setLocalPose(PxTransform(PxVec3(-2.0f, -5.0f, 0.0f), PxQuat(PxIdentity)));
		}

		void Update()
		{
			Movement();
		}

		void Movement()
		{

			// Limit Player Speed (Check Magnitude)
			if (((PxRigidBody*)this->Get())->getLinearVelocity().normalize() > speed)
				((PxRigidBody*)this->Get())->setLinearVelocity(((PxRigidBody*)this->Get())->getLinearVelocity().getNormalized() * 49.0f);

			if(forward)
				((PxRigidBody*)this->Get())->addForce(PxVec3(0.0f, 0.0f, -1.0f) * speed, PxForceMode::eIMPULSE);
			else if(back)
				((PxRigidBody*)this->Get())->addForce(PxVec3(0.0f, 0.0f, 1.0f) * speed, PxForceMode::eIMPULSE);

			if(left)
				((PxRigidBody*)this->Get())->addForce(PxVec3(-1.0f, 0.0f, 0.0f) * speed, PxForceMode::eIMPULSE);
			else if (right)
				((PxRigidBody*)this->Get())->addForce(PxVec3(1.0f, 0.0f, 0.0f) * speed, PxForceMode::eIMPULSE);

		}

		// Custom Player Render
		void Render()
		{

		}
	};

	// Enemy Base Class
	class Enemy : public DynamicActor
	{

	public:

		float maxSpeed = 10.0f;

		PxRigidBody* targetToChase;

		Enemy(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = PxReal(1.0f))
			: DynamicActor(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			this->Name("Enemy");
		}

		virtual void Init()
		{
			this->Color(PxVec3(61.0f / 255.0f, 61.0f / 255.0f, 61.0f / 255.0f));
		}

		void Update()
		{
			Movement();
			LimitSpeed();
		}

		virtual void Movement()
		{
			printf("Default Movement method called \n");
		}

		// Limit Speed
		void LimitSpeed()
		{
			if (((PxRigidBody*)this->Get())->getLinearVelocity().normalize() > maxSpeed)
				((PxRigidBody*)this->Get())->setLinearVelocity(((PxRigidBody*)this->Get())->getLinearVelocity().getNormalized() * maxSpeed);
		}

		void SetChaseTarget(PxActor* target)
		{
			targetToChase = ((PxRigidBody*)target);

			if (!targetToChase)
			{
				printf("No Target Assigned\n");
				return;
			}

			printf("Target changed to %s \n", target->getName());
		}

	};

	// Chaser Enemy Class
	class Chaser : public Enemy
	{
	public:

		float maxSpeed = 10.0f;

		Chaser(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = PxReal(1.0f))
			: Enemy(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			this->Name("Chaser");
		}
	
		void Init()
		{

			this->Color(PxVec3(100.0f / 255.0f, 10.0f / 255.0f, 100.0f / 255.0f));

		}

		void Movement()
		{

			printf("Chaser Movement called!\n");

			if (targetToChase == NULL)
				return;

			/*PxVec3 a = ((PxRigidActor*)this->GetShape())->getGlobalPose().p;
			PxVec3 b = targetToChase->getGlobalPose().p;

			PxVec3 direction = a - b;

			((PxRigidBody*)this->GetShape())->addForce(direction);*/

		}
	};

	class Heavy : public Enemy
	{
	public:

		float maxSpeed = 2.0f;

		Heavy(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(0.5f, 0.5f, 0.5f), PxReal density = PxReal(2.0f))
			: Enemy(pose)
		{
			CreateShape(PxBoxGeometry(dimensions), density);
			this->Name("Heavy");
		}

		void Init(PxVec3 color = PxVec3(61.0f / 255.0f, 61.0f / 255.0f, 61.0f / 255.0f))
		{
			this->Color(color);
		}

		void Movement()
		{
			printf("Heavy Movement called!\n");
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
			CreateShape(PxBoxGeometry(PxVec3(5.0f, 0.5f, 0.5f)), density);
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.0f, 5.0f, 0.0f), PxQuat(PxIdentity)));

			// Post Left
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 3.0f, 0.5f)), density);
			GetShape(2)->setLocalPose(PxTransform(PxVec3(5.0f, 7.5f, 0.0f), PxQuat(PxIdentity)));

			// Post Right
			CreateShape(PxBoxGeometry(PxVec3(0.5f, 3.0f, 0.5f)), density);
			GetShape(3)->setLocalPose(PxTransform(PxVec3(-5.0f, 7.5f, 0.0f), PxQuat(PxIdentity)));

		}
	};

	// The ConvexMesh class
	class ConvexMesh : public DynamicActor
	{
	public:
		//constructor
		ConvexMesh(const std::vector<PxVec3>& verts, const PxTransform& pose=PxTransform(PxIdentity), PxReal density=1.f)
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

			if(!GetCooking()->cookConvexMesh(mesh_desc, stream))
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
		TriangleMesh(const std::vector<PxVec3>& verts, const std::vector<PxU32>& trigs, const PxTransform& pose=PxTransform(PxIdentity))
			: StaticActor(pose)
		{
			PxTriangleMeshDesc mesh_desc;
			mesh_desc.points.count = (PxU32)verts.size();
			mesh_desc.points.stride = sizeof(PxVec3);
			mesh_desc.points.data = &verts.front();
			mesh_desc.triangles.count = (PxU32)trigs.size();
			mesh_desc.triangles.stride = 3*sizeof(PxU32);
			mesh_desc.triangles.data = &trigs.front();

			CreateShape(PxTriangleMeshGeometry(CookMesh(mesh_desc)));
		}

		//mesh cooking (preparation)
		PxTriangleMesh* CookMesh(const PxTriangleMeshDesc& mesh_desc)
		{
			PxDefaultMemoryOutputStream stream;

			if(!GetCooking()->cookTriangleMesh(mesh_desc, stream))
				throw new Exception("TriangleMesh::CookMesh, cooking failed.");

			PxDefaultMemoryInputData input(stream.getData(), stream.getSize());

			return GetPhysics()->createTriangleMesh(input);
		}
	};

	// Distance joint with the springs switched on
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

	// Revolute Joint
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
	
}