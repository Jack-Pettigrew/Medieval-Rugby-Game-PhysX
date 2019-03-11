#include <iostream> //cout, cerr
#include <iomanip> //stream formatting
#include <windows.h> //delay function
#include <PxPhysicsAPI.h> //PhysX
#include <PxRigidActor.h> // RigidActor

using namespace std;
using namespace physx;

//PhysX objects
PxPhysics* physics;
PxFoundation* foundation;
debugger::comm::PvdConnection* vd_connection;

//simulation objects
PxScene* scene;
PxRigidDynamic* box;
PxRigidDynamic* box2;
PxRigidDynamic* boxArray[10][10];
PxRigidStatic* plane;

float timer = 0.0f;
int stepsIndex = 0;

///Initialise PhysX objects
bool PxInit()
{
	//default error and allocator callbacks
	static PxDefaultErrorCallback gDefaultErrorCallback;
	static PxDefaultAllocator gDefaultAllocatorCallback;

	//Init PhysX
	//foundation
	foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);

	if(!foundation)
		return false;

	//physics
	physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());

	if(!physics)
		return false;

	//connect to an external visual debugger (if exists)
	vd_connection = PxVisualDebuggerExt::createConnection(physics->getPvdConnectionManager(), "localhost", 5425, 100, 
		PxVisualDebuggerExt::getAllConnectionFlags());

	//create a default scene
	PxSceneDesc sceneDesc(physics->getTolerancesScale());

	if(!sceneDesc.cpuDispatcher)
	{
		PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);
		sceneDesc.cpuDispatcher = mCpuDispatcher;
	}

	if(!sceneDesc.filterShader)
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	scene = physics->createScene(sceneDesc);

	if (!scene)
		return false;

	return true;
}

/// Release all allocated resources
void PxRelease()
{
	if (scene)
		scene->release();
	if (vd_connection)
		vd_connection->release();
	if (physics)
		physics->release();
	if (foundation)
		foundation->release();
}

///Initialise the scene
void InitScene()
{
	//default gravity
	scene->setGravity(PxVec3(0.f, -9.81f, 0.f));

	//materials
	PxMaterial* default_material = physics->createMaterial(0.0f, 0.1f, 0.3f);   //static friction, dynamic friction, restitution

	//create a static plane (XZ)
	plane = PxCreatePlane(*physics, PxPlane(PxVec3(0.f, 1.f, 0.f), 0.f), *default_material);

	scene->addActor(*plane);

	//create a dynamic actor and place it 10 m above the ground
	box = CreateBox();
	box = physics->createRigidDynamic(PxTransform(PxVec3(0.5f, 100.0f, 0.0f)));
	box2 = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 0.5f, 0.0f)));
	box2->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	//create a box shape of 1m x 1m x 1m size (values are provided in halves)
	box->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *default_material);
	box2->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *default_material);

	//update the mass of the box
	PxRigidBodyExt::updateMassAndInertia(*box, 1.0f); //density of 1kg/m^3
	PxRigidBodyExt::updateMassAndInertia(*box2, 1.0f);
	scene->addActor(*box);
	scene->addActor(*box2);

	// Attempt at Box Fall Wall
	for (int i = 0; i < 10; i++)
	{
		for (int y = 0; y < 10; y++)
		{
			PxRigidDynamic* boxElement;
			boxElement = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 10.0f, 0.0f)));
			boxElement->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *default_material);
			scene->addActor(*boxElement);
		}
	}

	box->addForce(PxVec3(0.0f, 0.0f, 0.0f));
	
}

/// Perform a single simulation step
void Update(PxReal delta_time)
{
	scene->simulate(delta_time);

	scene->fetchResults(true);
}

/// The main function
int main()
{
	//initialise PhysX	
	if (!PxInit())
	{
		cerr << "Could not initialise PhysX." << endl;
		return 0;
	}

	//initialise the scene
	InitScene();

	//set the simulation step to 1/60th of a second
	PxReal delta_time = 1.f/60.f;

	//simulate until the 'Esc' is pressed
	while (!GetAsyncKeyState(VK_ESCAPE))
	{
		//'visualise' position and velocity of the box
		PxVec3 position = box->getGlobalPose().p;
		PxVec3 velocity = box->getLinearVelocity();
		stepsIndex += 1;
		cout << stepsIndex << " ";
		cout << setiosflags(ios::fixed) << setprecision(2) << "x=" << position.x << 
			", y=" << position.y << ", z=" << position.z << ",  ";
		cout << setiosflags(ios::fixed) << setprecision(2) << "vx=" << velocity.x <<
			", vy=" << velocity.y << ", vz=" << velocity.z << ", ";
		cout << "Mass= " << box->getMass() << endl;

		//perform a single simulation step
		Update(delta_time);

		//box2->setKinematicTarget(PxTransform(PxVec3(10.0f, 0.0f, 0.0f)));
		//box2->setGlobalPose(PxTransform(PxVec3(10.f, 0.f, 0.f)));

		/*if (stepsIndex == 10)
		{
			box->setGlobalPose(PxTransform(PxVec3(10.f, 0.f, 0.f)));
		}*/

		//timer += delta_time + 0.1;
		//cout << "Timer: " << timer << endl;
		
		//introduce 100ms delay for easier visual analysis of the results
		//Sleep(100);
	}

	//Release all resources
	PxRelease();

	return 0;
}

PxRigidDynamic* CreateBox()
{
	PxRigidDynamic* box;




	return box;
}