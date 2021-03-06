

#include "VisualDebugger.h"
#include <vector>
#include "Extras\Camera.h"
#include "Extras\Renderer.h"
#include "Extras\HUD.h"

namespace VisualDebugger
{
	using namespace physx;

	enum RenderMode
	{
		DEBUG,
		NORMAL,
		BOTH
	};

	enum HUDState
	{
		EMPTY = 0,
		HELP = 1,
		PAUSE = 2,
		PROFILE = 3
	};

	//function declarations
	void KeyHold();
	void KeySpecial(int key, int x, int y);
	void KeyRelease(unsigned char key, int x, int y);
	void KeyPress(unsigned char key, int x, int y);

	void motionCallback(int x, int y);
	void mouseCallback(int button, int state, int x, int y);
	void exitCallback(void);

	void RenderScene();
	void ToggleRenderMode();
	void HUDInit();

	///simulation objects
	Camera* camera;
	PhysicsEngine::MyScene* scene;
	PxReal delta_time = 1.f / 60.f;
	PxReal gForceStrength = 500.0f;
	PxReal gForceStrengthBall = 10.0f;
	RenderMode render_mode = NORMAL;
	const int MAX_KEYS = 256;
	bool key_state[MAX_KEYS];
	bool hud_show = true;
	HUD hud;
	SZ_ChronoTimer timerFPS, timerRender, timerSceneUpdate;

	//Init the debugger
	void Init(const char *window_name, int width, int height)
	{
		///Init PhysX
		PhysicsEngine::PxInit();
		scene = new PhysicsEngine::MyScene();
		scene->Init();

		///Init renderer
		Renderer::BackgroundColor(PxVec3(150.f / 255.f, 150.f / 255.f, 150.f / 255.f));
		Renderer::SetRenderDetail(40);
		Renderer::InitWindow(window_name, width, height);
		Renderer::Init();

		camera = new Camera(PxVec3(0.0f, 40.0f, 50.0f), PxVec3(0.f, -0.4f, -1.0f), 40.0f);
		//camera = new Camera(PxVec3(0.0f, 5.0f, 15.0f), PxVec3(0.f, 0.0f, -1.0f), 5.f);

		//initialise HUD
		HUDInit();

		///Assign callbacks
		//render
		glutDisplayFunc(RenderScene);

		//keyboard
		glutKeyboardFunc(KeyPress);
		glutSpecialFunc(KeySpecial);
		glutKeyboardUpFunc(KeyRelease);

		//mouse
		glutMouseFunc(mouseCallback);
		glutMotionFunc(motionCallback);

		//exit
		atexit(exitCallback);

		//init motion callback
		motionCallback(0, 0);
	}

	void HUDInit()
	{
		//initialise HUD
		//add an empty screen
		hud.AddLine(EMPTY, "");
		//add a help screen
		
		hud.AddLine(HELP, " Simulation");
		hud.AddLine(HELP, "    F9 - select next actor");
		hud.AddLine(HELP, "    F10 - pause");
		hud.AddLine(HELP, "    F12 - reset");
		hud.AddLine(HELP, "");
		hud.AddLine(HELP, " Display");
		hud.AddLine(HELP, "    F5 - Display Performance");
		hud.AddLine(HELP, "    F6 - shadows on/off");
		hud.AddLine(HELP, "    F7 - render mode");
		hud.AddLine(HELP, "");
		hud.AddLine(HELP, " Camera");
		hud.AddLine(HELP, "    W,S,A,D,Q,Z - forward,backward,left,right,up,down");
		hud.AddLine(HELP, "    mouse + click - change orientation");
		hud.AddLine(HELP, "    F8 - reset view");
		hud.AddLine(HELP, "");
		hud.AddLine(HELP, " Force (applied to the selected actor)");
		hud.AddLine(HELP, "    I,K,J,L,U,M - forward,backward,left,right,up,down");
		hud.AddLine(HELP, "");

		//add a pause screen
		hud.AddLine(PAUSE, "");
		hud.AddLine(PAUSE, "");
		hud.AddLine(PAUSE, "");
		hud.AddLine(PAUSE, "   Simulation paused. Press F10 to continue.");
		hud.AddLine(PROFILE, " Medievil Rugby Game Performance");
		//set font size for all screens
		hud.FontSize(0.018f);
		//set font color for all screens
		hud.Color(PxVec3(0.f, 0.f, 0.f));
	}

	//Start the main loop
	void Start()
	{
		glutMainLoop();
	}

	float frame = 0.0f;
	float avgFPS = 0.0f;
	float sceneUpdateTime = 0.0f;
	string stringFPS = "    Average FPS: ";
	string avgRenderUpdate = "    Average Render Update Time: ";
	string avgSceneUpdate =  "    Average Scene Update Time: ";
	string dynamicActorCount = "    # of Dynamic Actors: ";
	string sceneClothCount = "    # of Cloth: ";

	// Clears and Rewrites the FPS
	void RefreshProfileHUD()
	{
		hud.Clear(PROFILE);
		hud.AddLine(PROFILE, " Medievil Rugby Game Performance");

		if(scene->testCase)
			hud.AddLine(PROFILE, "    Test Case: Enabled");
		else
			hud.AddLine(PROFILE, "    Test Case: Disabled");

		hud.AddLine(PROFILE, "");
		hud.AddLine(PROFILE, stringFPS);
		hud.AddLine(PROFILE, avgRenderUpdate);
		hud.AddLine(PROFILE, avgSceneUpdate);
		hud.AddLine(PROFILE, "");

		hud.AddLine(PROFILE, " Objects Stats:");
		dynamicActorCount = "    # of Dynamic Actors: " + std::to_string(PhysicsEngine::DynamicActor::dynamicCount);
		hud.AddLine(PROFILE, dynamicActorCount);
		sceneClothCount = "    # of Cloth: " + std::to_string(scene->clothCount);
		hud.AddLine(PROFILE, sceneClothCount);
		hud.AddLine(PROFILE, "");

		hud.AddLine(PROFILE, " Player Controls:");
		hud.AddLine(PROFILE, "    Mouse + Hold: Rotate Camera");
		hud.AddLine(PROFILE, "    WASD : Move");
		hud.AddLine(PROFILE, "    F : Trebuchet Throw");
		hud.AddLine(PROFILE, "");
		hud.AddLine(PROFILE, " Debug Controls:");
		hud.AddLine(PROFILE, "    T : Toggle Test Case on Goal Score");
		hud.AddLine(PROFILE, "    P : Move Player to Trebuchet");
		hud.AddLine(PROFILE, "    1 : Spawn 1 Ball");
		hud.AddLine(PROFILE, "    2 : Spawn 100 Balls");
		hud.AddLine(PROFILE, "    3 : Spawn 100 Balls in formation");
		hud.AddLine(PROFILE, "    4 : Spawn 200 Balls");
		hud.AddLine(PROFILE, "    5 : Spawn 1 Cloth (Low Vertice)");
		hud.AddLine(PROFILE, "    6 : Spawn 20 Crowded Enemies");
	}

	// Calculates Average FPS + Render Loop Time
	void UpdatePerformanceStats()
	{
		frame++;
		if (timerFPS.getChronoTime() >= 1000.0f)
		{
			avgFPS = ((float)frame / timerFPS.getChronoTime()) * 1000;
			frame = 0;
			
			stringFPS = "    Average FPS: " + std::to_string(avgFPS);
			timerFPS.resetChronoTimer();
		}

		avgRenderUpdate = "    Average Render Time: " + std::to_string(timerRender.getChronoTime()) + " [ms]";
		timerRender.resetChronoTimer();

		avgSceneUpdate = "    Average Scene Update Time: " + std::to_string(sceneUpdateTime) + " [ms]";
	}

	//Render the scene and perform a single simulation step
	void RenderScene()
	{
	
		// handle pressed keys
		KeyHold();

		// start rendering
		Renderer::Start(camera->getEye(), camera->getDir());

		if ((render_mode == DEBUG) || (render_mode == BOTH))
		{
			Renderer::Render(scene->Get()->getRenderBuffer());
		}

		if ((render_mode == NORMAL) || (render_mode == BOTH))
		{
			std::vector<PxActor*> actors = scene->GetAllActors();
			if (actors.size())
				Renderer::Render(&actors[0], (PxU32)actors.size());
		}

		// adjust the HUD state
		if (hud_show)
		{
			if (scene->Pause())
				hud.ActiveScreen(PAUSE);
			else
				hud.ActiveScreen(HELP);
		}
		else
			hud.ActiveScreen(PROFILE);

		// render HUD
		hud.Render();

		// finish rendering
		Renderer::Finish();

		timerSceneUpdate.resetChronoTimer();
		// perform a single simulation step
		scene->Update(delta_time);
		sceneUpdateTime = timerSceneUpdate.getChronoTime();

		// Work out AVG FPS
		UpdatePerformanceStats();
		// Clear and Rewrite the HUD
		RefreshProfileHUD();

		camera->SetFollowTarget(((PxRigidBody*)scene->player->Get()));

		camera->FollowUpdate(delta_time);
	}

	// On Key Press
	void UserKeyPress(int key)
	{
		// Player Movement
		if (scene->playerController == scene->playerControls && !scene->player->done)
		{
			switch (toupper(key))
			{
			case 'W':
				scene->player->forward = true;
				break;
			case 'S':
				scene->player->back = true;
				break;
			case 'A':
				scene->player->left = true;
				break;
			case 'D':
				scene->player->right = true;
				break;

			case 'R':
				scene->ExampleKeyPressHandler();
				break;
			case 'X':
				scene->pressed = !scene->pressed;
				break;
			default:
				break;
			}
		}

		// Trebuchet Controls
		else if (scene->playerController == scene->trebuchetControls)
		{
			switch (toupper(key))
			{
			case 'F':
				((PxRigidBody*)scene->ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
				scene->trebuchetBase->Kick();
				scene->trebuchetJoint->DriveVelocity(-4.75);
				scene->kickTime = true;

				break;

			default:
				break;
			}
		}

		switch (toupper(key))
		{
		case '1':
			scene->SpawnBalls(1, false);
			break;
		case '2':
			scene->SpawnBalls(100, false);
			break;
		case '3':
			scene->SpawnBalls(100, true);
			break;
		case '4':
			scene->SpawnBalls(200, false);
			break;
		case '5':
			scene->SpawnCloth(1);
			break;
		case 'P':
			scene->MovePlayerToTrebuchet();
			break;
		case 'T':
			scene->testCase = !scene->testCase;
			break;
		}
	}

	// On Key Release
	void UserKeyRelease(int key)
	{
		// Player Controls
		if (scene->playerController == scene->playerControls)
		{
			switch (toupper(key))
			{
				//implement your own
			case 'W':
				scene->player->forward = false;
				break;
			case 'S':
				scene->player->back = false;
				break;
			case 'A':
				scene->player->left = false;
				break;
			case 'D':
				scene->player->right = false;
				break;

			case 'R':
				scene->ExampleKeyReleaseHandler();
				break;
			default:
				break;
			}
		}

		// Trebuchet Controls
		else if (scene->playerController == scene->trebuchetControls)
		{
			switch (toupper(key))
			{
			case 'A':
				scene->trebuchetBase->left = false;
				break;
			case 'D':
				scene->trebuchetBase->right = false;
				break;

			case 'R':
				scene->ExampleKeyReleaseHandler();
				break;
			default:
				break;
			}
		}
	}

	// Force Keys
	void ForceInput(int key)
	{
		if (!scene->GetSelectedActor())
			return;

		switch (toupper(key))
		{
			// Force controls on the selected actor
		case 'I': //forward

			if (scene->GetSelectedActor()->getName() == "Ball")
			{
				scene->GetSelectedActor()->addForce(PxVec3(0, 0, -1) * gForceStrengthBall, PxForceMode::eIMPULSE);
			}
			else
			{
				scene->GetSelectedActor()->addForce(PxVec3(0, 0, -1)*gForceStrength);
			}

			break;
		case 'K': //backward
			scene->GetSelectedActor()->addForce(PxVec3(0, 0, 1)*gForceStrength);
			break;
		case 'J': //left
			scene->GetSelectedActor()->addForce(PxVec3(-1, 0, 0)*gForceStrength);
			break;
		case 'L': //right
			scene->GetSelectedActor()->addForce(PxVec3(1, 0, 0)*gForceStrength);
			break;
		case 'U': //up
			scene->GetSelectedActor()->addForce(PxVec3(0, 1, -1)*gForceStrengthBall, PxForceMode::eIMPULSE);
			break;
		case 'M': //down
			scene->GetSelectedActor()->addForce(PxVec3(0, -1, 0)*gForceStrength);
			break;

		default:
			break;
		}
	}

	void UserKeyHold(int key)
	{
	}

	// Camera Controls
	void CameraInput(int key)
	{
		switch (toupper(key))
		{
		case '8':
			camera->MoveForward(delta_time);
			break;
		case '5':
			camera->MoveBackward(delta_time);
			break;
		case '4':
			camera->MoveLeft(delta_time);
			break;
		case '6':
			camera->MoveRight(delta_time);
			scene->spawnEnemies = true;
			break;
		case 'Q':
			camera->MoveUp(delta_time);
			break;
		case 'Z':
			camera->MoveDown(delta_time);
			break;
		default:
			break;
		}
	}


	///handle special keys
	void KeySpecial(int key, int x, int y)
	{
		//simulation control
		switch (key)
		{
			//display control
		case GLUT_KEY_F5:
			//hud on/off
			hud_show = !hud_show;
			break;
		case GLUT_KEY_F6:
			//shadows on/off
			Renderer::ShowShadows(!Renderer::ShowShadows());
			break;
		case GLUT_KEY_F7:
			//toggle render mode
			ToggleRenderMode();
			break;
		case GLUT_KEY_F8:
			//reset camera view
			camera->Reset();
			break;
			//simulation control
		case GLUT_KEY_F9:
			//select next actor
			scene->SelectNextActor();
			camera->SetFollowTarget(scene->GetSelectedActor());
			break;
		case GLUT_KEY_F10:
			//toggle scene pause
			scene->Pause(!scene->Pause());
			break;
		case GLUT_KEY_F12:
			//resect scene
			scene->Reset();
			break;
		default:
			break;
		}
	}

	//handle single key presses
	void KeyPress(unsigned char key, int x, int y)
	{
		//do it only once
		if (key_state[key] == true)
			return;

		key_state[key] = true;

		//exit
		if (key == 27)
			exit(0);

		UserKeyPress(key);
	}

	//handle key release
	void KeyRelease(unsigned char key, int x, int y)
	{
		key_state[key] = false;
		UserKeyRelease(key);
	}

	//handle holded keys
	void KeyHold()
	{
		for (int i = 0; i < MAX_KEYS; i++)
		{
			if (key_state[i]) // if key down
			{
				CameraInput(i);
				ForceInput(i);
				UserKeyHold(i);
			}
		}
	}

	///mouse handling
	int mMouseX = 0;
	int mMouseY = 0;

	void motionCallback(int x, int y)
	{
		int dx = mMouseX - x;
		int dy = mMouseY - y;

		camera->Motion(dx, dy, delta_time);

		mMouseX = x;
		mMouseY = y;
	}

	void mouseCallback(int button, int state, int x, int y)
	{
		mMouseX = x;
		mMouseY = y;
	}

	void ToggleRenderMode()
	{
		if (render_mode == NORMAL)
			render_mode = DEBUG;
		else if (render_mode == DEBUG)
			render_mode = BOTH;
		else if (render_mode == BOTH)
			render_mode = NORMAL;
	}

	///exit callback
	void exitCallback(void)
	{
		delete camera;
		delete scene;
		PhysicsEngine::PxRelease();
	}
}

