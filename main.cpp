#include "Ogre\ExampleApplication.h"
#include "map.h"
#include "OIS/OIS.h"



class MazeFrameListener : public Ogre::FrameListener
{
    public:

    Vector3 mTranslateVector;
    Radian mRotX, mRotY, up, down;

    MazeFrameListener(Ogre::Camera *mCamera,Ogre::RenderWindow *win, Map* map)
    {
        this->mCamera = mCamera;
        this->map = map;

        Vector3 in = Vector3(map->portal_in.x, 0, map->portal_in.y);
        in = map->to_global_space(in);
        in.y = 550;
        Vector3 in_dir = in;
        in_dir.z -= 10;

        mCamera->setPosition(in_dir);
        mCamera->lookAt(in);



        this->win = win;
        up = Degree(55);
        down = Degree(-55);


        OIS::ParamList pl;
		size_t windowHnd = 0;
		std::ostringstream windowHndStr;

		win->getCustomAttribute("WINDOW", &windowHnd);
		windowHndStr << windowHnd;
		pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));

		mInputManager = OIS::InputManager::createInputSystem( pl );

		mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject( OIS::OISKeyboard, false ));
		mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject( OIS::OISMouse, false ));
    }


    bool frameRenderingQueued(const FrameEvent& evt)
	{
	    mKeyboard->capture();
	    mMouse->capture();

	    mTranslateVector = Vector3::ZERO;
		double moveScale = 4000*evt.timeSinceLastFrame;

		if(moveScale>map->max_allowed_step)
		{
            moveScale = map->max_allowed_step;
		}

		bool moved = false;

		if(mKeyboard->isKeyDown(OIS::KC_A))
		{
		    moved = true;
			mTranslateVector.x = -moveScale;
		}

		if(mKeyboard->isKeyDown(OIS::KC_D))
		{
			moved = true;
			mTranslateVector.x = moveScale;
		}

		if(mKeyboard->isKeyDown(OIS::KC_W) )
		{
            moved = true;
			mTranslateVector.z = -moveScale;
		}

		if(mKeyboard->isKeyDown(OIS::KC_S) )
		{
            moved = true;
			mTranslateVector.z = moveScale;
		}
        const OIS::MouseState &ms = mMouse->getMouseState();


        mCamera->setDirection(0,0,-1);
        mRotX += Degree(-ms.X.rel * 0.13);
		mRotY += Degree(-ms.Y.rel * 0.13);
        /*mRotX = Degree(-ms.X.rel * 0.13);
		mRotY = Degree(-ms.Y.rel * 0.13);*/

		if(mRotY<down)
            mRotY = down;
        if(mRotY>up)
            mRotY = up;
        mCamera->yaw(mRotX);

        if(moved)
        {
            Vector3 before = mCamera->getPosition();
            mCamera->moveRelative(mTranslateVector);
            Vector3 after = mCamera->getPosition();
            if(map->correct_position(before,after))
            {
                mCamera->setPosition(after);
            }
        }

		mCamera->pitch(mRotY);
		return true;
	}

    Ogre::RenderWindow *win;
    Ogre::Camera *mCamera;
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	Map* map;
};

class Example1 : public ExampleApplication
{
    public:
        Map *map;
        MazeFrameListener* frame_listener;

        Example1()
        {
            frame_listener = 0;
            map = 0;
        }
        ~Example1()
        {
            if(frame_listener)
            {
                delete frame_listener;
                frame_listener = 0;
            }
            if(map)
            {
                delete map;
                map = 0;
            }
        }
        void createFrameListener()
        {
            frame_listener = new MazeFrameListener(mCamera,mWindow,map);
            mRoot->addFrameListener(frame_listener);
        }
        void createCamera()
        {
            mCamera = mSceneMgr->createCamera("MyCamera1");

            //mCamera->setPolygonMode(PM_WIREFRAME);
        }
        void createScene()
        {


            map = new Map("../../../maps/level01.txt",mSceneMgr);

            map->build_geometry();



                        /*Ogre::Light* light = mSceneMgr->createLight("Light1");
            light->setType(Ogre::Light::LT_POINT);
            light->setDiffuseColour(Ogre::ColourValue(1.0f,1.0f,1.0f));
            light->setPosition(Ogre::Vector3(5,3,10)*map->scale);*/

           /* Ogre::SceneNode* node = mSceneMgr->createSceneNode("Node1");
            mSceneMgr->getRootSceneNode()->addChild(node);

            Ogre::Entity* LightEnt = mSceneMgr->createEntity("MyEntity","sphere.mesh");
            Ogre::SceneNode* node3 = node->createChildSceneNode("node3");
            node3->setScale(0.1f,0.1f,0.1f);
            node3->setPosition(Ogre::Vector3(5,10,10)*map->scale);
            node3->attachObject(LightEnt);*/
        }
};

int main()
{
    Example1 app;
    app.go();

    return 0;
}

