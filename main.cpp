#include "Ogre\ExampleApplication.h"
#include "map.h"
#include "OIS/OIS.h"


class GameLogicClass
{
    private:
         bool allow_to_move;
         float time_to_allow_move;
         Camera * mCamera;
    public:
        GameLogicClass()
        {
            allow_to_move = true;
        }

        void set_camera(Camera* mCamera)
        {
            this->mCamera = mCamera;
        }

        bool allow_move()
        {
            return allow_to_move;
        }
        void freeze(float time)
        {
            time_to_allow_move = time;
            allow_to_move = false;
            CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(), "PortalCompositor", true);
        }
        void new_frame(float time)
        {
            if(!allow_to_move)
            {
                time_to_allow_move -= time;
                if(time_to_allow_move<0)
                {
                    allow_to_move = true;
                    CompositorManager::getSingleton().setCompositorEnabled(mCamera->getViewport(), "PortalCompositor", false);
                }
            }
        }

};
GameLogicClass GameLogic;




class MazeFrameListener;
class Example1;


class Example1 : public ExampleApplication
{
    public:
        Map *map;
        MazeFrameListener* frame_listener;
        Light* light;
        SceneNode* nodeLight;

        Example1();
        ~Example1();
        void createFrameListener();
        void createCamera();
        void setLightPosition(Vector3 pos,double center);
        void createScene();
};

class MazeFrameListener : public Ogre::FrameListener
{
    public:

    Vector3 mTranslateVector;
    Radian mRotX, mRotY, up, down;
    Example1* main;

    MazeFrameListener(Ogre::Camera *mCamera,Ogre::RenderWindow *win, Map* map,Example1* main);
    bool frameRenderingQueued(const FrameEvent& evt);

    Ogre::RenderWindow *win;
    Ogre::Camera *mCamera;
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	Map* map;
};



    MazeFrameListener::MazeFrameListener(Ogre::Camera *mCamera,Ogre::RenderWindow *win, Map* map,Example1* main)
    {
        this->mCamera = mCamera;
        this->map = map;
        this->main = main;

        Vector3 in = Vector3(map->portal_in.x, 0, map->portal_in.y);
        in = map->to_global_space(in);
        in.y = 150;
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


    bool MazeFrameListener::frameRenderingQueued(const FrameEvent& evt)
	{

        mKeyboard->capture();
        mMouse->capture();

        GameLogic.new_frame(evt.timeSinceLastFrame);

        if(GameLogic.allow_move())
	    {

            mTranslateVector = Vector3::ZERO;
            double moveScale = 5000*evt.timeSinceLastFrame;

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



                Vector3 pos = mCamera->getPosition();
                Vector2 pos2d = Vector2(pos.x,pos.z);

                Portal* portal = map->find_portal(pos2d);
                if(portal)
                {
                    GameLogic.freeze(1.0);
                    Vector3 in = Vector3(portal->center.x, 0, portal->center.y);
                    in.z+=5;
                    in = map->to_global_space(in);
                    in.y = 150;
                    Vector3 in_dir = in;
                    switch(portal->rotation)
                    {
                        case Portal::down:mRotX = 0;break;
                        case Portal::up:mRotX = M_PI;break;
                        case Portal::left:mRotX = -M_PI_2;break;
                        case Portal::right:mRotX = M_PI_2;break;
                    }
                    mCamera->setPosition(in);
                    mCamera->setDirection(0,0,-1);

                    mRotY = 0.0;
                    mCamera->yaw(mRotX);
                }
            }


            mCamera->pitch(mRotY);
        }



        main->setLightPosition(mCamera->getPosition(),map->center_y);


		map->set_time(evt.timeSinceLastFrame);
		return true;
	}



        Example1::Example1()
        {
            frame_listener = 0;
            map = 0;

        }
        Example1::~Example1()
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
        void Example1::createFrameListener()
        {
            frame_listener = new MazeFrameListener(mCamera,mWindow,map,this);
            mRoot->addFrameListener(frame_listener);
        }
        void Example1::createCamera()
        {
            mCamera = mSceneMgr->createCamera("MyCamera1");
            GameLogic.set_camera(mCamera);
            //mCamera->setPolygonMode(PM_WIREFRAME);
        }

        void Example1::setLightPosition(Vector3 pos,double center)
        {
            light->setPosition(pos);
            pos.y = center;
            nodeLight->setPosition(pos);
        }

        void Example1::createScene()
        {


            map = new Map("../../../maps/level01.txt",mSceneMgr);

            map->build_geometry();



            light = mSceneMgr->createLight("Light1");
            light->setType(Ogre::Light::LT_POINT);
            light->setDiffuseColour(Ogre::ColourValue(0.6f,0.6f,0.3f));
            //light->setSpecularColour(Ogre::ColourValue(1.0f,1.0f,1.0f));

            nodeLight = mSceneMgr->createSceneNode("LightNode1");
            mSceneMgr->getRootSceneNode()->addChild(nodeLight);
            ParticleSystem* partSystem = mSceneMgr->createParticleSystem("Smoke","Particles/Dust");
            nodeLight->attachObject(partSystem);


            CompositorManager::getSingleton().addCompositor(mCamera->getViewport(), "PortalCompositor");


            /*Entity* LightEnt = mSceneMgr->createEntity("MyEntity","sphere.mesh");
            LightEnt->setMaterialName("Main/Light");
            nodeLight->attachObject(LightEnt);*/
        }

int main()
{
    Example1 app;
    app.go();

    return 0;
}

