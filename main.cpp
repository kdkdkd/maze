#include "Ogre\Ogre.h"
#include "map.h"
#include "OIS/OIS.h"


class GameLogicClass :public CompositorInstance::Listener
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
        void freeze()
        {
            time_to_allow_move = 1;
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

        void notifyMaterialSetup (uint32 pass_id, MaterialPtr &mat)
        {
            mat->getBestTechnique()->getPass(pass_id)->getFragmentProgramParameters()->setNamedConstant("multiply",Real(1.0/exp(10.0)));

        }
        void notifyMaterialRender(uint32 pass_id, MaterialPtr &mat)
        {
            mat->getBestTechnique()->getPass(pass_id)->getFragmentProgramParameters()->setNamedConstant("multiply",Real(1.0/exp(10.0 * time_to_allow_move)));
        }

};
GameLogicClass GameLogic;




class MazeFrameListener;
class Game;


class Game
{
    public:
        Map *map;
        MazeFrameListener* frame_listener;
        Light* light;
        SceneNode* nodeLight;
        SceneManager* mSceneMgr;
        Camera * mCamera;
        Viewport * mViewport;
        RenderWindow * mWindow;
        Root * mRoot;
        RaySceneQuery * mRayCast;

        Game();
        ~Game();
        bool createConfig();
        void createWindow();
        void createSceneManager();
        void loadRes();
        void createFrameListener();
        void createCamera();
        void createViewport();
        void createRayQuery();
        void castViewRay();
        void setLightPosition(Vector3 pos,double center);
        void createScene();
};

class MazeFrameListener : public FrameListener, public OIS::MouseListener
{
    public:

    Vector3 mTranslateVector;
    Radian mRotX, mRotY, up, down;
    Game* main;

    MazeFrameListener(Ogre::Camera *mCamera,Ogre::RenderWindow *win, Map* map,Game* main);
    bool frameRenderingQueued(const FrameEvent& evt);

    RenderWindow *win;
    Camera *mCamera;
	OIS::InputManager* mInputManager;
	OIS::Mouse*    mMouse;
	OIS::Keyboard* mKeyboard;
	Map* map;
	bool mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
	bool mouseMoved( const OIS::MouseEvent &arg );
    bool mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id );
};
    bool MazeFrameListener::mouseMoved( const OIS::MouseEvent &arg )
    {
        return false;
    }

	bool MazeFrameListener::mouseReleased( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
	{
        return false;
	}


    bool MazeFrameListener::mousePressed( const OIS::MouseEvent &arg, OIS::MouseButtonID id )
    {
        cout<<"BUTTON"<<endl;
        main->castViewRay();
        return false;
    }

    MazeFrameListener::MazeFrameListener(Camera *mCamera,RenderWindow *win, Map* map,Game* main)
    {
        this->mCamera = mCamera;
        this->map = map;
        this->main = main;

        Vector3 in = Vector3(map->portal_in.x, 0, map->portal_in.y);
        in = map->to_global_space(in);
        in.y = 1;
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
        mMouse->setBuffered(true);
        mMouse->setEventCallback(this);
    }


    bool MazeFrameListener::frameRenderingQueued(const FrameEvent& evt)
	{

        mKeyboard->capture();
        mMouse->capture();
        mMouse->setEventCallback(this);
        GameLogic.new_frame(evt.timeSinceLastFrame);

        if(GameLogic.allow_move())
	    {

            mTranslateVector = Vector3::ZERO;
            double moveScale = 25*evt.timeSinceLastFrame;

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
                //cout<<after<<endl;
                //cout<<main->mWindow->getStatistics().triangleCount<<endl;
                if(map->correct_position(before,after))
                {
                    mCamera->setPosition(after);
                }



                Vector3 pos = mCamera->getPosition();
                Vector2 pos2d = Vector2(pos.x,pos.z);

                Portal* portal = map->find_portal(pos2d);
                if(portal)
                {
                    GameLogic.freeze();
                    Vector3 in = Vector3(portal->center.x, 0, portal->center.y);
                    in.z+=5;
                    in = map->to_global_space(in);
                    in.y = 1;
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



        Game::Game()
        {
            frame_listener = 0;
            map = 0;

        }
        Game::~Game()
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
        void Game::createFrameListener()
        {
            frame_listener = new MazeFrameListener(mCamera,mWindow,map,this);
            mRoot->addFrameListener(frame_listener);
        }
        void Game::createCamera()
        {
            mCamera = mSceneMgr->createCamera("MyCamera1");
            mCamera->setNearClipDistance(Real(1));
            GameLogic.set_camera(mCamera);
            //mCamera->setPolygonMode(PM_WIREFRAME);
        }
        void Game::createViewport()
        {
            mViewport = mWindow->addViewport(mCamera);
            mViewport->setBackgroundColour(ColourValue(0.0,0.0,0.0));
            mCamera->setAspectRatio(Real(mViewport->getActualWidth())/Real(mViewport->getActualHeight()));
        }
        void Game::setLightPosition(Vector3 pos,double center)
        {
            light->setPosition(pos);
            pos.y = center;
            nodeLight->setPosition(pos);
        }
        void Game::createScene()
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
            CompositorManager::getSingleton().getCompositorChain(mCamera->getViewport())->getCompositor("PortalCompositor")->addListener(&GameLogic);

            /*Entity* LightEnt = mSceneMgr->createEntity("MyEntity","sphere.mesh");
            LightEnt->setMaterialName("Main/Light");
            nodeLight->attachObject(LightEnt);*/
        }
        bool Game::createConfig()
        {
            mRoot = new Root("plugins.cfg");
            if(!mRoot->restoreConfig())
            {
                if(mRoot->showConfigDialog())
                    mRoot->saveConfig();
                return false;
            }
            return true;
        }
        void  Game::createWindow()
        {
            mWindow = mRoot->initialise(true,"maze");
        }
        void  Game::createSceneManager()
        {
            mSceneMgr = mRoot->createSceneManager(ST_INTERIOR);
        }
        void  Game::loadRes()
        {
            ResourceGroupManager::getSingleton().addResourceLocation("../../../media","FileSystem");
            ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

        }
        void  Game::createRayQuery()
        {
            mRayCast = mSceneMgr->createRayQuery(Ray());
            //mRayCast->setSortByDistance(true);
        }
        void  Game::castViewRay()
        {
            Vector3 normal=mCamera->getDirection();
            normal.normalise();
            Ray ray(mCamera->getPosition(),normal);
            mRayCast->setRay(ray);


            RaySceneQueryResult query_result = mRayCast->execute();
            for (size_t qindex = 0; qindex  < query_result.size(); qindex ++)
            {
                //cout<<query_result[qindex].distance<<endl;
                if ((query_result[qindex].movable != NULL) && (query_result[qindex].movable->getMovableType().compare("Entity") == 0))
                {
                    Entity *pentity = static_cast<Ogre::Entity*>(query_result[qindex].movable);
                    cout<<pentity->getName()<<" "<<query_result[qindex].distance<</*pentity->getBoundingBox()<<*/endl;
                }
            }

        }

int main()
{
    Game g;

    if(!g.createConfig())
        return -1;
    g.createWindow();
    g.createSceneManager();

    g.createCamera();
    g.createViewport();
    g.loadRes();
    g.createScene();
    g.createFrameListener();
    g.createRayQuery();
    g.mRoot->startRendering();




    return 0;
}

