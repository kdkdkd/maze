#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED


#include "Ogre\Ogre.h"
#include "xml/pugixml.hpp"
#include "optimization.h"
#include <vector>

using namespace pugi;
using namespace std;
using namespace Ogre;


struct Portal
{
    Vector2 position;
    Vector2 center;
    enum Rotation
    {
        up,
        down,
        left,
        right
    };
    Rotation rotation;
};

struct PortalPair
{
    public:
    Vector3 color;
    Portal portal1;
    Portal portal2;

};

struct NodeDirectons
{
    public:
    Vector2 forw;
    Vector2 back;
    Vector2 left;
    Vector2 right;
    bool is_forw;
    bool is_back;
    bool is_left;
    bool is_right;

    NodeDirectons()
    {
        is_forw = false;
        is_back = false;
        is_left = false;
        is_right = false;
    }
};

struct Quad
{
    public:
        float x0;
        float x1;
        float y0;
        float y1;
        bool strong;
        Quad(float x0,float x1,float y0,float y1);
        Quad();
        void arrange();
        bool is_in_quad(float x,float y);
};

class Block
{
    public:
    Vector2 differ;
    double constant;
    bool is_positive_direction;
    void arrange();
};

bool compare_blocks (Block first,  Block second);

bool compare_quads (Quad first,Quad second);

class Map
{
    private:
        double max_x;
        double max_y;
        double real_max_x;
        double real_max_y;
        /*double real_min_x;
        double real_min_y;*/
        xml_document doc;
        SceneManager *mSceneMgr;
        double scale_inverse;
        std::vector<Quad> quads;
        float blocks_per_point;
        double global_time;


        //CONFIG CONSTANTS
        double text_y;
        GpuProgramParametersSharedPtr params_portal;



        std::vector<Block> blocks_vertical;
        std::vector<Block> blocks_horizontal;
        float volume[9][16][16][16];

        int volume_x;
        int octaves;
        int volume_y;
        int volume_z;

        std::vector<PortalPair> portals;

    public:
        std::vector<Vector3> decals;
        double center_y;
        double height_y;
        Vector3 scale;
        double eps_dist;
        double max_allowed_step;
        Vector2 portal_in;
        Vector2 portal_out;


        Map(const char * filename,SceneManager * mSceneMgr);



        //TRANSFORMATIONS
        double to_real_x(double x);

        double to_real_y(double y);

        Vector2 to_real(Vector2 point);

        Vector3 to_local_space(Vector3 pos);

        Vector3 to_global_space(Vector3 pos);


        //DO NOT ALLOW TO MOVE THROUGH WALLS
        bool correct_vertical_block_position(Vector3& after_local, Block& vertical_block,bool right);

        bool correct_horizontal_block_position(Vector3& after_local, Block& horizontal_block,bool up);

        bool correct_position(Vector3& before,Vector3& after);

        //SIMPLIFY WALL
        void compact_block(std::vector<Block>& block);

        void compact_blocks();

        //GET MAXIMUM MAP HEIGHT AND WIDTH
        void get_dimensions();

        //GET NODE PRODERTIES
        NodeDirectons get_directions(const char* id);

        //GET POINT POSITION
        Vector2 get_point_by_id(const char* id);
        //GET POINT INDEX
        int get_index_by_id(const char* id);

        //GET PORTAL IN
        Vector2 get_portal_in();

        //GET PORTAL OUT
        Vector2 get_portal_out();




        //GET POINT ON EDGE OF RANDOM GENERATED GEOMETRY
        Vector3 get_point(int edge,float density[8]);

        //GET NORMAL ON RANDOM GENERATED GEOMETRY
        Vector3 get_normal(Vector3 point);

        void set_tangent_space(ManualObject*  manual, Vector3 point);

        //GET CUBE OF RANDOM GENERATED GEOMETRY
        void draw_node(ManualObject*  manual,Vector3 start,Vector3 dim, int & index,float density[8]);

        float get_squared_distance_to_void(float i,float j,float k);

        //GET POINT FROM VOLUME OF RANDOM NUMBERS
        float get_volume_point(int volume_index,float i,float j, float k);

        //GET DENSITY IN SOME POINT
        float get_density(float i,float j,float k);

        //BUILD RANDOM GEOMETRY
        void build_random_geometry();

        //RENDER ONLY ONE PORTAL
        void render_portal(ManualObject* manual_portals, int index, Portal portal, Vector3 color);

        //CREATE ENTINITY OF PORTALs
        Entity * render_portals();

        //INSERT PORTAL QUAD ALL WALLS
        Portal add_portal(xml_node portal);


        //FILL VECTOR WITH PORTALS, PATCH BLOKS AND QUADS
        void parse_portals();


        //FIND IF BLOCK CONTAINS ANOTHER ONE AND MAKE HOLE IN IT
        void merge_block(Block b,std::vector<Block>& dest);


        void merge_block_vertical(Block b);


        void merge_block_horizontal(Block b);

        //BUILD REGULAR GEOMETRY
        void build_geometry();

        //FINDS PORTAL, IF NOT FOUND - RETURNS ZERO
        Portal * find_portal(Vector2 position);

        //SET TIME FOR RENDERING
        void set_time(double time);


        //ADD DECAL
        void add_decal_quad(Vector3 a,Vector3 b,Vector3 c,Vector3 d);

        //SEARCH DECAL IN SAME POSITION
        bool search_for_decal(Vector3 decal);


};

#endif // MAP_H_INCLUDED




