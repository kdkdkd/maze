#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED

#include "xml/pugixml.hpp"
#include "optimization.h"
#include <vector>
#include "Ogre/ExampleApplication.h"
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


        Quad(float x0,float x1,float y0,float y1)
        {
            strong = false;
            this->x0 = x0;
            this->x1 = x1;
            this->y0 = y0;
            this->y1 = y1;
        }
        Quad()
        {
            strong = false;

        }

        void arrange()
        {
            if(x0>x1)
            {
                float swap = x0;x0 = x1;x1 = swap;
            }
            if(y0>y1)
            {
                float swap = y0;y0 = y1;y1 = swap;
            }
        }

        bool is_in_quad(float x,float y)
        {
            return x<=x1 && x>=x0 && y>=y0 && y <= y1;
        }

};

class Block
{
    public:
    Vector2 differ;
    double constant;
    bool is_positive_direction;
    void arrange()
    {
        if(differ.x>differ.y)
        {
            double swap = differ.x;differ.x = differ.y;differ.y = swap;
        }
    }
};

bool compare_blocks (Block first,  Block second)
{
    if(fabs(first.constant-second.constant)<0.01)
    {
        return first.differ.x>second.differ.x;
    }

    return first.constant>second.constant;
}

bool compare_quads (Quad first,Quad second)
{
    return first.x0>second.x0;
}

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
        double center_y;
        double height_y;
        Vector3 scale;
        double eps_dist;
        double max_allowed_step;
        Vector2 portal_in;
        Vector2 portal_out;


        Map(const char * filename,SceneManager * mSceneMgr)
        {
            scale = Vector3(200,200,200);
            volume_x = 16;
            volume_y = 16;
            volume_z = 16;
            octaves = 9;
            doc.load_file(filename);
            portal_in = to_real(get_portal_in());
            portal_in.y+=5;
            portal_out = to_real(get_portal_out());
            portal_out.y+=5;
            this->mSceneMgr = mSceneMgr;
            get_dimensions();
            text_y = 2.0;
            height_y = 10.0;
            eps_dist = 1.0;
            max_allowed_step = eps_dist * 0.5 * scale.x;
            blocks_per_point = 0.3;
            center_y = (height_y + 2) * scale.y * 0.5;
            global_time = 0.0;
        }



        //TRANSFORMATIONS
        double to_real_x(double x)
        {
            return (x - 1.0)*42.0;
        }

        double to_real_y(double y)
        {
            return -(y)*42.0;
        }

        Vector2 to_real(Vector2 point)
        {
            return Vector2(to_real_x(point.x),to_real_y(point.y));
        }

        Vector3 to_local_space(Vector3 pos)
        {
            return pos/scale;
        }

        Vector3 to_global_space(Vector3 pos)
        {
            return pos*scale;
        }





        //DO NOT ALLOW TO MOVE THROUGH WALLS
        bool correct_vertical_block_position(Vector3& after_local, Block& vertical_block,bool right)
        {
            if(after_local.z<vertical_block.differ.y + eps_dist && after_local.z>vertical_block.differ.x - eps_dist )
            {
                bool changed = false;
                if(right && !vertical_block.is_positive_direction)
                {
                    changed = true;
                    after_local.x = vertical_block.constant - eps_dist - 0.001;
                }
                if(!right && vertical_block.is_positive_direction)
                {
                    changed = true;
                    after_local.x = vertical_block.constant + eps_dist + 0.001;
                }
                return changed;
            }
            return false;
        }

        bool correct_horizontal_block_position(Vector3& after_local, Block& horizontal_block,bool up)
        {
            if(after_local.x<horizontal_block.differ.y + eps_dist && after_local.x>horizontal_block.differ.x - eps_dist )
            {
                bool changed = false;
                if(up &&  !horizontal_block.is_positive_direction
                   )
                {
                    changed = true;
                    after_local.z = horizontal_block.constant - eps_dist - 0.001;
                }
                if(!up &&  horizontal_block.is_positive_direction
                   )
                {
                     changed = true;
                     after_local.z = horizontal_block.constant + eps_dist + 0.001;
                }
                return changed;
            }
            return false;
        }

        bool correct_position(Vector3& before,Vector3& after)
        {
            std::vector<Block>::iterator vertical_block;
            std::vector<Block>::iterator horizontal_block;
            bool vertical_up = false;
            bool horizontal_up = false;

            Vector3 after_local = to_local_space(after);
            Vector3 after_local_copy = after_local;
            Vector3 after_local_y;
            Vector3 after_local_x;
            bool res_y = false;
            bool res_x = false;
            bool res = false;



            //CORRECT VERTICAL
            for(std::vector<Block>::iterator it_prev = blocks_vertical.begin();;it_prev++)
            {
                if(it_prev==blocks_vertical.end())
                    break;
                std::vector<Block>::iterator it_next = it_prev + 1;
                if(it_next==blocks_vertical.end())
                    break;
                if(it_prev->constant>=after_local.x && it_next->constant<=after_local.x)
                {
                    std::vector<Block>::iterator it_up = it_prev;
                    std::vector<Block>::iterator it_down = it_next;
                    double const_up = it_up->constant;
                    double const_down = it_down->constant;
                    //CORRECT RIGHT
                    if(const_up - after_local.x<eps_dist)
                    {
                        if(correct_vertical_block_position(after_local,*it_up,true))
                        {
                            res = true;
                            vertical_block = it_up;
                            vertical_up = true;

                        }else if(it_up != blocks_vertical.begin())
                        {
                            it_up--;
                            while(fabs(const_up-it_up->constant)<0.001)
                            {
                                if(correct_vertical_block_position(after_local,*it_up,true))
                                {
                                    res = true;
                                    vertical_block = it_up;
                                    vertical_up = true;
                                    break;
                                }


                                if(it_up == blocks_vertical.begin())
                                    break;
                                it_up--;
                            }
                        }
                    }

                    //CORRECT LEFT
                    else if(after_local.x - const_down<eps_dist)
                    {
                        if(correct_vertical_block_position(after_local,*it_down,false))
                        {
                            res = true;
                            vertical_block = it_down;
                            vertical_up = false;
                        }else
                        {
                            it_down++;
                            if(it_down != blocks_vertical.end())
                            {
                                while(fabs(it_down->constant-const_down)<0.001)
                                {
                                    if(correct_vertical_block_position(after_local,*it_down,false))
                                    {
                                        res = true;
                                        vertical_block = it_down;
                                        vertical_up = false;

                                        break;
                                    }

                                    it_down++;
                                    if(it_down == blocks_vertical.end())
                                        break;
                                }
                            }
                        }


                    }
                }
            }

                res_y = res;
                res = false;
                after_local_y = after_local;
                after_local = after_local_copy;

                //CORRECT HORIZONTAL
                for(std::vector<Block>::iterator it_prev = blocks_horizontal.begin();;it_prev++)
                {
                    if(it_prev==blocks_horizontal.end())
                        break;
                    std::vector<Block>::iterator it_next = it_prev + 1;
                    if(it_next==blocks_horizontal.end())
                        break;
                    if(it_prev->constant>=after_local.z && it_next->constant<=after_local.z)
                    {
                        std::vector<Block>::iterator it_up = it_prev;
                        std::vector<Block>::iterator it_down = it_next;
                        double const_up = it_up->constant;
                        double const_down = it_down->constant;
                        //CORRECT DOWN
                        if(const_up - after_local.z<eps_dist)
                        {

                            if(correct_horizontal_block_position(after_local,*it_up,true))
                            {
                                res = true;
                                horizontal_block = it_up;
                                horizontal_up = true;
                            }else if(it_up != blocks_horizontal.begin())
                            {
                                it_up--;
                                while(fabs(const_up-it_up->constant)<0.001)
                                {
                                    if(correct_horizontal_block_position(after_local,*it_up,true))
                                    {
                                        res = true;
                                        horizontal_block = it_up;
                                        horizontal_up = true;
                                        break;
                                    }


                                    if(it_up == blocks_horizontal.begin())
                                        break;
                                    it_up--;
                                }
                            }
                        }

                        //CORRECT UP
                        else if(after_local.z - const_down<eps_dist)
                        {
                            if(correct_horizontal_block_position(after_local,*it_down,false))
                            {
                                res = true;
                                horizontal_block = it_down;
                                horizontal_up = false;
                            }else
                            {
                                it_down++;
                                if(it_down != blocks_horizontal.end())
                                {
                                    while(fabs(it_down->constant-const_down)<0.001)
                                    {
                                        if(correct_horizontal_block_position(after_local,*it_down,false))
                                        {
                                            res = true;
                                            horizontal_block = it_down;
                                            horizontal_up = false;
                                            break;
                                        }

                                        it_down++;
                                        if(it_down == blocks_horizontal.end())
                                            break;
                                    }
                                }
                            }

                        }

                    }
                }

                res_x = res;
                after_local_x = after_local;

                if(!res_x && !res_y)
                    return false;
                if(!res_x && res_y)
                {
                    after = to_global_space(after_local_y);
                    return true;
                }
                if(res_x && !res_y)
                {
                    after = to_global_space(after_local_x);
                    return true;
                }

                Vector3 before_local = to_local_space(before);

                double dist_x = before_local.squaredDistance(after_local_x);
                double dist_y = before_local.squaredDistance(after_local_y);
                if(dist_x<dist_y)
                {
                        correct_vertical_block_position(after_local_x,*vertical_block,vertical_up);
                        after = to_global_space(after_local_x);

                }else
                {
                        correct_horizontal_block_position(after_local_y,*horizontal_block,horizontal_up);
                        after = to_global_space(after_local_y);
                }

                return true;

            //cout<<after_local.x<<" "<<after_local.z<<" "<<endl;
            /*if(after_local.x<0)
            {
                after.x = 0.0;
                return true;
            }*/
            //return res;
        }





        //SIMPLIFY WALL
        void compact_block(std::vector<Block>& block)
        {
            sort(block.begin(),block.end(),compare_blocks);

            Block prev;
            bool first = true;
            std::vector<Block> block_compact;
            for(std::vector<Block>::iterator it = block.begin();it != block.end();it++)
            {
                if(!first)
                {
                    if(fabs(it->differ.y - prev.differ.x)<0.01 && fabs(it->constant - prev.constant)<0.01)
                    {
                        block_compact.pop_back();
                        prev.differ.x = it->differ.x;
                        prev.arrange();
                        block_compact.push_back(prev);
                    }else
                    {
                        block_compact.push_back(*it);
                        prev = *it;
                    }

                }else
                {
                    block_compact.push_back(*it);
                    prev = *it;
                }
                first = false;
            }


            block.clear();

            block.resize(block_compact.size());
            copy ( block_compact.begin(), block_compact.end(), block.begin() );

        }

        void compact_blocks()
        {
            compact_block(blocks_vertical);
            compact_block(blocks_horizontal);
            sort(quads.begin(),quads.end(),compare_quads);
            /*cout<<"VERTICAL"<<endl;
            for(std::vector<Block>::iterator it=blocks_vertical.begin(); it!=blocks_vertical.end();++it )
            {
                cout<<it->constant<<" "<<it->differ.x<<" - "<<it->differ.y<<endl;
            }
            cout<<"HGoryz"<<endl;
            for(std::vector<Block>::iterator it=blocks_horizontal.begin();it !=blocks_horizontal.end();++it )
            {
                cout<<it->constant<<" "<<it->differ.x<<" - "<<it->differ.y<<endl;
            }

            cout<<"QUADS"<<endl;
            for(std::vector<Quad>::iterator it=quads.begin(); it!=quads.end();++it )
            {
                cout<<it->x0<<" , "<<it->x1<<" - "<<it->y0<<" , "<<it->y1<<endl;
            }*/
        }

        //GET MAXIMUM MAP HEIGHT AND WIDTH
        void get_dimensions()
        {
            max_x = 0.0;
            max_y = 0.0;

            xpath_node_set points = doc.select_nodes("/map/points/point");
            for (xpath_node_set::const_iterator it = points.begin(); it != points.end(); ++it)
            {
                xpath_node point = *it;
                double x = point.node().attribute("x").as_double();
                double y = point.node().attribute("y").as_double();

                if(x>max_x)
                    max_x = x;
                if(y>max_y)
                    max_y = y;
            }
            real_max_x = to_real_x(max_x) + 5.0;
            real_max_y = to_real_y(max_y);
            max_x = 5.0/(real_max_x);
            max_y = 5.0/(real_max_y);
        }

        //GET NODE PRODERTIES
        NodeDirectons get_directions(const char* id)
        {
            Vector2 main = get_point_by_id(id);
            NodeDirectons res;
            xpath_node_set lines = doc.select_nodes("/map/lines/line");
            for (xpath_node_set::const_iterator it = lines.begin(); it != lines.end(); ++it)
            {
                pugi::xpath_node line = *it;
                const char *id_other = 0;
                const char *start = line.node().attribute("start").value();
                const char *end = line.node().attribute("end").value();
                if(strcmp(id,start)==0)
                {
                    id_other = end;
                }else if(strcmp(id,end)==0)
                {
                    id_other = start;
                }
                if(id_other)
                {
                    Vector2 other = get_point_by_id(id_other);
                    if(fabs(main.x - other.x)<0.1)
                    {
                        if(main.y<other.y)
                        {
                            res.forw = other;
                            res.is_forw = true;
                        }else
                        {
                            res.back = other;
                            res.is_back = true;
                        }
                    }else
                    {
                        if(main.x<other.x)
                        {
                            res.right = other;
                            res.is_right = true;
                        }else
                        {
                            res.left = other;
                            res.is_left = true;
                        }
                    }

                }
            }
            return res;

        }

        //GET POINT POSITION
        Vector2 get_point_by_id(const char* id)
        {
            xpath_node_set points = doc.select_nodes("/map/points/point");


            for (xpath_node_set::const_iterator it = points.begin(); it != points.end(); ++it)
            {
                xpath_node point = *it;
                if(strcmp(point.node().attribute("id").value(),id)==0)
                {
                    return Vector2(point.node().attribute("x").as_double(),point.node().attribute("y").as_double());
                }
            }
            return Vector2::ZERO;
        }

        //GET POINT INDEX
        int get_index_by_id(const char* id)
        {
            xpath_node_set points = doc.select_nodes("/map/points/point");
            int index = 0;

            for (xpath_node_set::const_iterator it = points.begin(); it != points.end(); ++it)
            {
                xpath_node point = *it;
                if(strcmp(point.node().attribute("id").value(),id)==0)
                {
                    return index;
                }
                index++;
            }
            return -1;
        }

        //GET PORTAL IN
        Vector2 get_portal_in()
        {
            return get_point_by_id(doc.child("map").child("portalin").attribute("point").value());
        }

        //GET PORTAL OUT
        Vector2 get_portal_out()
        {
            return get_point_by_id(doc.child("map").child("portalout").attribute("point").value());
        }





        //GET POINT ON EDGE OF RANDOM GENERATED GEOMETRY
        Vector3 get_point(int edge,float density[8])
        {
            Vector3 point1;
            Vector3 point2;
            float density1;
            float density2;
            switch(edge)
            {
                case 0:point1 = Vector3(0,0,0);point2 = Vector3(0,1,0);density1 = density[0];density2 = density[1];break;
                case 1:point1 = Vector3(0,1,0);point2 = Vector3(1,1,0);density1 = density[1];density2 = density[2];break;
                case 2:point1 = Vector3(1,1,0);point2 = Vector3(1,0,0);density1 = density[2];density2 = density[3];break;
                case 3:point1 = Vector3(1,0,0);point2 = Vector3(0,0,0);density1 = density[3];density2 = density[0];break;

                case 4:point1 = Vector3(0,0,1);point2 = Vector3(0,1,1);density1 = density[4];density2 = density[5];break;
                case 5:point1 = Vector3(0,1,1);point2 = Vector3(1,1,1);density1 = density[5];density2 = density[6];break;
                case 6:point1 = Vector3(1,1,1);point2 = Vector3(1,0,1);density1 = density[6];density2 = density[7];break;
                case 7:point1 = Vector3(1,0,1);point2 = Vector3(0,0,1);density1 = density[7];density2 = density[4];break;

                case 8:point1 = Vector3(0,0,0);point2 = Vector3(0,0,1);density1 = density[0];density2 = density[4];break;
                case 9:point1 = Vector3(0,1,0);point2 = Vector3(0,1,1);density1 = density[1];density2 = density[5];break;
                case 10:point1 = Vector3(1,1,0);point2 = Vector3(1,1,1);density1 = density[2];density2 = density[6];break;
                case 11:point1 = Vector3(1,0,0);point2 = Vector3(1,0,1);density1 = density[3];density2 = density[7];break;

            }
            float fabs_density1 = fabs(density1);
            float point1_dest_k = fabs_density1/(fabs_density1 + fabs(density2));
            return point1 * (1 - point1_dest_k) + point1_dest_k * point2;
        }

        //GET NORMAL ON RANDOM GENERATED GEOMETRY
        Vector3 get_normal(Vector3 point)
        {
            /*if(point.y<0.01)
                return Vector3::UNIT_Y;*/
            //cout<<"get_normal"<<point.x<<" "<<point.y<<" "<<point.z<<" "<<endl;
            float d = 2;
            Vector3 grad = Vector3(0,0,0);
            //while(fabs(grad.x)<0.001 && fabs(grad.y)<0.001 && fabs(grad.z)<0.001)
            {
                grad.x = get_density(point.x + d,point.y,point.z) - get_density(point.x - d,point.y,point.z);
                grad.y = get_density(point.x,point.y+d,point.z) - get_density(point.x,point.y-d,point.z);
                grad.z = get_density(point.x,point.y,point.z+d) - get_density(point.x,point.y,point.z-d);
                //d *= 2;
            }
            //cout<<get_density(point.x + d,point.y,point.z) - get_density(point.x - d,point.y,point.z)<<"  "<<get_density(point.x,point.y+d,point.z) - get_density(point.x,point.y-d,point.z)<<"   "<<get_density(point.x,point.y,point.z+d) - get_density(point.x,point.y,point.z-d)<<"Normal"<<endl;
            grad.normalise();
            return -grad;
            //return Vector3(0,1,0);
        }

        void set_tangent_space(ManualObject*  manual, Vector3 point)
        {
            float blocks_per_point_reverse = 1.0 / blocks_per_point;
            Vector3 normal = get_normal(point*blocks_per_point_reverse);
            manual->normal(normal);
            /*if(fabs(normal.x)>fabs(normal.y) && fabs(normal.x)>fabs(normal.z))
            {
                manual->tangent(0,1,0);
                manual->textureCoord(0,0,1);
            }else if(fabs(normal.y)>fabs(normal.z) && fabs(normal.y)>fabs(normal.x))
            {
                manual->tangent(1,0,0);
                manual->textureCoord(0,0,1);

            }else
            {
                manual->tangent(1,0,0);
                manual->textureCoord(0,1,0);
            }*/
        }

        //GET CUBE OF RANDOM GENERATED GEOMETRY
        void draw_node(ManualObject*  manual,Vector3 start,Vector3 dim, int & index,float density[8])
        {
            static char num_of_edge[256] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 2, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 3, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 3, 2, 3, 3, 2, 3, 4, 4, 3, 3, 4, 4, 3, 4, 5, 5, 2, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 3, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 4, 2, 3, 3, 4, 3, 4, 2, 3, 3, 4, 4, 5, 4, 5, 3, 2, 3, 4, 4, 3, 4, 5, 3, 2, 4, 5, 5, 4, 5, 2, 4, 1, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 3, 2, 3, 3, 4, 3, 4, 4, 5, 3, 2, 4, 3, 4, 3, 5, 2, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 4, 3, 4, 4, 3, 4, 5, 5, 4, 4, 3, 5, 2, 5, 4, 2, 1, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 2, 3, 3, 2, 3, 4, 4, 5, 4, 5, 5, 2, 4, 3, 5, 4, 3, 2, 4, 1, 3, 4, 4, 5, 4, 5, 3, 4, 4, 5, 5, 2, 3, 4, 2, 1, 2, 3, 3, 2, 3, 4, 2, 1, 3, 2, 4, 1, 2, 1, 1, 0};
            static char edge[256][15] = { {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1},{ 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1},{ 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1},{ 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1},{ 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1},{ 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1},{ 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1},{ 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1},{ 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1},{ 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1},{ 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1},{ 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1},{ 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1},{ 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1},{ 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1},{ 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1},{ 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1},{ 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1},{ 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1},{ 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1},{ 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1},{ 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1},{ 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1},{ 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1},{ 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1},{ 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1},{ 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1},{ 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1},{ 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1},{ 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1},{ 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1},{ 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1},{ 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1},{ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1},{ 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1},{ 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1},{ 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1},{ 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0},{ 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0},{ 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1},{ 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1},{ 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1},{ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1},{ 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1},{ 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1},{ 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1},{ 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1},{ 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1},{ 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1},{ 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1},{ 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1},{ 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1},{ 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1},{ 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1},{ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1},{ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1},{ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9},{ 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1},{ 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1},{ 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1},{ 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6},{ 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1},{ 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11},{ 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7},{ 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1},{ 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1},{ 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1},{ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1},{ 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1},{ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1},{ 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1},{ 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1},{ 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1},{ 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1},{ 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1},{ 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1},{ 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1},{ 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1},{ 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1},{ 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1},{ 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1},{ 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1},{ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1},{ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9},{ 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1},{ 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1},{ 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7},{ 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11},{ 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1},{ 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6},{ 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1},{ 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1},{ 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1},{ 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1},{ 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1},{ 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1},{ 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1},{ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1},{ 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1},{ 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1},{ 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1},{ 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1},{ 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1},{ 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1},{ 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1},{ 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1},{ 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1},{ 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1},{ 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3},{ 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1},{ 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1},{ 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1},{ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1},{ 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1},{ 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3},{ 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1},{ 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1},{ 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1},{ 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1},{ 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1},{ 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1},{ 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6},{ 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1},{ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1},{ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1},{ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8},{ 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1},{ 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4},{ 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10},{ 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1},{ 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1},{ 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1},{ 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1},{ 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1},{ 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1},{ 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10},{ 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5},{ 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1},{ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1},{ 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1},{ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8},{ 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6},{ 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1},{ 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1},{ 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1},{ 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1},{ 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1},{ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1},{ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1},{ 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2},{ 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1},{ 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1},{ 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1},{ 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2},{ 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1},{ 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1},{ 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1},{ 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1},{ 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1},{ 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4},{ 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1},{ 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11},{ 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5},{ 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1},{ 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1},{ 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9},{ 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1},{ 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1},{ 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1},{ 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1},{ 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1},{ 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1},{ 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4},{ 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1},{ 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3},{ 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1},{ 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1},{ 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1},{ 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7},{ 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10},{ 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1},{ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1},{ 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1},{ 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1},{ 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1},{ 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1},{ 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1},{ 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1},{ 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

            //cout<<density[0]<<" "<<density[1]<<" "<<density[2]<<" "<<density[3]<<" "<<density[4]<<" "<<density[5]<<" "<<density[6]<<" "<<density[7]<<endl;
            int edge_index = 0;
            int mult = 1;
            edge_index+=(density[0]>=0.0) * mult;mult<<=1;
            edge_index+=(density[1]>=0.0) * mult;mult<<=1;
            edge_index+=(density[2]>=0.0) * mult;mult<<=1;
            edge_index+=(density[3]>=0.0) * mult;mult<<=1;
            edge_index+=(density[4]>=0.0) * mult;mult<<=1;
            edge_index+=(density[5]>=0.0) * mult;mult<<=1;
            edge_index+=(density[6]>=0.0) * mult;mult<<=1;
            edge_index+=(density[7]>=0.0) * mult;

            int triangles = num_of_edge[edge_index];
            //cout<<triangles<<endl;

            for(int triangle = 0;triangle<triangles;triangle++)
            {
                int edge_1 = edge[edge_index][triangle*3];
                int edge_2 = edge[edge_index][triangle*3 + 1];
                int edge_3 = edge[edge_index][triangle*3 + 2];
                //cout<<edge_1<<" "<<edge_2<<" "<<edge_3<<endl;
                Vector3 point1 = (get_point(edge_1,density) + start);
                Vector3 point2 = (get_point(edge_2,density) + start);
                Vector3 point3 = (get_point(edge_3,density) + start);

                manual->position(point1 * dim);
                set_tangent_space(manual,point1);

                manual->position(point2 * dim);
                set_tangent_space(manual,point2);

                manual->position(point3 * dim);
                set_tangent_space(manual,point3);


                manual->index(index);
                manual->index(index + 1);
                manual->index(index + 2);
                index+=3;
                /*cout<<point1.x<<" "<<point1.y<<" "<<point1.z<<endl;
                cout<<point2.x<<" "<<point2.y<<" "<<point2.z<<endl;
                cout<<point3.x<<" "<<point3.y<<" "<<point3.z<<endl;

                cout<<endl;*/

            }
        }

        float get_squared_distance_to_void(float i,float j,float k)
        {

            /*if(j>height_y)
                res.x = (j-height_y)*(j-height_y);
            else if(j<0)
                res.x = j * j;*/

            bool was_res = false;
            float dist_2d = 0.0;
            float dx = 0.0;
            float dy = 0.0;
            float dz = 0.0;
            bool in = false;

            if(j > height_y)
                dy = j - height_y;
            else if(j < 0)
                dy = -j;






            for(std::vector<Quad>::iterator it_quad = quads.begin();it_quad!=quads.end();it_quad++)
            {
                if(it_quad->is_in_quad(i,k))
                {

                    if(dy>0.001)
                    {
                        dx = 0.0;
                        dz = 0.0;
                        break;
                    }else
                    {
                        if(it_quad->strong)
                            return -100000;

                        in = true;
                        dx = 0.0;
                        dz = 0.0;
                        dist_2d = 0.0;

                        was_res = false;
                        dy = height_y - j;
                        if(j<dy)
                            dy = j;


                        for(std::vector<Block>::iterator it = blocks_vertical.begin();it != blocks_vertical.end();it++)
                        {
                            float dx_candidate = 0.0;
                            float dz_candidate = 0.0;

                            dx_candidate = fabs(it->constant - i);
                            if(k>it->differ.y)
                                dz_candidate = k - it->differ.y;
                            if(k<it->differ.x)
                                dz_candidate = it->differ.x - k;

                            float dist_candidate = dx_candidate * dx_candidate + dz_candidate * dz_candidate;

                            if(!was_res || dist_candidate < dist_2d)
                            {
                                dist_2d = dist_candidate;
                                dz = dz_candidate;
                                dx = dx_candidate;
                                was_res = true;
                            }
                        }


                        for(std::vector<Block>::iterator it = blocks_horizontal.begin();it != blocks_horizontal.end();it++)
                        {
                            float dx_candidate = 0.0;
                            float dz_candidate = 0.0;

                            dz_candidate = fabs(it->constant - k);
                            if(i>it->differ.y)
                                dx_candidate = i - it->differ.y;
                            if(i<it->differ.x)
                                dx_candidate = it->differ.x - i;

                            float dist_candidate = dx_candidate * dx_candidate + dz_candidate * dz_candidate;

                            if(!was_res || dist_candidate < dist_2d)
                            {
                                dist_2d = dist_candidate;
                                dz = dz_candidate;
                                dx = dx_candidate;
                                was_res = true;
                            }
                        }
                        break;
                    }

                }else
                {

                    float dx_candidate = 0.0;
                    float dz_candidate = 0.0;

                    if(i < it_quad->x0)
                    {
                        dx_candidate = it_quad->x0 - i;
                    }else if(i > it_quad->x1)
                    {
                        dx_candidate = i - it_quad->x1;
                    }


                    if(k < it_quad->y0)
                    {
                        dz_candidate = it_quad->y0 - k;
                    }else if(k > it_quad->y1)
                    {
                        dz_candidate = k - it_quad->y1;
                    }

                    float dist_candidate = dx_candidate * dx_candidate + dz_candidate * dz_candidate;

                    if(!was_res || dist_candidate < dist_2d)
                    {
                        dist_2d = dist_candidate;
                        dz = dz_candidate;
                        dx = dx_candidate;
                        was_res = true;
                    }

                }
            }
            float res = dx*dx + dy*dy + dz*dz;
            if(in)
                res *= -1.0;
            return res;
        }

        //GET POINT FROM VOLUME OF RANDOM NUMBERS
        float get_volume_point(int volume_index,float i,float j, float k)
        {
            float gi = fabs(i);
            float gj = fabs(j);
            float gk = fabs(k);
            int x0 = floor(gi);
            int x1 = ceil(gi);
            int y0 = floor(gj);
            int y1 = ceil(gj);
            int z0 = floor(gk);
            int z1 = ceil(gk);
            float xd = gi - x0;
            float yd = gj - y0;
            float zd = gk - z0;

            float i1 = volume[volume_index][x0%volume_x][y0%volume_y][z0%volume_z]*(1-zd) + volume[volume_index][x0%volume_x][y0%volume_y][z1%volume_z]*(zd);
            float i2 = volume[volume_index][x0%volume_x][y1%volume_y][z0%volume_z]*(1-zd) + volume[volume_index][x0%volume_x][y1%volume_y][z1%volume_z]*(zd);
            float j2 = volume[volume_index][x1%volume_x][y1%volume_y][z0%volume_z]*(1-zd) + volume[volume_index][x1%volume_x][y1%volume_y][z1%volume_z]*(zd);
            float j1 = volume[volume_index][x1%volume_x][y0%volume_y][z0%volume_z]*(1-zd) + volume[volume_index][x1%volume_x][y0%volume_y][z1%volume_z]*(zd);

            float w1 = i1*(1-yd) + i2*yd;
            float w2 = j1*(1-yd) + j2*yd;

            return (w1)*(1-xd) + w2*xd;

        }

        //GET DENSITY IN SOME POINT
        float get_density(float i,float j,float k)
        {

            /*float fi=i/20.0;
            float fj=j/20.0;
            float fk=k/20.0;

            fi += get_volume_point(0,fi*0.004,fj*0.004,fk*0.004)*2;
            fj += get_volume_point(1,fi*0.004,fj*0.004,fk*0.004)*2;
            fk += get_volume_point(2,fi*0.004,fj*0.004,fk*0.004)*2;

            float res = (- j + (float)volume_y/2.0)/20.0;
            res += get_volume_point(0,fi,fj,fk);/
            //res += get_volume_point(1,2*fi,2*fj,2*fk) * 0.5;
            //res += get_volume_point(2,4*fi,4*fj,4*fk)*0.5;
            //res += get_volume_point(1,8*fi,8*fj,8*fk)*0.25;
            //res += get_volume_point(1,2*fi,2*fj,2*fk) * 0.5;
            //res += get_volume_point(2,4*fi,4*fj,4*fk) * 0.25;
            //res += get_volume_point(3,8*fi,8*fj,8*fk)*0.125;
            //res += get_volume_point(4,16*fi,16*fj,16*fk)*0.0625;
            //res += get_volume_point(5,32*fi,32*fj,32*fk)*0.03125;
            //res += get_volume_point(6,64*fi,64*fj,64*fk)*0.03125*0.5;
            //res += get_volume_point(7,128*fi,128*fj,128*fk)*0.03125*0.25;
            //res += get_volume_point(8,256*fi,256*fj,256*fk)*0.03125*0.125;
            //res += get_volume_point(4,8*fi,8*fj,8*fk)*0.125;

            //res += get_volume_point(1,2*fi,2*fj,2*fk) * 0.5;
            //res += get_volume_point(2,4*fi,4*fj,4*fk) * 0.25;
            //res += get_volume_point(3,8*fi,8*fj,8*fk)*0.125;
            //res += get_volume_point(4,16*fi,16*fj,16*fk)*0.0625;
            //res += get_volume_point(5,32*fi,32*fj,32*fk)*0.03125;
            //res += get_volume_point(6,64*fi,64*fj,64*fk)*0.03125*0.5;
            //res += get_volume_point(7,128*fi,128*fj,128*fk)*0.03125*0.25;
            //res += get_volume_point(8,256*fi,256*fj,256*fk)*0.03125*0.125;*/
            //res += volume[(2*i)%volume_x][(2*j)%volume_y][(2*k)%volume_z] * 0.5;
            //res += volume[(4*i)%volume_x][(4*j)%volume_y][(4*k)%volume_z] * 0.25;

            float dist = get_squared_distance_to_void(i,j,k);
            /*float m = 6.0;
            float m_reserve = 1.0 / m;
            if(res>m)
                return m;*/

             /*if(j>10)
                return 1.0;*/



            float res = 0.0;


            float mult = 0.08;
            /*if(j<0)
                mult = 6.0;*/


            float mult_random = fabs(j*0.5 + 5);
            /*if(mult_random<0.0)
                mult_random = 0.0;*/
            /*if(j<0)
            {
                //res = dist * mult + get_volume_point(0,i*0.15,j*0.15,k*0.15) * 0.6;
                res = 1;
            }
            else*/
            {
                res = dist * mult - 3;
                res+=get_volume_point(1,i*0.05,j*0.05,k*0.05) * mult_random * 0.3;
                //res+=get_volume_point(2,i*0.3,j*0.05,k*0.3) * mult_random * 0.6 * 2;
                /*float init_xz = 0.05;
                float init_y = 0.05;
                float init_mult = 1.0;

                for(int it = 1;it<8;it++)
                {
                    res += get_volume_point(it,i*init_xz,j*init_y,k*init_xz) * mult_random * 0.6 * init_mult;
                    init_xz*=4;
                    init_y*=4;
                    init_mult*=0.5;
                }*/


                /*float warpx = get_volume_point(1, i*0.1, j*0.1, k*0.1 );
                float warpy = get_volume_point(4, i*0.1, j*0.1, k*0.1 );
                float warpz = get_volume_point(5, i*0.1, j*0.1, k*0.1 );
                i += warpx * 16;
                j += warpy * 16;
                k += warpz * 16;*/
                //res += get_volume_point(3,i*0.5,j*0.04,k*0.5) * mult_random * 0.5;
                /*res += get_volume_point(4,i*1.0,j*1.0,k*1.0) * mult_random * 0.3;
                res += get_volume_point(5,i*2.0,j*2.0,k*2.0) * mult_random * 0.3 * 0.5;
                res += get_volume_point(6,i*4.0,j*4.0,k*4.0) * mult_random * 0.3 * 0.25;*/
            }

            //res += get_volume_point(0,i*0.2,j*0.2,k*0.2) * 3.0;




                //res += get_volume_point(0,i * 2,j*0.08,k * 2) * mult_random * 0.5;
                //res += get_volume_point(1,i*0.1,j*0.1,k*0.1) * mult_random *0.5;
                //res += get_volume_point(2,i*0.2,j*0.2,k*0.2) * mult_random *0.25;
                //res += get_volume_point(3,i*0.4,j*0.4,k*0.4) * mult_random * 0.125;
                /*res += get_volume_point(0,i*0.2,j*0.2,k*0.2) * 5.0;
                res += get_volume_point(0,i*0.4,j*0.4,k*0.4) * 2.0;*/
            /*res += get_volume_point(1,i*0.4,j*0.4,k*0.4) * 1.5;
            res += get_volume_point(2,i*0.8,j*0.8,k*0.8) * 0.75;*/

            if(res>1.0)
                res = 1.0;
            else if(res<-1.0)
                res = -1.0;


            return res;

        }

        //BUILD RANDOM GEOMETRY
        void build_random_geometry()
        {
            srand ( time(NULL) );
            for(int i = 0;i<volume_x;++i)
                for(int j = 0;j<volume_y;++j)
                    for(int k = 0;k<volume_z;++k)
                        for(int o = 0;o<octaves;++o)
                            volume[o][i][j][k] = ((2.0)*((float)rand()/RAND_MAX))-1.0;




            ManualObject*  manual_geometry = mSceneMgr->createManualObject("GeometryManual");
            manual_geometry->begin("Main/Cave", RenderOperation::OT_TRIANGLE_LIST);




            int index = 0;

            int mx = (real_max_x + 5.0) * blocks_per_point;
            int my = (height_y + 5.0)* blocks_per_point;
            int mz = (real_max_y - 5.0)* blocks_per_point;
            float blocks_per_point_reverse = 1.0 / blocks_per_point;

            for(int i_int = -20.0 * blocks_per_point;i_int<mx + 20.0 * blocks_per_point;++i_int)
                for(int j_int = -10.0 * blocks_per_point;j_int<my + 20.0 * blocks_per_point;++j_int)
                    for(int k_int = 20 * blocks_per_point;k_int>mz - 20.0 * blocks_per_point;--k_int)
                        {

                            float i = i_int*blocks_per_point_reverse;
                            float j = j_int*blocks_per_point_reverse;
                            float k = k_int*blocks_per_point_reverse;
                            float i1 = (i_int + 1.0)*blocks_per_point_reverse;
                            float j1 = (j_int + 1.0)*blocks_per_point_reverse;
                            float k1 = (k_int + 1.0)*blocks_per_point_reverse;
                            float density[8] = {get_density(i,j,k),get_density(i,j1,k),get_density(i1,j1,k),get_density(i1,j,k),get_density(i,j,k1),get_density(i,j1,k1),get_density(i1,j1,k1),get_density(i1,j,k1)};
                            draw_node(manual_geometry,Vector3(i_int,j_int,k_int),Vector3(blocks_per_point_reverse,blocks_per_point_reverse,blocks_per_point_reverse),index,density);

                        }


            cout<<"POLYGONS"<<index/3<<endl;



            manual_geometry->end();
            MeshPtr mesh = manual_geometry->convertToMesh("Geometry");
            meshmagick::OptimiseTool optim;
            optim.setNormTolerance(0.01);
            optim.setPosTolerance(0.01);
            optim.setUVTolerance(0.01);

            //optim.processMeshAndSimplify(mesh,0.5);
            optim.processMesh(mesh);



            Entity * ent_geometry = mSceneMgr->createEntity("Geometry");


            StaticGeometry* all_geometry = mSceneMgr->createStaticGeometry("AllStatic");



            all_geometry->addEntity(ent_geometry,Ogre::Vector3(0,0,0),Quaternion::IDENTITY,scale);

            Entity* end = mSceneMgr->createEntity("end","sphere.mesh");
            end->setMaterialName("Main/End");

            Vector3 in_end = Vector3(portal_out.x, 0, portal_out.y);
            in_end = to_global_space(in_end);
            in_end.y = 750;

            all_geometry->addEntity(end,in_end,Quaternion::IDENTITY,Vector3::UNIT_SCALE*3);

            //PORTALS
            Entity * ent_portals = render_portals();
            if(ent_portals)
                all_geometry->addEntity(ent_portals,Ogre::Vector3(0,0,0),Quaternion::IDENTITY,scale);



            all_geometry->build();

        }

        //RENDER ONLY ONE PORTAL
        void render_portal(ManualObject* manual_portals, int index, Portal portal, Vector3 color)
        {
            float dx = 8;
            float dy = 5;
            float dtex  = 0.0;
            float color_multiplier = 0.6;
            Vector3 normal;
            switch(portal.rotation)
            {

                case Portal::down:
                     normal = Vector3(0,0,-1);
                     dy*=-1;
                case Portal::up:
                     normal = Vector3(0,0,1);


                    manual_portals->position(portal.position.x - 5 - dx, - dx,portal.position.y - dy);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(-dtex,-dtex);
                    manual_portals->normal(normal);

                    manual_portals->position(portal.position.x + 5 + dx, - dx,portal.position.y - dy);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(1+dtex,-dtex);
                    manual_portals->normal(normal);

                    manual_portals->position(portal.position.x - 5 - dx,height_y + dx,portal.position.y - dy);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(-dtex,1+dtex);
                    manual_portals->normal(normal);

                    manual_portals->position(portal.position.x + 5 + dx,height_y + dx,portal.position.y - dy);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(1+dtex,1+dtex);
                    manual_portals->normal(normal);

                    manual_portals->index(index);
                    manual_portals->index(index + 1);
                    manual_portals->index(index + 2);


                    manual_portals->index(index + 3);
                    manual_portals->index(index + 2);
                    manual_portals->index(index + 1);
                 break;

                 case Portal::right:
                    normal = Vector3(-1,0,0);
                    dy*=-1;
                 case Portal::left:
                    normal = Vector3(1,0,0);

                    manual_portals->position(portal.position.x-dy,-dx,portal.position.y - 2*dx);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(-dtex,-dtex);
                    manual_portals->normal(normal);

                    manual_portals->position(portal.position.x-dy,-dx,portal.position.y+10+ dx*0.5);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(1+dtex,-dtex);
                    manual_portals->normal(normal);

                    manual_portals->position(portal.position.x-dy,height_y + dx,portal.position.y - 2*dx);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(-dtex,1+dtex);
                    manual_portals->normal(normal);

                    manual_portals->position(portal.position.x-dy,height_y + dx,portal.position.y+10+ dx*0.5);
                    manual_portals->colour(color.x*color_multiplier,color.y*color_multiplier,color.z*color_multiplier,0);
                    manual_portals->textureCoord(1+dtex,1+dtex);
                    manual_portals->normal(normal);

                    manual_portals->index(index);
                    manual_portals->index(index + 1);
                    manual_portals->index(index + 2);


                    manual_portals->index(index + 1);
                    manual_portals->index(index + 2);
                    manual_portals->index(index + 3);
                 break;
            }



        }

        //CREATE ENTINITY OF PORTALs
        Entity * render_portals()
        {
            ManualObject*  manual_portals = mSceneMgr->createManualObject("PortalsManual");
            manual_portals->begin("Main/Portal", RenderOperation::OT_TRIANGLE_LIST);

                int index = 0;
                for(std::vector<PortalPair>::iterator it = portals.begin();it!=portals.end();++it)
                {
                    render_portal(manual_portals,index,it->portal1,it->color);
                    index += 4;
                    render_portal(manual_portals,index,it->portal2,it->color);
                    index += 4;
                }
                if(!index)
                    return 0;


            manual_portals->end();
            manual_portals->convertToMesh("Portals");
            Entity* res = mSceneMgr->createEntity("Portals");
            for ( int iSubEntity = 0; iSubEntity < (int)res->getNumSubEntities(); ++iSubEntity )
            {
                MaterialPtr pMaterial = res->getSubEntity( iSubEntity )->getMaterial();
                Technique* pTechnique = pMaterial->getBestTechnique();
                Pass* pPass = pTechnique->getPass( 0 );
                params_portal = pPass->getFragmentProgramParameters();
            }
            return res;
        }

        //INSERT PORTAL QUAD ALL WALLS
        Portal add_portal(xml_node portal)
        {
            Portal portal_out;
            portal_out.position.x = to_real_x(portal.attribute("x").as_double());
            portal_out.position.y = to_real_y(portal.attribute("y").as_double());
            portal_out.center = portal_out.position;
            Quad q;
            float half_width = 5;
            float half_height = 3;

            if(!strcmp(portal.attribute("direction").value(),"up"))
            {
                portal_out.position.y -= half_height;
                portal_out.rotation = Portal::up;

                q.x0 = portal_out.position.x - half_width;
                q.x1 = portal_out.position.x + half_width;
                q.y0 = portal_out.position.y - half_height;
                q.y1 = portal_out.position.y + half_height;



             }else if(!strcmp(portal.attribute("direction").value(),"right"))
             {
                portal_out.rotation = Portal::right;
                portal_out.position.x += 5 + half_height;
                portal_out.position.y += 5;

                q.x0 = portal_out.position.x - half_height;
                q.x1 = portal_out.position.x + half_height;
                q.y0 = portal_out.position.y - half_width;
                q.y1 = portal_out.position.y + half_width;

             }
             else if(!strcmp(portal.attribute("direction").value(),"left"))
             {
                portal_out.rotation = Portal::left;
                portal_out.position.x -= 5 + half_height;
                portal_out.position.y += 5;

                q.x0 = portal_out.position.x - half_height;
                q.x1 = portal_out.position.x + half_height;
                q.y0 = portal_out.position.y - half_width;
                q.y1 = portal_out.position.y + half_width;

            }else //down
            {
                portal_out.rotation = Portal::down;
                portal_out.position.y += 10 + half_height;
                q.x0 = portal_out.position.x - half_width;
                q.x1 = portal_out.position.x + half_width;
                q.y0 = portal_out.position.y - half_height;
                q.y1 = portal_out.position.y + half_height;


            }
            q.strong = true;
            quads.push_back(q);

            Block b;
            b.constant = q.x0;b.differ.x = q.y0;b.differ.y = q.y1;b.is_positive_direction = true;merge_block_vertical(b);
            b.constant = q.x1;b.differ.x = q.y0;b.differ.y = q.y1;b.is_positive_direction = false;merge_block_vertical(b);

            b.constant = q.y0;b.differ.x = q.x0;b.differ.y = q.x1;b.is_positive_direction = true;merge_block_horizontal(b);
            b.constant = q.y1;b.differ.x = q.x0;b.differ.y = q.x1;b.is_positive_direction = false;merge_block_horizontal(b);
            return portal_out;
        }


        //FILL VECTOR WITH PORTALS, PATCH BLOKS AND QUADS
        void parse_portals()
        {
            xpath_node_set portals = doc.select_nodes("/map/portals/portalpair");


            for (xpath_node_set::const_iterator it = portals.begin(); it != portals.end(); ++it)
            {
                PortalPair insert_portal;
                xpath_node portalpair = *it;
                if(strcmp(portalpair.node().attribute("color").value(),"red")==0)
                {
                    insert_portal.color = Vector3(1,0,0);
                }else if (strcmp(portalpair.node().attribute("color").value(),"green")==0)
                {
                    insert_portal.color = Vector3(0,1,0);
                }else
                {
                    insert_portal.color = Vector3(0,0,1);
                }
                xml_node portal1 = portalpair.node().child("portal");
                insert_portal.portal1 = add_portal(portal1);
                xml_node portal2 = portal1.next_sibling("portal");
                insert_portal.portal2 = add_portal(portal2);



                this->portals.push_back(insert_portal);
            }



        }


        //FIND IF BLOCK CONTAINS ANOTHER ONE AND MAKE HOLE IN IT
        void merge_block(Block b,std::vector<Block>& dest)
        {



            for(std::vector<Block>::iterator it=dest.begin();it!=dest.end();++it)
            {


                if(fabs(it->constant - b.constant)<0.001&& b.differ.x - it->differ.x>-0.001 && it->differ.y - b.differ.y > -0.001)
                {
                    Block b1; b1.constant = it->constant; b1.differ.x = it->differ.x;b1.differ.y = b.differ.x;b1.is_positive_direction = it->is_positive_direction;
                    Block b2; b2.constant = it->constant; b2.differ.x = b.differ.y;b2.differ.y = it->differ.y;b2.is_positive_direction = it->is_positive_direction;
                    dest.erase(it);
                    if(fabs(b1.differ.x - b1.differ.y)>0.01)
                    {
                        dest.push_back(b1);
                    }
                    if(fabs(b2.differ.x - b2.differ.y)>0.01)
                    {
                        dest.push_back(b2);
                    }
                    return;
                }
            }
            dest.push_back(b);

        }


        void merge_block_vertical(Block b)
        {
            merge_block(b,blocks_vertical);
        }


        void merge_block_horizontal(Block b)
        {
            merge_block(b,blocks_horizontal);
        }

        //BUILD REGULAR GEOMETRY
        void build_geometry()
        {


            xpath_node_set points = doc.select_nodes("/map/points/point");

            for(xpath_node_set::const_iterator it = points.begin(); it != points.end(); ++it)
            {
                xpath_node point = *it;
                double x = to_real_x(point.node().attribute("x").as_double());
                double y = to_real_y(point.node().attribute("y").as_double());


                //DRAW CORNERS
                NodeDirectons directions = get_directions(point.node().attribute("id").value());
                if(!directions.is_back)
                {
                    Block b;b.constant = y + 10.0;b.differ = Vector2(-5.0 + x,5.0 + x);b.arrange();b.is_positive_direction = false;blocks_horizontal.push_back(b);
                }
                if(!directions.is_forw)
                {
                    Block b;b.constant = y;b.differ = Vector2(-5.0 + x,5.0 + x);b.arrange();b.is_positive_direction = true;blocks_horizontal.push_back(b);
                }
                if(!directions.is_left)
                {
                    Block b;b.constant = -5.0 + x;b.differ = Vector2(y,y + 10.0);b.arrange();b.is_positive_direction = true;blocks_vertical.push_back(b);
                }
                if(!directions.is_right)
                {
                    Block b;b.constant = 5.0 + x;b.differ = Vector2(y,y + 10.0);b.arrange();b.is_positive_direction = false;blocks_vertical.push_back(b);
                }

            }


            xpath_node_set lines = doc.select_nodes("/map/lines/line");
            for (xpath_node_set::const_iterator it = lines.begin(); it != lines.end(); ++it)
            {
                pugi::xpath_node line = *it;
                Vector2 start = get_point_by_id(line.node().attribute("start").value());
                Vector2 end = get_point_by_id(line.node().attribute("end").value());
                Vector2 real_start = to_real(start);
                Vector2 real_end = to_real(end);

                int start_index = 4 * get_index_by_id(line.node().attribute("start").value());
                int end_index = 4 * get_index_by_id(line.node().attribute("end").value());

                //DRAW WALL NODES NOT CORNERS
                if(fabs(start.x - end.x)<0.1)
                {
                    if(start.y<end.y)
                    {
                        Vector2 swap;
                        swap = start;start = end;end = swap;
                        swap = real_start;real_start = real_end;real_end = swap;
                        int swap_i;
                        swap_i = start_index;start_index = end_index;end_index = swap_i;
                    }
                    {
                        Block b1;b1.constant = -5.0 + real_start.x;b1.differ = Vector2(real_end.y,real_start.y + 10.0);b1.arrange();b1.is_positive_direction = true;blocks_vertical.push_back(b1);
                        Block b2;b2.constant = 5.0 + real_start.x;b2.differ = Vector2(real_end.y,real_start.y + 10.0);b2.arrange();b2.is_positive_direction = false;blocks_vertical.push_back(b2);
                        Quad q(-5.0 + real_start.x,5.0 + real_start.x,real_start.y + 10.0,real_end.y);q.arrange();q.y0-=10.0;q.y1+=10.0;quads.push_back(q);
                    }

                }else
                {

                    if(start.x<end.x)
                    {


                        Vector2 swap;
                        swap = start;start = end;end = swap;
                        swap = real_start;real_start = real_end;real_end = swap;
                        int swap_i;
                        swap_i = start_index;start_index = end_index;end_index = swap_i;
                    }
                        Block b1;b1.constant = real_start.y+10.0;b1.differ = Vector2(-5.0 + real_start.x,5.0 + real_end.x);b1.arrange();b1.is_positive_direction = false;blocks_horizontal.push_back(b1);
                        Block b2;b2.constant = real_start.y;b2.differ = Vector2(-5.0 + real_start.x,5.0 + real_end.x);b2.arrange();b2.is_positive_direction = true;blocks_horizontal.push_back(b2);
                        Quad q(-5.0 + real_start.x,5.0 + real_end.x,real_start.y,real_start.y+10.0);q.arrange();q.x0-=10.0;q.x1+=10.0;quads.push_back(q);

                }




            }
            parse_portals();

            compact_blocks();
            build_random_geometry();

            quads.clear();

        }


        //FINDS PORTAL, IF NOT FOUND - RETURNS ZERO
        Portal * find_portal(Vector2 position)
        {
            position.x /= scale.x;
            position.y /= scale.y;
            float scale_y_local = to_real_y(0.075);
            scale_y_local*=scale_y_local;
            for(std::vector<PortalPair>::iterator it = portals.begin();it!=portals.end();++it)
            {
                if(it->portal1.position.squaredDistance(position)<scale_y_local)
                {
                    return &(it->portal2);
                }
                if(it->portal2.position.squaredDistance(position)<scale_y_local)
                {
                    return &(it->portal1);
                }
            }

            return 0;
        }

        //SET TIME FOR RENDERING
        void set_time(double time)
        {
            global_time+=time*10;
            if ( params_portal->_findNamedConstantDefinition( "time"))
            {
                params_portal->setNamedConstant( "time", Real(global_time*0.005) );
            }
        }

};

#endif // MAP_H_INCLUDED




