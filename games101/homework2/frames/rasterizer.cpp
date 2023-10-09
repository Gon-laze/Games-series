// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>



rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    // // float ans = 0.0;
    // use cross product ans' zVal to judge --by Gon laze
    float cross_Z[3];
    Vector2f pVec[3];
    // // for (int u = 0; u < rst::superSample_size; u++)
    // // {
    // //     auto tx = x + (u % rst::superSample_size)*rst::superSample_Wmeta;
    // //     auto ty = y + (u / rst::superSample_size)*rst::superSample_Wmeta;
    // //     for (int i = 0; i < 3; i++)
    // //     {
    // //         pVec[i].x() = tx - _v[i].x();
    // //         pVec[i].y() = ty - _v[i].y();
    // //     }
    // //     for (int i = 0; i < 3; i++)
    // //         cross_Z[i] = (pVec[(i+1)%3].x() * pVec[(i+2)%3].y()) - (pVec[(i+1)%3].y() * pVec[(i+2)%3].x());

    // //     if (cross_Z[0] >=0)     ans += (cross_Z[1]>=0) && (cross_Z[2]>=0) ? rst::superSample_Smeta : 0.0;
    // //     else                    ans += (cross_Z[1]<=0) && (cross_Z[2]<=0) ? rst::superSample_Smeta : 0.0;
    // // }

    // // return ans;
        for (int i = 0; i < 3; i++)
        {
            pVec[i].x() = x - _v[i].x();
            pVec[i].y() = y - _v[i].y();
        }
        for (int i = 0; i < 3; i++)
            cross_Z[i] = (pVec[(i+1)%3].x() * pVec[(i+2)%3].y()) - (pVec[(i+1)%3].y() * pVec[(i+2)%3].x());

        if (cross_Z[0] >=0)     return (cross_Z[1]>=0) && (cross_Z[2]>=0);
        else                    return (cross_Z[1]<=0) && (cross_Z[2]<=0);
        
}

/*
    These formulas are supposed to yield a set of generalizations: 
    for any projected point P(x,y,z) IN THE SAME PLANE as the triangle, this can be used to compute alpha, beta, and gamma such that P = (alpha)OA + (beta)OB + (gamma)OC
    In particular: for any point P inside the triangle (including the boundary), we have alpha + beta + gamma = 1
    
    *P.S. surmise that different plane also works but it means nothing there(we only want 2D calculate for points that in the same plane)
    note by Gon laze
*/
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{

    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();

        //     // add by Gon laze
        //     // {std::cout << vec << std::endl << "``````````\n";}
        }
        // // add by Gon laze
        // // {std::cout << std::endl << "##########\n";}
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            // ? So confused. Probably wrong(overdone).
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    float x_min = floor(std::min(std::min(v[0].x(), v[1].x()), v[2].x()));
    float x_max = floor(std::max(std::max(v[0].x(), v[1].x()), v[2].x()));
    float y_min = floor(std::min(std::min(v[0].y(), v[1].y()), v[2].y()));
    float y_max = floor(std::max(std::max(v[0].y(), v[1].y()), v[2].y()));

    //    // std::cout << "x: " << x_min << '\t' << x_max << std::endl;
    //    // std::cout << "y: " << y_min << '\t' << y_max << std::endl;
    //    // std::cout << t.v[0] << std::endl;
    //    // std::cout << t.v[1] << std::endl;
    //    // std::cout << t.v[2] << std::endl;
    for (auto _y_ = y_min; _y_ <= y_max; _y_++)
    {
        for (auto _x_ = x_min; _x_ <= x_max; _x_++)
        {
            for (auto u = 0; u < superSample_size; u++)
            {
                // use center axis to judge
                auto x = _x_ + (u % superSample_width) * superSample_Wmeta + superSample_Wmeta / 2.0;
                auto y = _y_ + (u / superSample_width) * superSample_Wmeta + superSample_Wmeta / 2.0;

                if (insideTriangle(x, y, t.v) == false)
                    continue;
                //  // std::cout << x << '\t' << y <<'\t' << superSample_level << std::endl;
                // ! TODO: This bounding box could be so huge! Try other ways to boost.
                
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                /*
                    This part may looks confusing, but actually the code is trying to express that in the final result 
                    (refers to the model that has been perspective-orthogonal transformed, but still retains the z-va-
                    lue and has not yet been projected in 2D), alpha /= v[0].w() (because we didn't divide by w earli-
                    er in our calculations to make it homogeneous).        
                    more info: https://zhuanlan.zhihu.com/p/448575965
                    ? still confused. What is the w_reciprocal for(should be a const val(1) when P is in the triangle)? 
                    ? Perhaps a general solution for points no matter if they are in traingle?
                    TODO: try to comprehend.
                    TODO: try to simplifed this.
                */
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                auto pIndex = rst::rasterizer::get_index(x, y);
                if (depth_buf[u][pIndex] > z_interpolated)
                {
                    depth_buf[u][pIndex] = z_interpolated;
                    
                    // ? Try some color mixing (is this correct?)
                    set_pixel({floor(x),floor(y),z_interpolated}, frame_buf[pIndex] + t.getColor() * (float)superSample_Smeta);
                }                
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        // edit by Gon laze: superSample: create depthBuf for each sample
        for (auto & db : depth_buf)
            std::fill(db.begin(), db.end(), std::numeric_limits<float>::infinity());
        // std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    // edit by Gon laze: superSample: create depthBuf for each sample
    for (auto & db : depth_buf)
        db.resize(w * h);
    // depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;

    // add by Gon laze: border detect(from homework 1)
    if (point.x() < 0 || point.x() >= width ||
    point.y() < 0 || point.y() >= height) return;

    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on