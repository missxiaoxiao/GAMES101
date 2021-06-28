// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>

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
static bool checkSign(float a, float b) {
    return (a >= 0 && b >= 0) || (a <= 0 && b <= 0);
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    Vector3f p = Vector3f(x, y, 1);
    Vector3f dir[3] = { p - _v[0], p- _v[1], p -_v[2] };

    Vector3f edge[3] = { _v[0] - _v[1], _v[1] - _v[2],_v[2] - _v[0] };

    float cross[3] =
    {
        dir[0][0] * edge[0][1] - dir[0][1] * edge[0][0],
        dir[1][0] * edge[1][1] - dir[1][1] * edge[1][0],
        dir[2][0] * edge[2][1] - dir[2][1] * edge[2][0],
    };

    return checkSign(cross[0],cross[1])&& checkSign(cross[0],cross[2]);
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

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
        }
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

static bool isMSAA = true;
static Vector2f points[4] =
{
    Vector2f(0.25f,0.25f),
    Vector2f(0.25f,0.75f),
    Vector2f(0.75f,0.25f),
    Vector2f(0.75f,0.75f),
};
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float minX = std::min(std::min(v[0][0], v[1][0]), v[2][0]);
    float minY = std::min(std::min(v[0][1], v[1][1]), v[2][1]);

    float maxX = std::max(std::max(v[0][0], v[1][0]), v[2][0]);
    float maxY = std::max(std::max(v[0][1], v[1][1]), v[2][1]);

    if (isMSAA) {
        float z_interpolated = FLT_MAX;
        float alpha, beta, gamma;
        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {
                int _idx = 0;   
                for (int i = 0; i < 4; i++) {
                    if (insideTriangle(x + points[i][0], y + points[i][1], t.v)) {
                        _idx++;
                        std::tuple<float, float, float> p = computeBarycentric2D(x + points[i][0], y + points[i][1], t.v);
                        alpha = std::get<0>(p);
                        beta = std::get<1>(p);
                        gamma = std::get<2>(p);
                        float w_reciprocal = 1.0f / (alpha / v[0][3] + beta / v[1][3] + gamma / v[2][3]);
                        float z = alpha * v[0][2] / v[0][3] + beta * v[1][2] / v[1][3] + gamma * v[2][2] / v[2][3];
                        z *= w_reciprocal;
                        z_interpolated = std::min(z, z_interpolated);
                    }
                }
                if (_idx != 0) {
                    float a = (float)_idx / 4;
                    auto ind = (height - 1 - y) * width + x;
                    if (z_interpolated < depth_buf[ind]) {
                        depth_buf[ind] = z_interpolated;
                        set_pixel(Vector3f(x, y, depth_buf[ind]), t.getColor() * a);
                    }
                }
            }
        }
    }
    else {
		for (int x = minX; x < maxX; x++) {
			for (int y = minY; y < maxY; y++) {
				if (insideTriangle(x + 0.5f, y + 0.5f, t.v)) {
					float alpha, beta, gamma;
					std::tuple<float, float, float> p = computeBarycentric2D(x, y, t.v);
					alpha = std::get<0>(p);
					beta = std::get<1>(p);
					gamma = std::get<2>(p);
					float w_reciprocal = 1.0f / (alpha / v[0][3] + beta / v[1][3] + gamma / v[2][3]);
                    float z_interpolated = alpha * v[0][2] / v[0][3] + beta * v[1][2] / v[1][3] + gamma * v[2][2] / v[2][3];
                    z_interpolated *= w_reciprocal;
                    auto ind = (height - 1 - y) * width + x;
                    if (z_interpolated < depth_buf[ind]) {
                        depth_buf[ind] = z_interpolated;
                        set_pixel(Vector3f(x, y, depth_buf[ind]), t.getColor());
                    }
                }
            }
        }
    }
 
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on