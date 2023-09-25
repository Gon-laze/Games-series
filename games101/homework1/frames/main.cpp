#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();


    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // ! A trap! this 'cos'is using radium, not angle.....
    // ! get one tips: std::cos(sin and others) use radium and cmath cos(sin and others) use angle
    rotation_angle *= (MY_PI/180.0);
    Eigen::Matrix4f transform;
    transform   <<  cos(rotation_angle), -sin(rotation_angle), 0, 0, 
                    sin(rotation_angle), cos(rotation_angle), 0, 0, 
                    0, 0, 1, 0, 
                    0, 0, 0, 1;
    
    model = transform;
    // std::cout << transform << std::endl << "````````" << std::endl;
    return  model;
}

// added by Gon laze
// a func that can let the model rotates around any axis
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    angle *= (MY_PI/180.0);

    Eigen::Matrix3f M_rotation;
    Eigen::Matrix3f N;
    N <<    0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;
    M_rotation =    cos(angle) * Eigen::Matrix3f::Identity() +
                    (1-cos(angle)) * axis * axis.transpose() +
                    sin(angle) * N;

    Eigen::Matrix4f ans;
    ans <<  M_rotation(0, 0), M_rotation(0, 1), M_rotation(0, 2), 0, 
            M_rotation(1, 0), M_rotation(1, 1), M_rotation(1, 2), 0, 
            M_rotation(2, 0), M_rotation(2, 1), M_rotation(2, 2), 0, 
            0, 0, 0, 1;

    return ans;

}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    /* 2 step convert: */
    // 1. perspect -> ortho
    Eigen::Matrix4f M_persp2ortho;
    M_persp2ortho <<    zNear, 0, 0, 0,
                        0, zNear, 0, 0,
                        0, 0, zNear+zFar, -(zNear*zFar),
                        // -1 for z should be a negative value
                        0, 0, -1, 0;

    // 2. ortho ->(Translate & scale) final
    float Space_height = 2 * zNear * tan(eye_fov/2);
    float Space_width = Space_height * aspect_ratio;
    float Space_distance = zFar - zNear;

    Eigen::Matrix4f M_ortho_trans;
    Eigen::Matrix4f M_ortho_scale;
    M_ortho_trans = Eigen::Matrix4f::Identity();
    M_ortho_scale   <<  2/Space_width, 0, 0, 0,
                        0, 2/Space_height, 0, 0,
                        0, 0, 2/Space_distance, 0,
                        0, 0, 0, 1;

    projection = M_ortho_scale * M_ortho_trans * M_persp2ortho;

    return projection;
}

const Eigen::Vector3f Axis_x = {1, 0, 0};
const Eigen::Vector3f Axis_y = {0, 1, 0};
const Eigen::Vector3f Axis_z = {0, 0, 1};


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    // Eigen::Vector3f eye_pos = {0, 0, 5};
    Eigen::Vector3f eye_pos = {0, 0, 8};


    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {1, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    // added by Gon laze
    float angle_x = 0;
    float angle_y = 0;
    float angle_z = angle;

    if (command_line) {

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));

        // added by Gon laze
        r.set_model(
            get_rotation(Axis_x, angle_x) * 
            get_rotation(Axis_y, angle_y) * 
            get_rotation(Axis_z, angle_z)
        );
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(
            get_rotation(Axis_x, angle_x) * 
            get_rotation(Axis_y, angle_y) * 
            get_rotation(Axis_z, angle_z)
        );
        std::cout << angle_x << '\t' << angle_y << '\t' << angle_z << std::endl;
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);


        std::cout << "frame count: " << frame_count++ << '\n';


        if (key == 'a') {
            angle_z += 10;
        }
        else if (key == 'd') {
            angle_z -= 10;
        }
        else if (key == 'w') {
            angle_x += 10;
        }
        else if (key == 's') {
            angle_x -= 10;
        }
        else if (key == 'q') {
            angle_y += 10;
        }
        else if (key == 'e') {
            angle_y -= 10;
        }    
    }

    return 0;
}
