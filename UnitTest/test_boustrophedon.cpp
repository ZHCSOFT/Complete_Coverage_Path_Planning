#include <iostream>
#include "boustrophedon_explorator.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "yaml-cpp/yaml.h"


int main()
{
    BoustrophedonExplorer boustrophedon_explorer_;
    std::vector<geometry_msgs::Pose2D> exploration_path;

    cv::Mat room_map = cv::imread("/home/zhcsoft/pythonProjects/a_star/sample/20230824.pgm");
    YAML::Node room_config = YAML::LoadFile("/home/zhcsoft/pythonProjects/a_star/sample/20230824.yaml");
    YAML::Node amr_config = YAML::LoadFile("/home/zhcsoft/pythonProjects/a_star/sample/amr.yaml");

    //make non-white pixels black
	for (int y = 0; y < room_map.rows; y++)
	{
		for (int x = 0; x < room_map.cols; x++)
		{
			//find not reachable regions and make them black
			if (room_map.at<unsigned char>(y, x) < 250)
			{
				room_map.at<unsigned char>(y, x) = 0;
			}
			//else make it white
			else
			{
				room_map.at<unsigned char>(y, x) = 255;
			}
		}
	}

    // Parameters

    // map_params
    float map_resolution = 0.0;
    int map_starting_position_x = 0;
    int map_starting_position_y = 0;
    double map_starting_position_theta = 0.0;
    int map_origin_x = 0;
    int map_origin_y = 0;
    double map_origin_theta = 0.0;

    double map_path_eps_px = 0.0;
    double map_min_cell_area_px = 0.0;
    int map_max_deviation_from_track_px = 0;

    try
    {
        map_resolution = room_config["resolution"].as<float>();

        map_starting_position_x = room_config["starting_position"][0].as<int>();
        map_starting_position_y = room_config["starting_position"][1].as<int>();
        map_starting_position_theta = room_config["starting_position"][2].as<double>();
        map_origin_x = (int)room_config["origin"][0].as<float>();
        map_origin_y = (int)room_config["origin"][1].as<float>();
        map_origin_theta = room_config["origin"][2].as<double>();

        map_path_eps_px = room_config["path_eps_px"].as<double>();
        map_min_cell_area_px = room_config["min_cell_area_px"].as<double>();
        map_max_deviation_from_track_px = room_config["max_deviation_from_track_px"].as<int>();
    }
    catch (std::exception& e)
    {
        std::cout << "[ ERROR ] Failed read from map_yaml for exception" << e.what() << std::endl;
    }
    
    cv::Point map_starting_position;
    map_starting_position.x = map_starting_position_x;
    map_starting_position.y = map_starting_position_y;

    cv::Point2d map_origin;
    map_origin.x = map_origin_x;
    map_origin.y = map_origin_y;

    // amr_params
    double amr_coverage_radius = 0.0;
    double amr_grid_obstacle_offset = 0.0;
    int amr_cell_visiting_order = 1;
    int planning_mode = 1;
    bool amr_plan_for_footprint = false;
    Eigen::Matrix<float, 2, 1> amr_to_fov_vector_meter;
    double amr_grid_spacing_meter = 0.0;

    try
    {
        amr_coverage_radius = amr_config["coverage_radius"].as<double>();
        amr_grid_obstacle_offset = amr_config["grid_obstacle_offset"].as<double>();
        amr_cell_visiting_order = amr_config["cell_visiting_order"].as<int>();
        planning_mode = amr_config["planning_mode"].as<int>();
    }
    catch(const std::exception& e)
    {
        std::cout << "[ ERROR ] Failed read from amr_yaml for exception" << e.what() << std::endl;
    }

    if (planning_mode == 1)
    {
        amr_plan_for_footprint = true;

        std::vector<Eigen::Matrix<float, 2, 1> > fov_corners_meter(4);
        for (int i=0; i<4; i++)
        {
            try
            {
                fov_corners_meter[i] << amr_config["fov_points"][i]["x"].as<float>(), amr_config["fov_points"][i]["y"].as<float>();
            }
                
            catch(const std::exception& e)
            {
                std::cout << "[ ERROR ] Failed read fov_points from amr_yaml for exception" << e.what() << std::endl;
                fov_corners_meter[i] << 0.0, 0.0;
            }
        }

        float amr_fitting_circle_radius_meter = amr_config["fitting_circle_radius_meter"].as<float>();
        double amr_fov_resolution = amr_config["fov_resolution"].as<double>();

        computeFOVCenterAndRadius(fov_corners_meter, amr_fitting_circle_radius_meter, amr_to_fov_vector_meter, amr_fov_resolution);

        amr_grid_spacing_meter = amr_fitting_circle_radius_meter * std::sqrt(2);
    
    }
    else if (planning_mode == 2)
    {
        amr_plan_for_footprint = false;
        amr_grid_spacing_meter = amr_coverage_radius * std::sqrt(2);
        amr_to_fov_vector_meter << 0.0, 0.0;
    }
    else
    {
        std::cout << "[ ERROR ] Invalid amr planning mode on yaml" << std::endl;
    }

    double amr_grid_spacing_px = std::floor(amr_grid_spacing_meter / map_resolution);
    // Parameters (END)

    boustrophedon_explorer_.getExplorationPath(room_map, exploration_path,
                                               map_resolution, map_starting_position, map_origin,
                                               amr_grid_spacing_px, amr_grid_obstacle_offset,
                                               map_path_eps_px, amr_cell_visiting_order, amr_plan_for_footprint,
                                               amr_to_fov_vector_meter, map_min_cell_area_px, map_max_deviation_from_track_px);

    return 0;
}