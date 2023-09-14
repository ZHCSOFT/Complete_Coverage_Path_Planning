#include <fstream>
#include <opencv2/imgcodecs.hpp>

#include "dataloader.h"
#include "ipa_room_exploration/fov_to_robot_mapper.h"


DataLoader::DataLoader(){}

DataLoader::~DataLoader(){}

MapAndConfigData DataLoader::loadData(std::string map_path, std::string config_map_path, std::string config_amr_path)
{
    std::ifstream map_stream(map_path);
    if (!map_stream)
    {
        throw std::runtime_error("[ FATAL ] Failed to read map_path: " + map_path);
    }
    std::ifstream config_map_stream(config_map_path);
    if (!config_map_stream)
    {
        throw std::runtime_error("[ FATAL ] Failed to read config_map_path: " + config_map_path);
    }
    std::ifstream config_amr_stream(config_amr_path);
    if (!config_amr_stream)
    {
        throw std::runtime_error("[ FATAL ] Failed to read config_amr_path: " + config_amr_path);
    }

    // map file
    cv::Mat room_map = cv::imread(map_path);

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

    map_and_config_data.room_map = room_map;
    // map file (END)

    // room_config file
    YAML::Node room_config = YAML::LoadFile(config_map_path);

    int map_starting_position_x = 0;
    int map_starting_position_y = 0;

    int map_origin_x = 0;
    int map_origin_y = 0;

    try
    {
        map_and_config_data.map_resolution = room_config["resolution"].as<float>();

        map_starting_position_x = room_config["starting_position"][0].as<int>();
        map_starting_position_y = room_config["starting_position"][1].as<int>();
        map_and_config_data.map_starting_position_theta = room_config["starting_position"][2].as<double>();
        map_origin_x = (int)room_config["origin"][0].as<float>();
        map_origin_y = (int)room_config["origin"][1].as<float>();
        map_and_config_data.map_origin_theta = room_config["origin"][2].as<double>();

        map_and_config_data.map_path_eps_px = room_config["path_eps_px"].as<double>();
        map_and_config_data.map_min_cell_area_px = room_config["min_cell_area_px"].as<double>();
        map_and_config_data.map_max_deviation_from_track_px = room_config["max_deviation_from_track_px"].as<int>();
    }
    catch (std::exception& e)
    {
        std::cout << "[ ERROR ] Failed read from map_yaml for exception" << e.what() << std::endl;
    }

    map_and_config_data.map_starting_position.x = map_starting_position_x;
    map_and_config_data.map_starting_position.y = map_starting_position_y;

    map_and_config_data.map_origin.x = map_origin_x;
    map_and_config_data.map_origin.y = map_origin_y;
    // room_config file (END)

    // amr_config file
    YAML::Node amr_config = YAML::LoadFile(config_amr_path);

    try
    {
        map_and_config_data.amr_coverage_radius = amr_config["coverage_radius"].as<double>();
        map_and_config_data.amr_grid_obstacle_offset = amr_config["grid_obstacle_offset"].as<double>();
        map_and_config_data.amr_cell_visiting_order = amr_config["cell_visiting_order"].as<int>();
        map_and_config_data.planning_mode = amr_config["planning_mode"].as<int>();
    }
    catch(const std::exception& e)
    {
        std::cout << "[ ERROR ] Failed read from amr_yaml for exception" << e.what() << std::endl;
    }
    // amr_config file (END)

    if (map_and_config_data.planning_mode == 1)
    {
        map_and_config_data.amr_plan_for_footprint = true;

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

        computeFOVCenterAndRadius(fov_corners_meter, amr_fitting_circle_radius_meter, map_and_config_data.amr_to_fov_vector_meter, amr_fov_resolution);

        map_and_config_data.amr_grid_spacing_meter = amr_fitting_circle_radius_meter * std::sqrt(2);
    
    }
    else if (map_and_config_data.planning_mode == 2)
    {
        map_and_config_data.amr_plan_for_footprint = false;
        map_and_config_data.amr_grid_spacing_meter = map_and_config_data.amr_coverage_radius * std::sqrt(2);
        map_and_config_data.amr_to_fov_vector_meter << 0.0, 0.0;
    }
    else
    {
        std::cout << "[ ERROR ] Invalid amr planning mode on yaml" << std::endl;
    }

    map_and_config_data.amr_grid_spacing_px = std::floor(map_and_config_data.amr_grid_spacing_meter / map_and_config_data.map_resolution);
    // Parameters (END)

    return map_and_config_data;

}

MapAndConfigData DataLoader::getData()
{
    return map_and_config_data;
}
