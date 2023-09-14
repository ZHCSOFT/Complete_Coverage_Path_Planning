#include <iostream>
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

#include "yaml-cpp/yaml.h"


struct MapAndConfigData
{
    cv::Mat room_map;
    // map_params
    float map_resolution = 0.0;

    cv::Point map_starting_position;
    double map_starting_position_theta = 0.0;

    cv::Point2d map_origin;
    double map_origin_theta = 0.0;

    double map_path_eps_px = 0.0;
    double map_min_cell_area_px = 0.0;
    int map_max_deviation_from_track_px = 0;

    // amr_params
    double amr_coverage_radius = 0.0;
    double amr_grid_obstacle_offset = 0.0;
    int amr_cell_visiting_order = 1;
    int planning_mode = 1;
    bool amr_plan_for_footprint = false;
    Eigen::Matrix<float, 2, 1> amr_to_fov_vector_meter;
    double amr_grid_spacing_meter = 0.0;
    double amr_grid_spacing_px = 0.0;

};


class DataLoader
{
public:
    DataLoader();
    ~DataLoader();

    MapAndConfigData loadData(std::string map_path, std::string config_map_path, std::string config_amr_path);
    MapAndConfigData getData();

protected:
    MapAndConfigData map_and_config_data;

};
