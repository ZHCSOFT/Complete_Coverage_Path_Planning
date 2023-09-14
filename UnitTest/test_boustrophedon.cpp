#include <iostream>

#include "yaml-cpp/yaml.h"
#include "boustrophedon_explorator.h"
#include "dataloader.h"


int main()
{
    BoustrophedonExplorer boustrophedon_explorer_;
    std::vector<geometry_msgs::Pose2D> exploration_path;

    std::string room_map_path = "/home/zhcsoft/pythonProjects/a_star/sample/20230824.pgm";
    std::string room_config_path = "/home/zhcsoft/pythonProjects/a_star/sample/20230824.yaml";
    std::string amr_config_path = "/home/zhcsoft/pythonProjects/a_star/sample/amr.yaml";

    DataLoader dataloader;
    MapAndConfigData map_config_data = dataloader.loadData(room_map_path, room_config_path, amr_config_path);

    boustrophedon_explorer_.getExplorationPath(exploration_path,
                                               map_config_data.room_map, 
                                               map_config_data.map_resolution, map_config_data.map_starting_position, map_config_data.map_origin,
                                               map_config_data.amr_grid_spacing_px, map_config_data.amr_grid_obstacle_offset,
                                               map_config_data.map_path_eps_px, map_config_data.amr_cell_visiting_order, map_config_data.amr_plan_for_footprint,
                                               map_config_data.amr_to_fov_vector_meter, map_config_data.map_min_cell_area_px, map_config_data.map_max_deviation_from_track_px);

    return 0;
}