#include <fstream>
#include <opencv2/imgcodecs.hpp>

#include "dataloader.h"


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

    map_and_config_data.room_map = cv::imread(map_path);

    YAML::Node room_config = YAML::LoadFile(config_map_path);
    YAML::Node amr_config = YAML::LoadFile(config_amr_path);

}
