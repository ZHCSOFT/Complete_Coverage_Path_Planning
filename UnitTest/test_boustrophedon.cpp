#include <iostream>
#include "boustrophedon_explorator.h"
#include <opencv2/core.hpp>


int main()
{
    BoustrophedonExplorer boustrophedon_explorer_;
    std::vector<geometry_msgs::Pose2D> exploration_path;

    cv::Mat room_map = cv::imread("sample/20230824.pgm");

    return 0;
}