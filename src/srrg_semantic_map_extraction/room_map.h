#pragma once

#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "lines_extractor.h"
#include "color_extractor.h"

class RoomMap {
    private:
        cv::Mat original_map;
        cv::Mat original_with_doors_map;
        cv::Mat room_map;
        ColorExtractor color_extractor;
    public:
        RoomMap(const cv::Mat &original_map);
        void addDoors(std::vector<std::pair<cv::Point2i,cv::Point2i> > doors);
        void addDoors(const std::pair<cv::Point2i,cv::Point2i> &door);
        void extractRooms(const std::vector<cv::Point> &tags);
        cv::Mat getRoomByTag(const cv::Point &tag);
};
