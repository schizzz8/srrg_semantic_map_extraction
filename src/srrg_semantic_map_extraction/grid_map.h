#pragma once

#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "lines_extractor.h"

class GridMap {
private:
    cv::Mat original_map;
    std::vector<float> horizontal_lines;
    std::vector<float> vertical_lines;
    std::vector<cv::Mat> stack;
    std::vector<std::string> operations;
    std::vector<cv::Vec2f> extracted_grid;

    std::vector<float> computeLineDistances(std::vector<float> lines);
    void extractGrid();
public:
    GridMap(const cv::Mat &original_map);
    void produce();
    void showStack();
    bool saveStack(std::string path="./", std::string basename="map");
    cv::Point2i getCellFromImageCoords(cv::Point2i coords);
    cv::Point2i getImageCoordsFromCell(cv::Point2i cell);
    std::vector<float> getHorizontalDistances();
    std::vector<float> getVerticalDistances();
    const std::vector<cv::Vec2f>& grid(){return extracted_grid;}
};

