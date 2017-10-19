#pragma once

#include <stdint.h>
#include <time.h>

#include <sstream>
#include <string>
#include <vector>
#include <map>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cell.h"
#include "object.h"
#include "room_map.h"


class SemanticMatrix {
 public:
  SemanticMatrix(const int rows, const int cols);
  Cell* getCellByIndexes(const int row, const int col);
  void setRoomTags(const std::vector<std::pair<std::string, cv::Point> > &tags,
                   RoomMap &room_map,
                   const std::vector<float> &check_x,
                   const std::vector<float> &check_y);
  void setObjectTags(const std::vector<std::vector<cv::Point2i> > &cell_objects,
                     std::vector<Object> &object);
  void removeObjectByName(const std::string object_name);
  void removeObjectAdditionals(const std::string additionals);
  void showMatrixImage();
  void showMapImage(const cv::Mat &map,
                    const std::vector<float> &horizontal_distances,
                    const std::vector<float> &vertical_distances);
  std::string saveMatrixImage(std::string path="./",
                              std::string basename="map");
  std::string saveMapImage(const cv::Mat &map,
                           const std::vector<float> &horizontal_distances,
                           const std::vector<float> &vertical_distances,
                           std::string path="./",
                           std::string basename="map");
  std::vector<std::string> msg_format();

  const cv::Mat& matrix(){return _matrix;}

 private:
  int rows;
  int cols;
  std::vector<std::vector<Cell> > semantic_matrix;
  cv::Mat _matrix;

  void addCellValues(int row, int col,
                     std::string object_name = std::string(),
                     std::string additionals = std::string(),
                     bool change_adjacency = false,
                     direction adjacency_dir = RIGHT);
  void setDoor(const std::vector<cv::Point2i> &door_cells,
               Object &door);
  void setObject(const std::vector<cv::Point2i> &object_cells,
                 Object &object);
  cv::Mat drawMatrix();
  cv::Mat drawMap(const cv::Mat &map,
                  const std::vector<float> &horizontal_distances,
                  const std::vector<float> &vertical_distances);
};
