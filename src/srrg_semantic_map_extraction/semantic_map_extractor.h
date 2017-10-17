#pragma once

#include <iostream>
#include <string>
#include <tinyxml.h>
#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "grid_map.h"
#include "semantic_matrix.h"

class SemanticMapExtractor{
public:
    SemanticMapExtractor(std::string image_path_,
                         std::string robotname_="",
                         float resolution_=0.05f,
                         float origin_x_=0.0f,
                         float origin_y_=0.0f,
                         std::string path_="",
                         std::string stat_xml_file_path_="",
                         std::string dyn_xml_file_path_="",
                         int timeout_=30,
                         bool wait_service_=false,
                         bool load_dyn_map_=true):
        _image_path(image_path_),
        _robotname(robotname_),
        _resolution(resolution_),
        _origin_x(origin_x_),
        _origin_y(origin_y_),
        _path(path_),
        _stat_xml_file_path(stat_xml_file_path_),
        _dyn_xml_file_path(dyn_xml_file_path_),
        _timeout(timeout_),
        _wait_service(wait_service_),
        _load_dyn_map(load_dyn_map_){

        //load rooms and objects files
        std::ifstream ifile1(std::string(_stat_xml_file_path).c_str());
        std::ifstream ifile2(std::string(_dyn_xml_file_path).c_str());

        if (!_stat_xml_file_path.empty() && ifile1)
            loadXMLMap(_stat_xml_file_path);

        if (!_dyn_xml_file_path.empty() && ifile2 && _load_dyn_map)
            loadXMLMap(_dyn_xml_file_path);

        //load map filev
        cv::Mat src;

        src = cv::imread(_image_path);

        if( !src.data )
            std::cerr << "Failed to load image!" << std::endl;

        cv::cvtColor(src, _image, CV_BGR2GRAY);
        std::cerr << "Loaded RGB image " << _image.rows << "x" << _image.cols << std::endl;

        _grid_map = NULL;
        _semantic_matrix = NULL;
        _room_map = NULL;

    }

    void setRobotname(const std::string& robotname_){_robotname = robotname_;}
    const std::string& robotname(){return _robotname;}

    void setResolution(float resolution_){_resolution = resolution_;}
    float resolution(){return _resolution;}

    void setOriginX(float origin_x_){_origin_x = origin_x_;}
    float originX(){return _origin_x;}

    void setOriginY(float origin_y){_origin_y = origin_y;}
    float originY(){return _origin_y;}

    void setPath(const std::string& path_){_path = path_;}
    const std::string& path(){return _path;}

    void setStatXMLFilePath(const std::string& stat_xml_file_path_){_stat_xml_file_path = stat_xml_file_path_;}
    const std::string& statXMLFilePath(){return _stat_xml_file_path;}

    void setDynXMLFilePath(const std::string& dyn_xml_file_path_){_dyn_xml_file_path = dyn_xml_file_path_;}
    const std::string& dynXMLFilePath(){return _dyn_xml_file_path;}

    void buildSemanticMatrix();

    void showSemanticMatrix();
private:
    std::string _image_path;
    std::string _robotname;
    float _resolution;
    float _origin_x;
    float _origin_y;
    std::string _path;
    std::string _stat_xml_file_path;
    std::string _dyn_xml_file_path;
    int _timeout;
    bool _wait_service;
    bool _load_dyn_map;

    std::vector<std::pair<std::string, cv::Point> > _tags;
    std::vector<std::pair<std::string, cv::Point> > _imposed_tags;
    std::vector<bool> _static_tags;
    std::vector<Object> _objects;
    std::vector<bool> _static_objects;

    cv::Mat _image;
    GridMap* _grid_map;
    SemanticMatrix* _semantic_matrix;
    RoomMap* _room_map;

    void loadXMLMap(std::string XMLFilePath);
};
