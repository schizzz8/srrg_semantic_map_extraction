#include <iostream>
#include <string>
#include <tinyxml.h>
#include <fstream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "srrg_semantic_map_extraction/grid_map.h"
#include "srrg_semantic_map_extraction/semantic_matrix.h"

using namespace std;

string robotname="";
float resolution;
float origin_x;
float origin_y;
string path="";
string StatXMLFilePath="/home/dede/datasets/semantic_acquisition_test/mapXMLfilesimulated_dataset.xml";
string DynXMLFilePath="/home/dede/datasets/semantic_acquisition_test/augmentedMapXMLfilesimulated_dataset.xml";
int timeout=30;
bool wait_service=false;
bool load_dyn_map=true;

vector<std::pair<string, cv::Point> > tags;
vector<std::pair<string, cv::Point> > imposed_tags;
vector<bool> static_tags;
vector<Object> objects;
vector<bool> static_objects;

void loadXMLMap(std::string XMLFilePath);

int main(int argc, char** argv){

    //load rooms and objects files
    std::ifstream ifile1(std::string(StatXMLFilePath).c_str());
    std::ifstream ifile2(std::string(DynXMLFilePath).c_str());

    if (!StatXMLFilePath.empty() && ifile1)
        loadXMLMap(StatXMLFilePath);

    if (!DynXMLFilePath.empty() && ifile2 && load_dyn_map)
        loadXMLMap(DynXMLFilePath);

    //load map file
    cv::Mat src, src_gray;

    src = cv::imread(argv[1]);

    if( !src.data )
        return -1;

    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    int rows=src_gray.rows;
    int cols=src_gray.cols;
    cerr << "Loaded RGB image " << rows << "x" << cols << endl;

    resolution = 0.05f;
    origin_x = -23.4f;
    origin_y = -12.2f;

    GridMap* grid_map = new GridMap(src_gray);
    grid_map->produce();

    float accumulator;
    vector<float> vertical_distances = grid_map->getVerticalDistances();
    vector<float> horizontal_distances = grid_map->getHorizontalDistances();

    vector<float> horizontal_cell_centers;
    vector<float> vertical_cell_centers;

    for (vector<float>::iterator i = horizontal_distances.begin(); i != horizontal_distances.end(); i++) {
        if (i == horizontal_distances.begin())
            accumulator = (*i)/2;
        else
            accumulator += *i;

        horizontal_cell_centers.push_back(accumulator);
    }

    for (std::vector<float>::iterator i = vertical_distances.begin(); i != vertical_distances.end(); i++) {
        if (i == vertical_distances.begin())
            accumulator = (*i)/2;
        else
            accumulator += *i;

        vertical_cell_centers.push_back(accumulator);
    }

    SemanticMatrix* semantic_matrix = new SemanticMatrix(vertical_distances.size(), horizontal_distances.size());
    cerr << "Built new semantic matrix!" << endl;

    //adding rooms to the semantic matrix
    RoomMap* room_map = new RoomMap(src_gray);

    std::vector<std::pair<cv::Point2i, cv::Point2i> > doors_to_process;

    for (std::vector<Object>::iterator i = objects.begin(); i != objects.end(); i++)
        if (i->getType() == DOOR) {
            std::vector<cv::Point2i> vert = i->getVertexCoords();
            doors_to_process.push_back(std::make_pair(vert[0], vert[1]));
        }

    room_map->addDoors(doors_to_process);

    std::vector<cv::Point> tag_coords;
    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator it = tags.begin(); it != tags.end(); it++)
        tag_coords.push_back((*it).second);


    room_map->extractRooms(tag_coords);

    try {
        semantic_matrix->setRoomTags(tags, *room_map, horizontal_cell_centers, vertical_cell_centers);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }


    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator it = imposed_tags.begin(); it != imposed_tags.end(); it++) {
        try {
            (semantic_matrix->getCellByIndexes(
                        it->second.y,
                        it->second.x))->setRoomTag(it->first);
        }
        catch(char const* str) {
            ROS_ERROR("%s", str);
        }
    }


    //adding objects to the semantic matrix
    std::vector<std::vector<cv::Point2i> > cell_objects;

    for (std::vector<Object>::iterator i = objects.begin(); i != objects.end(); i++) {
        std::vector<cv::Point2i> vert = i->getVertexCoords();

        std::vector<cv::Point2i> cell;
        for (std::vector<cv::Point2i>::iterator it = vert.begin(); it != vert.end(); it++)
            cell.push_back(grid_map->getCellFromImageCoords(*it));

        cell_objects.push_back(cell);
    }

    try {
        semantic_matrix->setObjectTags(cell_objects, objects);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }

    semantic_matrix->showMatrixImage();
    cv::waitKey();

    return 0;
}

void loadXMLMap(std::string XMLFilePath) {
    TiXmlDocument xml(XMLFilePath);

    if (xml.LoadFile()) {
        ROS_INFO("XML file %s succesfully loaded", XMLFilePath.c_str());

        TiXmlHandle XMLHandle(&xml);
        TiXmlElement* elem;
        TiXmlHandle RootHandle(0);

        RootHandle = XMLHandle.FirstChildElement().Element();
        elem = RootHandle.FirstChild("areas").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0;
            const char *tag = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);

            if (tag) {
                tags.push_back(std::make_pair(string(tag), cv::Point(x, y)));

                if (XMLFilePath.compare(StatXMLFilePath) == 0)
                    static_tags.push_back(true);
                else
                    static_tags.push_back(false);
            }
        }

        elem = RootHandle.FirstChild("imposed_areas").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0;
            const char *tag = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);

            if (tag)
                imposed_tags.push_back(
                            std::make_pair(string(tag), cv::Point(x, y)));
        }

        elem = RootHandle.FirstChild("objects").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0, theta = 0, dimX = 0, dimY = 0, dimZ = 0;
            int object_type = 0;
            const char *obj = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);
            elem->QueryFloatAttribute("theta", &theta);
            elem->QueryFloatAttribute("dimX", &dimX);
            elem->QueryFloatAttribute("dimY", &dimY);
            elem->QueryFloatAttribute("dimZ", &dimZ);
            elem->QueryIntAttribute("type", &object_type);
            const char *properties = elem->Attribute("properties");

            if (obj && properties) {
                objects.push_back(
                            Object(obj,
                                   cv::Point2i(x, y),
                                   theta,
                                   cv::Vec3f(dimX, dimY, dimZ),
                                   properties,
                                   (type) object_type));

                if (XMLFilePath.compare(StatXMLFilePath) == 0)
                    static_objects.push_back(true);
                else
                    static_objects.push_back(false);
            }
        }
    } else {
        ROS_ERROR("No XML file found: doesn't matter, going on...");
    }
}

