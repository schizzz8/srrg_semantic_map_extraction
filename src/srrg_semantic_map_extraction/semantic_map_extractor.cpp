#include "semantic_map_extractor.h"

using namespace std;

void SemanticMapExtractor::buildSemanticMatrix(){
    _grid_map = new GridMap(_image);

    _grid_map->produce();

    float accumulator;
    vector<float> vertical_distances = _grid_map->getVerticalDistances();
    vector<float> horizontal_distances = _grid_map->getHorizontalDistances();

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

    _semantic_matrix = new SemanticMatrix(vertical_distances.size(), horizontal_distances.size());

    _room_map = new RoomMap(_image);

    std::vector<std::pair<cv::Point2i, cv::Point2i> > doors_to_process;

    for (std::vector<Object>::iterator i = _objects.begin(); i != _objects.end(); i++)
        if (i->getType() == DOOR) {
            std::vector<cv::Point2i> vert = i->getVertexCoords();
            doors_to_process.push_back(std::make_pair(vert[0], vert[1]));
        }

    _room_map->addDoors(doors_to_process);

    std::vector<cv::Point> tag_coords;
    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator it = _tags.begin(); it != _tags.end(); it++)
        tag_coords.push_back((*it).second);


    _room_map->extractRooms(tag_coords);

    try {
        _semantic_matrix->setRoomTags(_tags, *_room_map, horizontal_cell_centers, vertical_cell_centers);
    }
    catch(char const* str) {
        ROS_ERROR("%s", str);
    }


    for (std::vector<std::pair<std::string, cv::Point> >::const_iterator it = _imposed_tags.begin(); it != _imposed_tags.end(); it++) {
        try {
            (_semantic_matrix->getCellByIndexes(
                        it->second.y,
                        it->second.x))->setRoomTag(it->first);
        }
        catch(char const* str) {
            ROS_ERROR("%s", str);
        }
    }


}

void SemanticMapExtractor::showSemanticMatrix(){
    _semantic_matrix->showMatrixImage();
    cv::waitKey();
}

void SemanticMapExtractor::loadXMLMap(std::string XMLFilePath) {
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
                _tags.push_back(std::make_pair(string(tag), cv::Point(x, y)));

                if (XMLFilePath.compare(_stat_xml_file_path) == 0)
                    _static_tags.push_back(true);
                else
                    _static_tags.push_back(false);
            }
        }

        elem = RootHandle.FirstChild("imposed_areas").FirstChild().Element();

        for (; elem; elem = elem->NextSiblingElement()) {
            float x = 0, y = 0;
            const char *tag = elem->Attribute("name");
            elem->QueryFloatAttribute("x", &x);
            elem->QueryFloatAttribute("y", &y);

            if (tag)
                _imposed_tags.push_back(
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
                _objects.push_back(
                            Object(obj,
                                   cv::Point2i(x, y),
                                   theta,
                                   cv::Vec3f(dimX, dimY, dimZ),
                                   properties,
                                   (type) object_type));

                if (XMLFilePath.compare(_stat_xml_file_path) == 0)
                    _static_objects.push_back(true);
                else
                    _static_objects.push_back(false);
            }
        }
    } else {
        ROS_ERROR("No XML file found: doesn't matter, going on...");
    }
}

