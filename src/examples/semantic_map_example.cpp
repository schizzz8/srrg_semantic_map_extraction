#include <iostream>
#include <string>
#include <math.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "srrg_semantic_map_extraction/semantic_map_extractor.h"

#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_messages/sensor_message_sorter.h"
#include "srrg_types/cloud_3d.h"

#include "srrg_semantic_map_extraction/structure_analyzer.h"

#include "srrg_path_map/path_map.h"
#include "srrg_path_map/dijkstra_path_search.h"
#include "srrg_path_map/distance_map_path_search.h"
#include "srrg_path_map/clusterer_path_search.h"

using namespace std;
using namespace srrg_core;

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

int main(int argc, char** argv){
    SemanticMapExtractor* extractor = new SemanticMapExtractor(argv[1],
            robotname,
            0.5f,
            -23.4f,
            -12.2f,
            path,
            StatXMLFilePath,
            DynXMLFilePath);

    extractor->buildSemanticMatrix();

    //extractor->showSemanticMatrix();

    MessageReader reader;
    reader.open(argv[2]);

    Eigen::Isometry3f odom_to_map = Eigen::Isometry3f::Identity();
    odom_to_map.translate(Eigen::Vector3f(0.059,0.12,0.0f));
    odom_to_map.rotate(Eigen::AngleAxisf(-0.023,Eigen::Vector3f::UnitZ()));
    Eigen::Isometry3f robot_to_map = Eigen::Isometry3f::Identity();

    cv::namedWindow("classified",CV_WINDOW_NORMAL);

    cerr << endl;

    Eigen::Matrix3f camera_matrix;
    camera_matrix << 277.127,0,160.5,
            0,277.127,120.5,
            0,0,1;
    Eigen::Matrix3f iK= camera_matrix.inverse();

    BaseMessage* msg = 0;
    while ((msg = reader.readMessage())) {
        msg->untaint();
        BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(msg);
        if (sensor_msg) {
            cerr << "[INFO] Reading message of type: " << sensor_msg->tag() << endl;
        }
        PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
        if (pinhole_image_msg) {
            cv::Mat depth_image = pinhole_image_msg->image().clone();
            int rows=depth_image.rows;
            int cols=depth_image.cols;
            cerr << "Image size: " << rows << "x" << cols << endl;

            cv::imshow("depth_image",depth_image);
            cv::waitKey();

            Cloud3D* points = new Cloud3D;
            points->resize(rows*cols);
            float raw_depth_scale = 0.001f;//pinhole_image_msg->depthScale();
            Eigen::Isometry3f offset = pinhole_image_msg->offset();
            cerr << "Depth scale: " << raw_depth_scale << endl;

            // generate the points applying the inverse depth model

            cerr << "Camera matrix: " << pinhole_image_msg->cameraMatrix() << endl;

            cerr << "Sensor pose: " << offset.translation() << endl << offset.rotation() << endl;

            for (int r=0; r<depth_image.rows; r++) {
                const unsigned short* id_ptr  = depth_image.ptr<unsigned short>(r);
                for (int c=0; c<depth_image.cols; c++) {
                    unsigned short id = *id_ptr;
                    float d = id * raw_depth_scale;
                    Eigen::Vector3f camera_point = iK * Eigen::Vector3f(c*d,r*d,d);
                    Eigen::Vector3f world_point = offset * camera_point;
                    points->at(c+r*cols) = RichPoint3D(world_point);
                    id_ptr++;
                }
            }
            cerr << "Point cloud size: " << points->size() << endl;
            StructureAnalyzer analyzer;
            analyzer.compute(points);

            cv::Mat classified = analyzer.classified().clone();
            IntImage regions;
            regions.create(classified.rows,classified.cols);
            for (int r=0;r<classified.rows; ++r) {
                int* regions_ptr=regions.ptr<int>(r);
                const uchar* src_ptr=classified.ptr<const uchar>(r);
                for (int c=0;c<classified.cols; ++c, ++src_ptr, ++ regions_ptr){
                    *regions_ptr = (*src_ptr==255) ? 0 : -1;
                }
            }

            robot_to_map = odom_to_map*pinhole_image_msg->odometry();

            ClustererPathSearch clusterer;
            PathMap  output_map;
            clusterer.setOutputPathMap(output_map);
            clusterer.setRegionsImage(regions);
            clusterer.init();
            clusterer.compute();
            ClustererPathSearch::ClusterVector clusters = clusterer.clusters();
            cerr << "Found " << clusters.size() << " clusters" << endl;

            cv::Mat output;
            //cv::cvtColor(classified,output,CV_GRAY2RGB);
            output = extractor->semanticMatrix()->matrix().clone();
            int cluster_idx=0;
            for (const ClustererPathSearch::Cluster cluster: clusters) {

                Eigen::Vector3f a_w = points->at(analyzer.indices().at<int>(cluster.lower.x(),cluster.lower.y())).point();
                Eigen::Vector3f b_w = points->at(analyzer.indices().at<int>(cluster.lower.x(),cluster.upper.y())).point();
                Eigen::Vector3f c_w = points->at(analyzer.indices().at<int>(cluster.upper.x(),cluster.lower.y())).point();
                Eigen::Vector3f d_w = points->at(analyzer.indices().at<int>(cluster.upper.x(),cluster.upper.y())).point();

                a_w = robot_to_map*a_w;
                b_w = robot_to_map*b_w;
                c_w = robot_to_map*c_w;
                d_w = robot_to_map*d_w;

                cv::Point2i a_g = cv::Point2i(a_w.x(),a_w.y());
                cv::Point2i b_g = cv::Point2i(b_w.x(),b_w.y());
                cv::Point2i c_g = cv::Point2i(c_w.x(),c_w.y());
                cv::Point2i d_g = cv::Point2i(d_w.x(),d_w.y());

                cv::Point2i a_cell = extractor->gridMap()->getCellFromImageCoords(a_g);
                cv::Point2i b_cell = extractor->gridMap()->getCellFromImageCoords(b_g);
                cv::Point2i c_cell = extractor->gridMap()->getCellFromImageCoords(c_g);
                cv::Point2i d_cell = extractor->gridMap()->getCellFromImageCoords(d_g);

                cv::rectangle(output,
                              a_cell,
                              b_cell,
                              cv::Scalar(255,0,0),
                              1);
                cluster_idx++;
            }

            std::ofstream outfile;
            outfile.open("depth.cloud");
            points->write(outfile);
            outfile.close();

            cv::imshow("obstacles",output);
            cv::waitKey();

        }

    }

    return 0;
}
