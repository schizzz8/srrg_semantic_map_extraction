#include <iostream>
#include <string>
#include <math.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "srrg_semantic_map_extraction/semantic_map_extractor.h"

#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_types/cloud_3d.h"

#include "srrg_semantic_map_extraction/structure_analyzer.h"

#include "srrg_path_map/path_map.h"
#include "srrg_path_map/dijkstra_path_search.h"
#include "srrg_path_map/distance_map_path_search.h"
#include "srrg_path_map/clusterer_path_search.h"

using namespace std;
using namespace srrg_core;

string robotname="";
float resolution=0.05f;
float origin_x=-23.4f;
float origin_y=-12.2f;
string path="";
string StatXMLFilePath="/home/dede/datasets/semantic_acquisition_test/mapXMLfilesimulated_dataset.xml";
string DynXMLFilePath="/home/dede/datasets/semantic_acquisition_test/augmentedMapXMLfilesimulated_dataset.xml";
int timeout=30;
bool wait_service=false;
bool load_dyn_map=true;

int main(int argc, char** argv){
    SemanticMapExtractor* extractor = new SemanticMapExtractor(argv[1],
            robotname,
            resolution,
            origin_x,
            origin_y,
            path,
            StatXMLFilePath,
            DynXMLFilePath);

    extractor->buildSemanticMatrix();

    extractor->showSemanticMatrix();

    int map_rows = extractor->rows();
    int map_cols = extractor->cols();

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

    Eigen::Isometry3f offset = Eigen::Isometry3f::Identity();
    offset.translate(Eigen::Vector3f(-0.087,0.0475,1.5));
    offset.rotate(Eigen::Quaternionf(0.5,-0.5,0.5,-0.5));

    Eigen::Isometry3f odometry = Eigen::Isometry3f::Identity();

    Eigen::Vector3f origin (origin_x,origin_y,0);


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
            int depth_rows=depth_image.rows;
            int depth_cols=depth_image.cols;
            cerr << "Image size: " << depth_rows << "x" << depth_cols << endl;

            Cloud3D* points = new Cloud3D;
            points->resize(depth_rows*depth_cols);
            float raw_depth_scale = 0.001f;//pinhole_image_msg->depthScale();
            //cerr << "Depth scale: " << raw_depth_scale << endl;
            // generate the points applying the inverse depth model
            //cerr << "Camera matrix: " << pinhole_image_msg->cameraMatrix() << endl;
            //cerr << "Sensor pose: " << offset.translation() << endl << offset.rotation() << endl;

            for (int r=0; r<depth_rows; r++) {
                const unsigned short* id_ptr  = depth_image.ptr<unsigned short>(r);
                for (int c=0; c<depth_cols; c++) {
                    unsigned short id = *id_ptr;
                    float d = id * raw_depth_scale;
                    Eigen::Vector3f camera_point = iK * Eigen::Vector3f(c*d,r*d,d);
                    Eigen::Vector3f world_point = offset * camera_point;
                    points->at(c+r*depth_cols) = RichPoint3D(world_point);
                    //points->at(c+r*cols) = RichPoint3D(camera_point);
                    id_ptr++;
                }
            }
            cerr << "Point cloud size: " << points->size() << endl;

            std::ofstream outfile;
            outfile.open("depth.cloud");
            points->write(outfile);
            outfile.close();

            StructureAnalyzer analyzer;
            analyzer.compute(points);

            cv::Mat classified = analyzer.classified().clone();
            int classified_rows = classified.rows;
            int classified_cols = classified.cols;
            IntImage regions;
            regions.create(classified_rows,classified_cols);
            for (int r=0;r<classified_rows; ++r) {
                int* regions_ptr=regions.ptr<int>(r);
                const uchar* src_ptr=classified.ptr<const uchar>(r);
                for (int c=0;c<classified_cols; ++c, ++src_ptr, ++ regions_ptr){
                    *regions_ptr = (*src_ptr==255) ? 0 : -1;
                }
            }

            robot_to_map = odom_to_map*odometry;

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
            //cv::cvtColor(extractor->semanticMatrix()->matrix().clone(),output,CV_GRAY2RGB);
            int cluster_idx=0;

            cerr << "Output image size: " << output.rows << "x" << output.cols << endl;

            cv::Mat occupancy;
            cv::cvtColor(extractor->image().clone(),occupancy,CV_GRAY2RGB);
            //for (const ClustererPathSearch::Cluster cluster: clusters)
            for(int i=0; i<clusters.size(); i++){

                ClustererPathSearch::Cluster cluster = clusters[i];

                Eigen::Vector3f a_w (cluster.lower.y()*resolution + analyzer.lower().x(),
                                     (classified_rows-cluster.upper.x()-1)*resolution + analyzer.lower().y(),
                                     0);
                Eigen::Vector3f d_w (cluster.upper.y()*resolution + analyzer.lower().x(),
                                     (classified_rows-cluster.lower.x()-1)*resolution + analyzer.lower().y(),
                                     0);

                Eigen::Vector3f a_g = (a_w - origin)/resolution;
                Eigen::Vector3f d_g = (d_w - origin)/resolution;

                cv::Point2i a_gp (a_g.x(),map_rows-a_g.y()-1);
                cv::Point2i d_gp (d_g.x(),map_rows-d_g.y()-1);

                cerr << "a_g: " << a_gp << endl;
                cerr << "d_g: " << d_gp << endl;

                cv::rectangle(occupancy,
                              a_gp,
                              d_gp,
                              cv::Scalar(255,0,0),
                              1);


                cv::Point2i a_cell = extractor->gridMap()->getCellFromImageCoords(a_gp);
                cv::Point2i d_cell = extractor->gridMap()->getCellFromImageCoords(d_gp);

                cerr << "a_cell: " << a_cell << endl;
                cerr << "d_cell: " << d_cell << endl;

                cv::rectangle(output,
                              a_cell,
                              d_cell,
                              cv::Scalar(255,0,0),
                              1);
                cluster_idx++;
            }



            cv::imshow("classified",occupancy);
            cv::waitKey();

        }

    }

    return 0;
}
