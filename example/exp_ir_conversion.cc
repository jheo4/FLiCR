#include <flicr>
#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/filter.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

using namespace std;
using namespace flicr;


int main(int argc, char** argv) {
  cxxopts::Options options("FLiCR", "FLiCR");
  options.add_options()
    ("y, yaml", "YAML file", cxxopts::value<std::string>())
    ("h, help", "Print usage")
    ;

  auto parsedArgs = options.parse(argc, argv);
  if(parsedArgs.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  std::string yamlConfig;
  if(parsedArgs.count("yaml"))
  {
    yamlConfig = parsedArgs["yaml"].as<std::string>();
    std::cout << "YAML Config: " << yamlConfig << std::endl;
  }
  else
  {
    std::cout << "Invalid YAML Config" << std::endl;
    exit(0);
  }

  double st, et;

  YAML::Node config = YAML::LoadFile(yamlConfig);
  std::string lidarDataPath = config["lidar_data"].as<std::string>();
  std::string dataCategory  = config["data_cat"].as<std::string>();

  int riRow, riCol;
  int numScans = 100;

  riRow = (int)(HDL64_VERTICAL_DEGREE   / HDL64_THETA_PRECISION);
  riCol = (int)(HDL64_HORIZONTAL_DEGREE / HDL64_PI_PRECISION_4500);

  std::shared_ptr<spdlog::logger> pc2ri     = spdlog::basic_logger_st("ri_logger", "ir_logs/ri.log");
  std::shared_ptr<spdlog::logger> pc2rinp   = spdlog::basic_logger_st("rinp_logger", "ir_logs/ri_np.log");
  std::shared_ptr<spdlog::logger> pc2octree = spdlog::basic_logger_st("octree_logger", "ir_logs/octree.log");
  std::shared_ptr<spdlog::logger> pc2kdtree = spdlog::basic_logger_st("kdtree_logger", "ir_logs/kdtree.log");
  std::shared_ptr<spdlog::logger> pc2mesh   = spdlog::basic_logger_st("mesh_logger", "ir_logs/mesh.log");

  std::ostringstream os;
  PcReader pcReader;
  RiConverter riConverter(HDL64_MIN_RANGE, HDL64_MAX_RANGE,
                          HDL64_THETA_PRECISION, HDL64_PI_PRECISION_4500,
                          HDL64_VERTICAL_DEGREE, HDL64_HORIZONTAL_DEGREE,
                          HDL64_VERTICAL_DEGREE_OFFSET, HDL64_HORIZONTAL_DEGREE_OFFSET);

  for(int idx = 0; idx < numScans; idx++)
  {
    os << std::setw(10) << std::setfill('0') << idx;
    std::string fn = lidarDataPath + "/" + os.str() + ".bin";
    os.str(""); os.clear();

    types::PclPcXyz pcXyz = pcReader.readXyzFromXyziBin(fn);

    /* pc -> ri -> nRi -> yuv -> encoded bytes */
    cv::Mat ri;

    st = getTsNow();
    riConverter.convertPc2Ri(pcXyz, ri, true);
    et = getTsNow();
    pc2ri->info("Pc2Ri parallel exe\t{}", et-st);
    ri.release();

    st = getTsNow();
    riConverter.convertPc2Ri(pcXyz, ri, false);
    et = getTsNow();
    pc2rinp->info("Pc2Ri non-parallel exe\t{}", et-st);
    ri.release();

    st = getTsNow();
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(0.1);
    octree.setInputCloud(pcXyz);
    //octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    et = getTsNow();
    pc2octree->info("PC2Octree exe\t{}", et-st);

    st = getTsNow();
    pcl::KdTreeFLANN<types::PclXyz> kdtree;
    kdtree.setInputCloud(pcXyz);
    et = getTsNow();
    pc2kdtree->info("PC2kdtree exe\t{}", et-st);

    pcl::PolygonMeshPtr mesh = nullptr;
    st = getTsNow();
    pcl::NormalEstimation<types::PclXyz, pcl::Normal> normEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr norms(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<types::PclXyz>::Ptr kdTreeXyz(new pcl::search::KdTree<types::PclXyz>);

    // Get normal
    kdTreeXyz->setInputCloud(pcXyz);
    normEstimation.setInputCloud(pcXyz);
    normEstimation.setSearchMethod(kdTreeXyz);
    normEstimation.setKSearch(5);
    normEstimation.compute(*norms);

    pcl::PointCloud<pcl::PointNormal>::Ptr pcWithNorm(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*pcXyz, *norms, *pcWithNorm);

    pcl::search::KdTree<pcl::PointNormal>::Ptr kdTreeXyzNorm(new pcl::search::KdTree<pcl::PointNormal>);
    kdTreeXyzNorm->setInputCloud(pcWithNorm);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpTriangulation;
    pcl::PolygonMeshPtr tempMeshes(new pcl::PolygonMesh);
    mesh = tempMeshes;

    gpTriangulation.setSearchRadius(0.5);
    gpTriangulation.setMu(2.5);
    gpTriangulation.setMaximumNearestNeighbors(5);
    gpTriangulation.setMaximumSurfaceAngle(M_PI-1);
    gpTriangulation.setMinimumAngle(M_PI/90);
    gpTriangulation.setMaximumAngle(2*M_PI);
    gpTriangulation.setNormalConsistency(false);

    gpTriangulation.setInputCloud(pcWithNorm);
    gpTriangulation.setSearchMethod(kdTreeXyzNorm);
    gpTriangulation.reconstruct(*mesh);

    //std::vector<int> partID, pointStates;
    //partID = gpTriangulation.getPartIDs();
    //pointStates = gpTriangulation.getPointStates();

    et = getTsNow();
    pc2mesh->info("PC2Mesh exe\t{}", et-st);
    tempMeshes.reset();

    pcXyz->clear();

    printProgress((float)idx/(float)numScans);
  }
  printProgress(1);

  return 0;
}

