#pragma once
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <math.h>
#include <string.h>


#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <pcl/console/print.h>

#define PCL_NO_PRECOMPILE
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/principal_curvatures.h>
//#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/io/file_io.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/impl/common.hpp>
//#include <pcl/features/impl/fpfh_omp.hpp>
//#include <pcl/features/impl/fpfh.hpp>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/impl/min_cut_segmentation.hpp>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



//
//

struct MyPointType
{
	PCL_ADD_POINT4D;  
	PCL_ADD_NORMAL4D;
	union
	{
		float rgb;
		struct
		{
			uint8_t r;
			uint8_t g;
			uint8_t b;
		};
	};
	float u,v;
	union
	{
		struct
		{
			float vx;
			float vy;
			float vz;
		};
		float data_c[4];
	};
	union
	{
		struct
		{
			union
			{
				float curvaturess;
				float curvatures;
				float curvature;
			};
		};
		float data_c[4];
	};
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} 
EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
								   (float, rgb, rgb)
                                   (float, u, u)
								   (float, v, v)
								   (float, vx, vx)
								   (float, vy, vy)
								   (float, vz, vz)
								   (float, normal_x, normal_x)
								   (float, normal_y, normal_y)
								   (float, normal_z, normal_z)
								   (float, curvatures, curvaturess)
)

typedef std::pair<std::string, std::vector<float> > vfh_model;
typedef std::pair<std::string, pcl::PointCloud<MyPointType>::Ptr > benanntePointCloud;

class model3d
{
private:
	pcl::PointXYZ lastClickedPoint;
	pcl::PointCloud<MyPointType>::Ptr unfilteredCloud;
	pcl::PointCloud<MyPointType>::Ptr filteredCloud;
	pcl::PointCloud<MyPointType>::Ptr activeCluster;
	
	pcl::PointCloud<pcl::VFHSignature308>::Ptr globalFeaturesOfactiveCluster;

	std::vector < benanntePointCloud > gefundeneElemente;
	std::vector < pcl::PointCloud<MyPointType>::Ptr > detectedPlanes;
	std::vector < pcl::PointCloud<MyPointType>::Ptr > detectedClusters;

public:
	
	void setUnfilteredCloud(const std::string path_to_pcl_file);
	void setUnfilteredCloud(pcl::PointCloud<MyPointType>::Ptr cloud);
	void model3d::viewCloud(pcl::PointCloud<MyPointType>::Ptr cloud, const std::string windowName = "Cloud", bool withNormals = false);
	static void model3d::changeColorOfCloud(pcl::PointCloud<MyPointType>::Ptr cloud, uint8_t color = 255, int colortype = 1);
	void filterCloud(float passfilterRange = .7000, float downsampleLeafSize = 0.001);
	void segmentationPlaneRansac(float dickeEbene = 0.01, float anteilEbene = 0.1);
	void segmentationMinCut();
	void segmentationRegionGrow();
	void segmentationSupervoxel();
	void globaleFeatures( pcl::PointCloud<MyPointType>::Ptr cloud);
	void calcOrganizedNormals(pcl::PointCloud<MyPointType>::Ptr cloud, float radiusSearch=0.001);
	void calcUnorganizedNormals(pcl::PointCloud<MyPointType>::Ptr cloud, float radiusSearch=0.001);
	void calcNormals(pcl::PointCloud<MyPointType>::Ptr cloud, float radiusSearch=0.001);
	void calcGlobalFeatures(pcl::PointCloud<MyPointType>::Ptr cloud);
	void calcPose(float leaf=0.005);
	void saveTrainingData(std::string nameOrdner = "Ordnername" ,std::string nameModel = "Modellname", std::string nummerCluster = "0");
	void training();
	void suche(int objektSuchradius = 9);
	void uiMain();
	void uiAnzeige();
	void uiClusterwahl();
	model3d();
	~model3d(void);
};

