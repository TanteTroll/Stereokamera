#include "StdAfx.h"
#include "model3d.h"
#include "TwoPictureManipulation.h"
#include "Menue.h"
#include "Settings.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/point_picking_event.h>
#include <time.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>
#include <pcl/io/vtk_lib_io.h>


pcl::PointXYZ clickedPoint;

void addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
										pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
										std::string supervoxel_name,
										boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


void pp_callback(const pcl::visualization::PointPickingEvent & event, void* viewer_void )
{
	float pointX, pointY, pointZ, x, y, z;

	if ( event.getPointIndex() != -1)
	{
		event.getPoint(pointX,pointY,pointZ);

		x = clickedPoint.x;
		y = clickedPoint.y;
		z = clickedPoint.z;

		float abs= sqrt((x-pointX)*(x-pointX)+(y-pointY)*(y-pointY)+(z-pointZ)*(z-pointZ));
		std::cout<<"Abstand zum letzten Punkt: "<<abs/32*5<<std::endl;

		::clickedPoint.x = pointX;
		::clickedPoint.y = pointY;
		::clickedPoint.z = pointZ;
	}
	return;
}
bool loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
	int vfh_idx;
	// Load the file as a PCD
	try
	{
		pcl::PCLPointCloud2 cloud;
		int version;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		pcl::PCDReader r;
		int type; unsigned int idx;
		r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

		vfh_idx = pcl::getFieldIndex (cloud, "vfh");
		if (vfh_idx == -1)
		return (false);
		if ((int)cloud.width * cloud.height != 1)
		return (false);
	}
	catch (pcl::InvalidConversionException e)
	{
		return (false);
	}

	// Treat the VFH signature as a single Point Cloud
	pcl::PointCloud <pcl::VFHSignature308> point;
	pcl::io::loadPCDFile (path.string (), point);
	vfh.second.resize (308);

	std::vector <pcl::PCLPointField> fields;
	pcl::getFieldIndex (point, "vfh", fields);

	for (size_t i = 0; i < fields[vfh_idx].count; ++i)
	{
		vfh.second[i] = point.points[0].histogram[i];
	}
	vfh.first = path.string ();
	return (true);
}
inline void nearestKSearch (::flann::Index<::flann::ChiSquareDistance<float> > &index, const vfh_model &model, int k, ::flann::Matrix<int> &indices, ::flann::Matrix<float> &distances)
{
	// Query point
	::flann::Matrix<float> p = ::flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
	memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

	indices = ::flann::Matrix<int>(new int[k], 1, k);
	distances = ::flann::Matrix<float>(new float[k], 1, k);
	index.knnSearch (p, indices, distances, k, ::flann::SearchParams (512));
	delete[] p.ptr ();
}
bool loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}
void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<vfh_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      std::stringstream ss;
      ss << it->path ();
      pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
      loadFeatureModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      vfh_model m;
      if (loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}
model3d::model3d(void)
{
	activeCluster = pcl::PointCloud<MyPointType>::Ptr(new pcl::PointCloud<MyPointType>);
	unfilteredCloud = pcl::PointCloud<MyPointType>::Ptr(new pcl::PointCloud<MyPointType>);
	filteredCloud = pcl::PointCloud<MyPointType>::Ptr(new pcl::PointCloud<MyPointType>);
	globalFeaturesOfactiveCluster = pcl::PointCloud<pcl::VFHSignature308>::Ptr(new pcl::PointCloud<pcl::VFHSignature308>);
	this->lastClickedPoint.x = 0;
	this->lastClickedPoint.y = 0;
	this->lastClickedPoint.z = 0;
}
model3d::~model3d(void){}
void model3d::setUnfilteredCloud(const std::string path_to_pcl_file)
{
	pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
	if (pcl::io::loadPCDFile<MyPointType> (path_to_pcl_file, *cloud) == -1)
		pcl::console::print_error("Cloud nicht gefunden\n");
	else
		unfilteredCloud=cloud;
	return;
}
void model3d::setUnfilteredCloud(const pcl::PointCloud<MyPointType>::Ptr cloud)
{
	unfilteredCloud=cloud;
	return;
}
void model3d::viewCloud(pcl::PointCloud<MyPointType>::Ptr cloud, const std::string windowName, bool withNormals)
{
	pcl::visualization::PointCloudColorHandlerRGBField<MyPointType> rgb(cloud);
	pcl::visualization::PCLVisualizer viewer (windowName);
	viewer.setBackgroundColor(0.1,0.5,0.1);
	viewer.addPointCloud<MyPointType> (cloud, rgb, "scene_cloud");
	
	viewer.registerPointPickingCallback(pp_callback,(void*)&viewer);
	cout<<this->lastClickedPoint<<endl;
	viewer.addCoordinateSystem (0.1, "global", 0);
	if (withNormals)
		viewer.addPointCloudNormals<MyPointType,MyPointType> (cloud, cloud);

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce();
	}
	viewer.close();
	viewer.resetStoppedFlag();
	viewer.close();
	return;
}
void model3d::changeColorOfCloud(pcl::PointCloud<MyPointType>::Ptr cloud, uint8_t color, int colortype)
{
	color=color%256;
	for ( int i = 0; i < cloud->points.size () ; ++i )
	{
		switch (colortype)
		{
			case 1:
			{
				cloud->points[i].r = color;
				break;
			}
			case 2:
			{
				cloud->points[i].g = color;
				break;
			}
			default:
			{
				cloud->points[i].b = color;
				break;
			}
		}
	}
	return;
}
void model3d::filterCloud(float passfilterRange, float downsampleLeafSize)
{
	clock_t t;
	t = clock();
	pcl::console::print_highlight("Filterung!\n");
	pcl::PointCloud<MyPointType>::Ptr filteredCloud_tmp (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr filteredCloud_tmp2 (new pcl::PointCloud<MyPointType>);
	Menue::printOutput("PointCloud before filtering has:",unfilteredCloud->points.size (),"data points.");

	pcl::PassThrough<MyPointType> pass;
	pass.setInputCloud (unfilteredCloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, passfilterRange);
	pass.filter (*filteredCloud_tmp);

	pcl::VoxelGrid<MyPointType> sor;
	sor.setInputCloud (filteredCloud_tmp);
	sor.setLeafSize (downsampleLeafSize, downsampleLeafSize, downsampleLeafSize);
	sor.filter (*filteredCloud_tmp2);
	Menue::printOutput("PointCloud after filtering has:",filteredCloud_tmp2->points.size (),"data points.");
	this->filteredCloud = filteredCloud_tmp2;
	t = clock() - t;
	cout << "Filterung dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
}
void model3d::segmentationPlaneRansac(float dickeEbene, float anteilEbene)
{
	clock_t t;
	t = clock();
	pcl::console::print_highlight("Segmentation!\n");
	// Auswahl der Cloud, die bearbeiet werden soll
	pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr cloud_f (new pcl::PointCloud<MyPointType>);
	if ( this->filteredCloud -> points.size () < 100 )
		pcl::copyPointCloud( *this->unfilteredCloud, *cloud);
	else
		pcl::copyPointCloud( *this->filteredCloud, *cloud);
	if ( cloud -> points.size () < 100 )
	{
		pcl::console::print_error("Keine Input-Cloud vorhanden!\n");
		return;
	}
		
	// Segmentation für Ebene vorbereiten
	pcl::console::print_debug("Segmentation\n");
	pcl::SACSegmentation<MyPointType> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<MyPointType>::Ptr cloud_plane (new pcl::PointCloud<MyPointType>);
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
			//SAC_RANSAC - RANdom SAmple Consensus
			//SAC_LMEDS - Least Median of Squares
			//SAC_MSAC - M-Estimator SAmple Consensus
			//SAC_RRANSAC - Randomized RANSAC
			//SAC_RMSAC - Randomized MSAC
			//SAC_MLESAC - Maximum LikeLihood Estimation SAmple Consensus
			//SAC_PROSAC - PROgressive SAmple Consensus
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (dickeEbene);

	int i=0, nr_points = (int) cloud->points.size ();
	while (cloud->points.size () > ( 1 - anteilEbene ) * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			pcl::console::print_error ("Could not estimate a planar model for the given dataset.\n");
			break;
		}
						
		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<MyPointType> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		Menue::printOutput("PointCloud representing a planar component has",cloud_plane->points.size (),"data points.");
		this->detectedPlanes.push_back(cloud_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud = *cloud_f;
	}

	// Segmentation für Cluster vorbereiten
	pcl::search::KdTree<MyPointType>::Ptr tree (new pcl::search::KdTree<MyPointType>);
	tree->setInputCloud (cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<MyPointType> ec;
	ec.setClusterTolerance (.001); // .001 --> mm
	ec.setMinClusterSize (1000);   // Anzahl Punkte (minimal)
	ec.setMaxClusterSize (250000); // Anzahl Punkte maximal
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<MyPointType>::Ptr cloud_cluster (new pcl::PointCloud<MyPointType>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); 

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Menue::printOutput("PointCloud representing a Cluster has",cloud_cluster->points.size (),"data points.");
		this->detectedClusters.push_back(cloud_cluster);

		j++;
	}	
	t = clock() - t;
	cout << "Segmentierung dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
	return;

}
void model3d::segmentationMinCut()
{
	clock_t t;
	t = clock();
	pcl::console::print_highlight("Segmentation!\n");
	// Auswahl der Cloud, die bearbeiet werden soll
	pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr cloud_f (new pcl::PointCloud<MyPointType>);
	if ( this->filteredCloud -> points.size () < 100 )
		pcl::copyPointCloud( *this->unfilteredCloud, *cloud);
	else
		pcl::copyPointCloud( *this->filteredCloud, *cloud);
	if ( cloud -> points.size () < 100 )
	{
		pcl::console::print_error("Keine Input-Cloud vorhanden!\n");
		return;
	}
		
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<MyPointType> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::MinCutSegmentation<MyPointType> seg;
	seg.setInputCloud (cloud);
	seg.setIndices (indices);

	pcl::PointCloud<MyPointType>::Ptr foreground_points(new pcl::PointCloud<MyPointType> ());
	MyPointType point;
	point.x = 0;
	point.y = 0;
	point.z = 0.457;
	foreground_points->points.push_back(point);
	seg.setForegroundPoints (foreground_points);

	seg.setSigma (0.25);
	seg.setRadius (0.01);
	seg.setNumberOfNeighbours (14);
	seg.setSourceWeight (0.8);

	std::vector <pcl::PointIndices> clusters;
	seg.extract (clusters);

	std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}

	return;
}
void model3d::segmentationRegionGrow()
{
	clock_t t;
	t = clock();
	pcl::console::print_highlight("Segmentation!\n");
	// Auswahl der Cloud, die bearbeiet werden soll
	pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr cloud_f (new pcl::PointCloud<MyPointType>);
	if ( this->filteredCloud -> points.size () < 100 )
		pcl::copyPointCloud( *this->unfilteredCloud, *cloud);
	else
		pcl::copyPointCloud( *this->filteredCloud, *cloud);
	if ( cloud -> points.size () < 100 )
	{
		pcl::console::print_error("Keine Input-Cloud vorhanden!\n");
		return;
	}

	pcl::search::Search<MyPointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<MyPointType> > (new pcl::search::KdTree<MyPointType>);
	pcl::NormalEstimation<MyPointType, MyPointType> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*cloud);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<MyPointType> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::RegionGrowing<MyPointType, MyPointType> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
	//reg.setIndices (indices);
	reg.setInputNormals (cloud);
	reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (2.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}

	return;
}
void model3d::segmentationSupervoxel()
{
	clock_t t;
	t = clock();
	pcl::console::print_highlight("Segmentation!\n");
	// Auswahl der Cloud, die bearbeiet werden soll
	pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
	if ( this->filteredCloud -> points.size () < 100 )
		pcl::copyPointCloud( *this->unfilteredCloud, *cloud);
	else
		pcl::copyPointCloud( *this->filteredCloud, *cloud);
	if ( cloud -> points.size () < 100 )
	{
		pcl::console::print_error("Keine Input-Cloud vorhanden!\n");
		return;
	}

	for ( int i = 0; i < cloud->points.size (); ++i)
	{
		pcl::PointXYZRGBA point;
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].y;
		point.z = cloud->points[i].z;
		point.r = cloud->points[i].r;
		point.g = cloud->points[i].g;
		point.b = cloud->points[i].b;
		cloud_f->points.push_back(point);
	}


	bool use_transform = true;
	float voxel_resolution = 0.008f;
	bool voxel_res_specified = false;
	float seed_resolution = 0.1f;
	bool seed_res_specified = true;

	float color_importance = 0.2f;
	float spatial_importance = 0.4f;
	float normal_importance = 1.0f;

	//////////////////////////////  //////////////////////////////
	////// This is how to use supervoxels
	//////////////////////////////  //////////////////////////////

	pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution, use_transform);
	super.setInputCloud (cloud_f);
	super.setColorImportance (color_importance);
	super.setSpatialImportance (spatial_importance);
	super.setNormalImportance (normal_importance);

	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;

	pcl::console::print_highlight ("Extracting supervoxels!\n");
	super.extract (supervoxel_clusters);
	pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0.2, 0.6, 0.2);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
	viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_voxel_cloud = super.getColoredVoxelCloud ();
	viewer->addPointCloud (colored_voxel_cloud, "colored voxels");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "colored voxels");

	pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
	//We have this disabled so graph is easy to see, uncomment to see supervoxel normals
	//viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

	pcl::console::print_highlight ("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency (supervoxel_adjacency);
	//To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
	std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
	for ( ; label_itr != supervoxel_adjacency.end (); )
	{
		//First get the label
		uint32_t supervoxel_label = label_itr->first;
		//Now get the supervoxel corresponding to the label
		pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

		//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
		pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
		std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
		for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
		{
			pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
			adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
		}
		//Now we make a name for this polygon
		std::stringstream ss;
		ss << "supervoxel_" << supervoxel_label;
		//This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
		addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
		//Move iterator forward to next label
		label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
	}

	while (!viewer->wasStopped ())
	{
	viewer->spinOnce (100);
	}
	return;
}
void model3d::globaleFeatures(pcl::PointCloud<MyPointType>::Ptr cloud)
{
	clock_t t;
	t = clock();
	if ( cloud->points.size() < 100 )
	{
		pcl::console::print_error("Input-Cloud ist leer!\n");
		return;
	}
	Menue::printOutput("Anzahl Punkt im Cluster:", cloud->points.size() );
	t = clock() - t;
	cout << "Feature Berechnung dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
}
void model3d::calcOrganizedNormals(pcl::PointCloud<MyPointType>::Ptr cloud, float radiusSearch)
{
	if ( cloud->points.size() < 100 )
	{
		pcl::console::print_error("Input-Cloud ist leer!\n");
		return;
	}
	pcl::search::KdTree<MyPointType>::Ptr tree (new pcl::search::KdTree<MyPointType> ());
	pcl::NormalEstimationOMP <MyPointType, MyPointType> ne;
	ne.setInputCloud (cloud);
	//ne.setRadiusSearch (radiusSearch);
	ne.setKSearch(20);
	ne.setSearchMethod (tree);
	ne.compute (*cloud);
	return;
}
void model3d::calcUnorganizedNormals(pcl::PointCloud<MyPointType>::Ptr cloud, float radiusSearch)
{
	if ( cloud->points.size() < 100 )
	{
		pcl::console::print_error("Input-Cloud ist leer!\n");
		return;
	}
	pcl::search::KdTree<MyPointType>::Ptr tree (new pcl::search::KdTree<MyPointType> ());
	pcl::NormalEstimation <MyPointType, MyPointType> ne;
	ne.setInputCloud (cloud);
	//ne.setRadiusSearch (radiusSearch);
	ne.setKSearch(20);
	ne.setSearchMethod (tree);
	ne.compute (*cloud);
	return;
}
void model3d::calcNormals(pcl::PointCloud<MyPointType>::Ptr cloud, float radiusSearch)
{
	if ( cloud->isOrganized() == true )
		calcOrganizedNormals(cloud, radiusSearch);
	else 
		calcUnorganizedNormals(cloud, radiusSearch);
}
void model3d::calcGlobalFeatures(pcl::PointCloud<MyPointType>::Ptr cloud)
{
	clock_t t;
	t = clock();
	Menue::printOutput("Anzahl Punkt im Cluster:", cloud->points.size() );
	this->calcNormals(cloud);

	pcl::console::print_debug("Bestimme Featurepunkte \n");
	pcl::VFHEstimation<MyPointType, MyPointType, pcl::VFHSignature308> vfh;
	pcl::search::KdTree<MyPointType>::Ptr tree (new pcl::search::KdTree<MyPointType> ());
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (cloud);
	vfh.setSearchMethod (tree);
	vfh.compute (*vfhs);

	if ( vfhs->points.size () != 1 )
	{
		pcl::console::print_error("Featurebestimmung nicht erfolgreich. Size: %d \n", vfhs->points.size () );
		return;
	}
	else
	{
		pcl::console::print_debug("Featurebestimmung war erfolgreich \n" );
		globalFeaturesOfactiveCluster=vfhs;
	}	
	t = clock() - t;
	cout << "Feature Berechnung dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
	return;
}
void model3d::training()
{
	std::string extension (".pcd");
	std::string folder = "Featuremodels\\";
	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";
	transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

	pcl::console::print_debug("Lade Daten\n");
	std::vector<vfh_model> models;
	loadFeatureModels (folder, extension, models);

	pcl::console::print_color(stdout,2,4,"Hinweis: Bei Absturz sollte Trainingsdata geloescht werden!\n");
	pcl::console::print_debug("konvertiere Dateien\n");

	::flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
								
	for (size_t i = 0; i < data.rows; ++i)
		for (size_t j = 0; j < data.cols; ++j)
			data[i][j] = models[i].second[j];
	pcl::console::print_debug("Speichere konvertierte Daten\n");
	::flann::save_to_file (data, training_data_h5_file_name, "training_data");
	std::ofstream fs;
	fs.open (training_data_list_file_name.c_str ());
	for (size_t i = 0; i < models.size (); ++i)
	fs << models[i].first << "\n";
	fs.close ();

	pcl::console::print_debug("erstelle Index\n");
	pcl::console::print_info ("Building the kdtree index");
	pcl::console::print_value(" %s ",kdtree_idx_file_name.c_str ());
	pcl::console::print_info ("for");
	pcl::console::print_value( "%d ", (int)data.rows );
	pcl::console::print_info ("elements...\n");
	::flann::Index<::flann::ChiSquareDistance<float> > index (data, ::flann::LinearIndexParams ());		//genauere Nachbarsuche
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));		//schnellere Nachbarsuche
	index.buildIndex ();
	index.save (kdtree_idx_file_name);
	delete[] data.ptr ();
	return;
}
void model3d::calcPose(float leaf)
{
	clock_t t;
	t = clock();
	pcl::PointCloud<MyPointType>::Ptr object (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr scene (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr object_unfiltered (new pcl::PointCloud<MyPointType>);
	pcl::PointCloud<MyPointType>::Ptr scene_unfiltered (new pcl::PointCloud<MyPointType>);

	if ( gefundeneElemente.size() <= 0 ) 
	{
		pcl::console::print_error("keine Objekt gefunden\n");
		return;
	}

	for ( int clusterNumber = 0 ; clusterNumber < gefundeneElemente.size () ; ++clusterNumber)
	{
		Menue::printMenueEntrySmall(clusterNumber,"Objekt mit Abstand von",gefundeneElemente[clusterNumber].first.c_str(),"Punkten.");
	}		

	int userInput = -1;
	while (userInput < 0 || userInput > gefundeneElemente.size() )
		Menue::readValue(&userInput);
	object_unfiltered=gefundeneElemente[userInput].second;
	pcl::copyPointCloud(*this->activeCluster, *scene_unfiltered);
	if (scene_unfiltered->points.size() < 100 || object_unfiltered->points.size() < 100 )
	{
		pcl::console::print_error("keine Input Cloud vorhanden\n");
		return;
	}
	if (this->activeCluster->points.size() < 100 )
	{
		pcl::console::print_error("keine aktive Cloud ausgewaehlt\n");
		return;
	}

	// Downsample
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<MyPointType> grid;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object_unfiltered);
	grid.filter (*object);
	grid.setInputCloud (scene_unfiltered);
	grid.filter (*scene);
	scene=scene_unfiltered;
	object=object_unfiltered;

	//Feature berechnen
	pcl::console::print_highlight ("Feature fuer Objekt...\n");
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::FPFHEstimationOMP<MyPointType, MyPointType,pcl::FPFHSignature33> fest;
	fest.setKSearch (10);
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	pcl::console::print_highlight ("Feature fuer Szene...\n");
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);

	//allign
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::PointCloud<MyPointType>::Ptr object_aligned (new pcl::PointCloud<MyPointType>);
	pcl::SampleConsensusPrerejective<MyPointType,MyPointType,pcl::FPFHSignature33> align;
	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (10000);		// Number of RANSAC iterations
	align.setNumberOfSamples (3);			// Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (7);	// Number of nearest features to use
	align.setSimilarityThreshold (0.7f);	// Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f);		// Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}
				
	if (align.hasConverged ())
	{
		// Print results
		printf ("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
		
		t = clock() - t;
		cout << "Positionsberechnung dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;

		// Show alignment
		pcl::visualization::PointCloudColorHandlerRGBField<MyPointType> rgb(this->filteredCloud);
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud<MyPointType> (this->filteredCloud,rgb, "scene");
		visu.addPointCloud<MyPointType> (object_aligned, "object_aligned");
		while (!visu.wasStopped ())
		{
			visu.spinOnce();
		}
		visu.close();
		visu.resetStoppedFlag();
		visu.close();
	}
	else
	{
		pcl::console::print_error ("Alignment failed!\n");
	}
	return;
}
void model3d::saveTrainingData(std::string nameOrdner,std::string nameModel, std::string nummerCluster)
{
	std::string extension (".pcd");
	std::string folder = "Featuremodels\\";
	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";
	transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

	bool cancel = true;
	bool ONValid=true; bool MNValid=true; bool CNValid=true; bool FNValid=true;
	
	std::string pathname, filename_cloud,filename_features, completeFilename_cloud,	completeFilename_features;

	int input;
	do 
	{
		pathname=folder+nameOrdner+"\\";
		filename_cloud=nameModel+"_cluster_"+nummerCluster+"_nxyz.pcd";
		filename_features=nameModel+"_cluster_"+nummerCluster+"_nxyz_vfh.pcd";
		completeFilename_cloud=pathname+filename_cloud;
		completeFilename_features=pathname+filename_features;
								
		pcl::console::print_highlight("Name fuer Modell auswaehlen:\n");

		if (boost::filesystem::exists(completeFilename_features) ||
			boost::filesystem::exists(completeFilename_cloud) )
			pcl::console::print_warn("File ist bereits vorhanden. Wird ueberspeichert!\n");
								
		Menue::printMenueEntrySmall(0,"Ordnername",nameOrdner.c_str() );
		if(!boost::filesystem::windows_name(nameOrdner) )
		{
			pcl::console::print_error("Kein zulaessiger Name\n");
			ONValid=false;
		}else ONValid=true;

		Menue::printMenueEntrySmall(1,"Modellname",nameModel.c_str() );
		if(!boost::filesystem::windows_name(nameModel) )
		{
			pcl::console::print_error("Kein zulaessiger Name\n");
			MNValid=false;
		}else MNValid=true;

		Menue::printMenueEntrySmall(2,"Clusternummer",nummerCluster.c_str() );
		if(!boost::filesystem::windows_name(nummerCluster) )
		{
			pcl::console::print_error("Kein zulaessiger Name\n");
			CNValid=false;
		}else CNValid=true;

		if ( !boost::filesystem::exists( pathname ) )
		{
			pcl::console::print_error("Ausgewaehlter Ordner muss zunaechst erstellt werden!\n");
			FNValid=false;
		}else FNValid=true;

		if (ONValid&&MNValid&&CNValid&&FNValid)
		Menue::printMenueEntrySmall(10,"Hinzufuegen");

		Menue::printMenueEntrySmall(-1,"Abbrechen");
		
		Menue::readValue(&input);

		switch (input)
		{
		case  0:{cin>>nameOrdner;break;}
		case  1:{cin>>nameModel;break;}
		case  2:{cin>>nummerCluster;break;}
		case 10:
			{
				if (!ONValid||!MNValid||!CNValid||!FNValid)break;
				if (!boost::filesystem::exists("Cluster_Cloud.pcd") ||
					!boost::filesystem::exists("Cluster_Feature.pcd") )
				{
					pcl::console::print_error("File %s oder %s nicht vorhanden","Cluster_Feature.pcd","Cluster_Cloud.pcd\n");
					break;
				}

				Menue::printOutput("Saving Cloud to\n",completeFilename_cloud.c_str() );
				Menue::printOutput("Saving Features to\n",completeFilename_features.c_str() );

				pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
				pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);

				cout<<"Lade Files"<<endl;
				if (pcl::io::loadPCDFile<MyPointType> ("Cluster_Cloud.pcd", *cloud) == -1)
				{
					pcl::console::print_error("File %s konnte nicht geladen werden.","Cluster_Cloud.pcd");
					break;
				}
				if (pcl::io::loadPCDFile<pcl::VFHSignature308> ("Cluster_Feature.pcd", *vfhs) == -1)
				{
					pcl::console::print_error("File %s konnte nicht geladen werden.","Cluster_Feature.pcd");
					break;
				}
										
				cout<<"Speichere Files"<<endl;
				pcl::io::savePCDFileASCII (completeFilename_cloud, *cloud);
				pcl::io::savePCDFileASCII (completeFilename_features, *vfhs);
				break;
			}
		default : break;
		}
	}while (input >= 0);
	
	return;
}
void model3d::suche(int objektSuchradius)
{
	clock_t t;
	t = clock();
	std::string extension (".pcd");
	std::string folder = "Featuremodels\\";
	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";
	transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

	double thresh = DBL_MAX;
	int k = objektSuchradius;			
									
	gefundeneElemente.erase (gefundeneElemente.begin(),gefundeneElemente.end() );
	pcl::console::print_info( "Lade Histogramm ");
	pcl::console::print_value("Cluster_Feature.pcd\n");
	vfh_model histogram;
	loadHist ( "Cluster_Feature.pcd" , histogram );

	std::vector<vfh_model> models;
	::flann::Matrix<int> k_indices;
	::flann::Matrix<float> k_distances;
	::flann::Matrix<float> data;
									
	pcl::console::print_debug("Lade Modell-Daten\n");
	if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
	{
		pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
		training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
		return;
	}
	else
	{
		loadFileList (models, training_data_list_file_name);
		::flann::load_from_file (data, training_data_h5_file_name, "training_data");
		pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
		(int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	}
	pcl::console::print_debug("Lade KD-Tree Index\n");
	if (!boost::filesystem::exists (kdtree_idx_file_name))
	{
		pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
		return;
	}
	else
	{
		::flann::Index<::flann::ChiSquareDistance<float> > index (data, ::flann::SavedIndexParams ("kdtree.idx"));
		index.buildIndex ();
		nearestKSearch (index, histogram, k, k_indices, k_distances);
	}
	t = clock() - t;
	cout << "Objektsuche dauerte "<<((float)t)/CLOCKS_PER_SEC<<" Sekunden!"<<endl;
	pcl::console::print_debug("Output:\n");
	pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, "Feature.pcd");
	for (int i = 0; i < k; ++i)
		pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
		i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
	pcl::visualization::PCLVisualizer p ("VFH Cluster Classifier");
	int y_s = (int)floor (sqrt ((double)k));
	int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
	double x_step = (double)(1 / (double)x_s);
	double y_step = (double)(1 / (double)y_s);
	pcl::console::print_highlight ("Preparing to load "); 
	pcl::console::print_value ("%d", k); 
	pcl::console::print_info (" files ("); 
	pcl::console::print_value ("%d", x_s);    
	pcl::console::print_info ("x"); 
	pcl::console::print_value ("%d", y_s); 
	pcl::console::print_info (" / ");
	pcl::console::print_value ("%f", x_step); 
	pcl::console::print_info ("x"); 
	pcl::console::print_value ("%f", y_step); 
	pcl::console::print_info (")\n");
	int viewport = 0, l = 0, m = 0;

	for (int i = 0; i < k; ++i)
	{
		std::string cloud_name = models.at (k_indices[0][i]).first;
		boost::replace_last (cloud_name, "_vfh", "");
		p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
		l++;
		if (l >= x_s)
		{
			l = 0;
			m++;
		}

		pcl::PCLPointCloud2 cloud;
		pcl::console::print_highlight (stderr, "Loading "); pcl::console::print_value (stderr, "%s ", cloud_name.c_str ());
		if (pcl::io::loadPCDFile (cloud_name, cloud) == -1)
			break;
		// Convert from blob to PointCloud
		pcl::PointCloud<MyPointType> cloud_xyz;
		pcl::fromPCLPointCloud2 (cloud, cloud_xyz);

		if (cloud_xyz.points.size () == 0)
			break;

		pcl::console::print_info ("[done, "); 
		pcl::console::print_value ("%d", (int)cloud_xyz.points.size ()); 
		pcl::console::print_info (" points]\n");
		pcl::console::print_info ("Available dimensions: "); 
		pcl::console::print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

			// Demean the cloud
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (cloud_xyz, centroid);
		pcl::PointCloud<MyPointType>::Ptr cloud_xyz_demean (new pcl::PointCloud<MyPointType>);
		pcl::demeanPointCloud<MyPointType> (cloud_xyz, centroid, *cloud_xyz_demean);
		// Add to renderer*
		pcl::visualization::PointCloudColorHandlerRGBField<MyPointType> rgb(cloud_xyz_demean);
		p.addPointCloud (cloud_xyz_demean, rgb, cloud_name, viewport);
		benanntePointCloud cloud_gef;
		cloud_gef.second=cloud_xyz_demean;

		// Check if the model found is within our inlier tolerance
		std::stringstream ss;
		ss << k_distances[0][i];
		//if (k_distances[0][i] > thresh)
		//{
		//	p.addText (ss.str (), 20, 30, 1, 0, 0, ss.str (), viewport);  // display the text with red
		//	// Create a red line
		//		pcl::PointXYZ min_p, max_p;
		//		pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
		//		std::stringstream line_name;
		//		line_name << "line_" << i;
		//		p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
		//		p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);
		//}
		//else
			p.addText (ss.str (), 20, 30, 0, 1, 0, ss.str (), viewport);

			// Increase the font size for the score*
			p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str (), viewport);
			cloud_gef.first=ss.str();
			gefundeneElemente.push_back(cloud_gef);
			// Add the cluster name
			p.addText (cloud_name, 20, 10, cloud_name, viewport);
		}
		// Add coordianate systems to all viewports
		p.addCoordinateSystem (0.1, "global", 0);
										

		while (!p.wasStopped ())
		{
			p.spinOnce ();
		}
		p.close();
		p.resetStoppedFlag();
		p.close();
		return;
}
void model3d::uiMain()
{
	int input=0;
	this->filterCloud();
	this->segmentationPlaneRansac();
	while ( true )
	{
		
		Menue::printMenueEntrySmall(0,"Anzeige: ","Anzeigen von berechneten Punktwolken");
		Menue::printMenueEntrySmall(1,"Clusterauswahl:"," aktiven Cluster auswaehlen");
		Menue::printMenueEntrySmall(2,"Objekttraining:"," fuegt abgespeicherten Cluster der Datenbank bei");
		Menue::printMenueEntrySmall(3,"Objektsuche:"," sucht aktiven Cluster in Szene");
		Menue::printMenueEntrySmall(4,"Pose berechnen: ","vergleicht Objektsuche mit aktivem Cluster");
		Menue::readValue(&input);
		switch ( input )
		{
		case 0:
			{
				this->uiAnzeige();
				break;
			}
		case 1:
			{
				this->uiClusterwahl();
				this->calcGlobalFeatures(activeCluster);
				pcl::console::print_debug("Speichere Featurepunkte \n" );
				pcl::io::savePCDFileASCII ("Cluster_Feature.pcd", *this->globalFeaturesOfactiveCluster);
				pcl::console::print_debug("Speichere Pointcloud\n" );
				pcl::io::savePCDFileASCII ("Cluster_Cloud.pcd", *this->activeCluster);
				break;
			}
		case 2:
			{
				this->saveTrainingData();
				this->training();
				break;
			}
		case 3:
			{
				this->suche();
				break;
			}
		case 4:
			{
				this->calcPose();
				break;
			}
		default: break;
		}
	}
}
void model3d::uiAnzeige()
{
	int input=0;
	while ( input >= 0 )
	{
		Menue::printMenueEntry(-1,"Zurueck");
		Menue::printMenueEntry(0,"Szene anzeigen");
		if( this->activeCluster->points.size() > 100)
			Menue::printMenueEntry(1,"Aktiven Cluster anzeigen");
		Menue::printMenueEntry(2,"Segmentierung anzeigen");
		Menue::printMenueEntrySmall(3,"Testfunktion","Min Cut Segmentierung");
		Menue::printMenueEntrySmall(4,"Testfunktion","Region Grow Segmentierung");
		Menue::printMenueEntrySmall(5,"Testfunktion","Supervoxel Segmentierung");
		Menue::printMenueEntrySmall(6,"Testfunktion","Spline Fitting");
		Menue::readValue(&input);
		switch ( input )
		{
		case 0:
			{
				if (filteredCloud -> points.size() > 100 )
					this->viewCloud(this->filteredCloud, "Gefilterte Punktwolke");
				else
				{
					if (unfilteredCloud -> points.size() > 100 )
						this->viewCloud(this->unfilteredCloud, "Ungefilterte Punktwolke");
					else
					{
						pcl::console::print_error("Input-Cloud ist leer!\n");
						break;
					}
				}
				break;
			}
		case 1:
			{
				if (activeCluster -> points.size() > 100 )
					this->viewCloud(this->activeCluster,"Cluster",true);
				else
				{
					pcl::console::print_error("Input-Cloud ist leer!\n");
					break;
				}
				break;
			}
		case 2:
			{
				
				pcl::visualization::PCLVisualizer viewer ("Segmentation");
				viewer.addCoordinateSystem (0.1, "global", 0);
				
				pcl::visualization::PointCloudColorHandlerRGBField<MyPointType> rgb(this->filteredCloud);
				viewer.addPointCloud<MyPointType> (this->filteredCloud, rgb, "scene_cloud");
				
				for ( int i = 0; i < this->detectedClusters.size () ; ++i )
				{
					pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
					pcl::copyPointCloud( *this->detectedClusters[i],*cloud );
					model3d::changeColorOfCloud( cloud, (1+i)*73 );
					pcl::visualization::PointCloudColorHandlerRGBField<MyPointType> rgb(cloud);
					std::string cloudName = "Cluster" + std::to_string(static_cast<long long>(i));
					viewer.addPointCloud<MyPointType> (cloud, rgb, cloudName);
				}
				for ( int i = 0; i < this->detectedPlanes.size () ; ++i )
				{
					pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
					pcl::copyPointCloud( *this->detectedPlanes[i], *cloud );
					model3d::changeColorOfCloud( cloud, (1+i)*73, 2 );
					pcl::visualization::PointCloudColorHandlerRGBField<MyPointType> rgb(cloud);
					std::string cloudName = "Plane" + std::to_string(static_cast<long long>(i));
					viewer.addPointCloud<MyPointType> (cloud, rgb, cloudName);
				}

				while (!viewer.wasStopped ())
				{
					viewer.spinOnce();
				}
				viewer.close();
				viewer.resetStoppedFlag();
				viewer.close();
				break;
			}
		case 3:
			{
				this->segmentationMinCut();
				break;
			}
		case 4:
			{
				this->segmentationRegionGrow();
				break;
			}
		case 5:
			{
				this->segmentationSupervoxel();
				break;
			}
		default : break;
		}
	}
}
void model3d::uiClusterwahl()
{
	int userInput;
	if ( this->detectedClusters.size() == 0 ) pcl::console::print_error("Kein Cluster vorhanden\n");
	for ( int clusterNumber = 0 ; clusterNumber < this->detectedClusters.size () ; ++clusterNumber)
	{
		Menue::printMenueEntrySmall(clusterNumber,"Cluster mit",this->detectedClusters[clusterNumber]->points.size(),"Punkten.");
	}	

	do Menue::readValue(&userInput);
	while ( userInput > this->detectedClusters.size() );
		
	if ( userInput >= 0 )
		activeCluster=this->detectedClusters[userInput];
	return;
}

void addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
		pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
		std::string supervoxel_name,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
	
	//Iterate through all adjacent points, and add a center point to adjacent point pair
	pcl::PointCloud<pcl::PointXYZRGBA>::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
	for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
	{
		points->InsertNextPoint (supervoxel_center.data);
		points->InsertNextPoint (adjacent_itr->data);
	}
	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
	// Add the points to the dataset
	polyData->SetPoints (points);
	
	polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
	for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
		polyLine->GetPointIds ()->SetId (i,i);
	cells->InsertNextCell (polyLine);
	// Add the lines to the dataset
	polyData->SetLines (cells);
	viewer->addModelFromPolyData (polyData,supervoxel_name);
}