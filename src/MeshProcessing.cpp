#include "MeshProcessing.h"
#include <pcl/Vertices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <vector>


// Calculates the area of ​​a triangle, receives the xyz coordinates of the three angles, returns the area
double calculateTriangleMeshArea(
	double x1, double y1, double z1,
	double x2, double y2, double z2,
	double x3, double y3, double z3) {
	// Heron's formula
	double a, b, c, q;
	double area;
	a = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2) + pow((z1 - z2), 2));
	b = sqrt(pow((x1 - x3), 2) + pow((y1 - y3), 2) + pow((z1 - z3), 2));
	c = sqrt(pow((x3 - x2), 2) + pow((y3 - y2), 2) + pow((z3 - z2), 2));
	q = (a + b + c) / 2;
	area = sqrt(q * (q - a) * (q - b) * (q - c));
	return area;
}

// Receives a constant reference to pcl::PolygonMesh and calculates the total area of ​​the mesh (note: the mesh does not have to be triangles)
// v0.2 This version just triangulates polygons by given points, but has two limitations:
//   (1) Polygon vertices must be ordered; (2) Only for convex polygons, not for concave polygons
// FIXME:
double calculatePCLPolygonMeshArea(const pcl::PolygonMesh& mesh) {
	size_t index1, index2, index3;
	double x1, y1, z1, x2, y2, z2, x3, y3, z3;
	double area = 0;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	// for each polygon
	for (size_t i = 0; i != mesh.polygons.size(); ++i) {

		std::vector<uint32_t> vertexIndexes = mesh.polygons[i].vertices;
		int n = vertexIndexes.size();  // n 边形

		// Decompose an n-gon into (n - 2) triangles
		double polygonArea = 0;
		for (int k = 1; k != n - 1; ++k) {
			index1 = vertexIndexes[0];
			index2 = vertexIndexes[k];
			index3 = vertexIndexes[k + 1];

			x1 = cloud[index1].x;
			y1 = cloud[index1].y;
			z1 = cloud[index1].z;

			x2 = cloud[index2].x;
			y2 = cloud[index2].y;
			z2 = cloud[index2].z;

			x3 = cloud[index3].x;
			y3 = cloud[index3].y;
			z3 = cloud[index3].z;

			polygonArea += calculateTriangleMeshArea(x1, y1, z1, x2, y2, z2, x3, y3, z3);
		}

		area += polygonArea;
	}
	return area;
}

// Greedy Projection triangulation
pcl::PolygonMesh triangulationGreedyProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(xyzCloud);
	normalEstimation.setInputCloud(xyzCloud);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setKSearch(20);
	normalEstimation.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	// Concatenate the obtained point data and normal data
	pcl::concatenateFields(*xyzCloud, *normals, *cloudWithNormals);

	// another kd-tree for reconstruction
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloudWithNormals);

	// reconstruction
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh mesh;
	// options
	gp3.setSearchRadius(25);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 2);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);
	gp3.setInputCloud(cloudWithNormals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(mesh);

	return mesh;
}
