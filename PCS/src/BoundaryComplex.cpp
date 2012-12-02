#include "BoundaryComplex.h"

#include <GL/glew.h>
#include <fstream>

#include <CGAL/basic.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_hierarchy_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>

#include "geometry.h"
#include "base.h"
#include "DisjointSets.h"

using namespace std;
using namespace CGAL;


_Vertex *v = NULL;	// vertex array
int vertexCount = 0, triangleCount = 0, insertedPoints = 0;
struct Triangle;
struct Tetrahedron;
typedef enum { NC_CONFORM, NC_VERTEXCONNECTED, NC_ISOLATED, NC_NONPLANAR } NCType;




class Vertex{
private:
	int _index;
	bool _isOpen;
	NCType _NCType;

public:
	Vertex(){
		_index = -1;
		_isOpen = false;
		_NCType = NC_CONFORM;
	}

	//set index
	void setIndex(int index){
		_index = index;
	}

	// get index
	inline int index(){
		return _index;
	}

	// set vertex classification
	void setOpen(bool isOpen){
		_isOpen = isOpen;
	}

	// get vertex classification

	inline bool isOpen(){
		return _isOpen;
	}

#ifdef OBSOLETE
	// set vertex classification
	void setHoleAdjacent(bool isHoleAdjacent){
		_isHoleAdjacent = isHoleAdjacent;
	}

	// get vertex classification
	inline bool isHoleAdjacent(){
		return _isHoleAdjacent;
	}
#endif

	// set vertex classification
	void setNC(NCType type){
		_NCType = type;
	}

	// get vertex classification
	inline bool isNC(){
		return (_NCType != NC_CONFORM);
	}

	// get non-conforming classification
	inline NCType getNC()	{
		return _NCType;
	}
};

class Tetrahedron{
private:
	int _label;
	int _orientation;
	Triangle *_triangle[4];

public:
	Tetrahedron(){
		_label = -1;
		_orientation = -1;
		for (int i = 0; i < 4; i++)
			_triangle[i] = NULL;
	}

	// set label
	void setLabel(int label){
		_label = label;
	}

	// get label
	int label(){
		return _label;
	}

	//set orientation
	void setOrientation(int orientation){
		_orientation = orientation;
	}

	// get orientation
	int orientation(){
		return _orientation;
	}

	// set triangle with given index
	void setTriangle(int index, Triangle *triangle){
		_triangle[index] = triangle;
	}

	// return triangle with given index
	inline Triangle *triangle(int index){
		return _triangle[index];
	}
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<Vertex, K> Vb;
typedef CGAL::Triangulation_cell_base_with_info_3<Tetrahedron, K> Cb;
typedef CGAL::Triangulation_data_structure_3<Vb, Cb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Dt;
typedef Dt::Cell_handle Cell;
typedef Dt::Facet DTriangle;
typedef Dt::Edge DEdge;
typedef Dt::Vertex_handle DVertex;
typedef Dt::Point Point;
typedef pair<int, int> IntPair;
typedef pair<IntPair, int> IntTriple;

Dt *dt;	// Delaunay Triangulation

// indices of edges start at lowest vertex index of triangle, ccw at halftriangles inside tetrahedron
const int tetraTriVertexIndices[4][3] = { { 1, 3, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 0, 1, 2 } };
const int tetraTriEdgeIndices[4][3] = { { 5, 1, 4 }, { 5, 3, 2 }, { 4, 0, 3 }, { 1, 2, 0 } };	// guarantees that edges with same index as vertex in triangle are opposed
const int tetraEdgeVertexIndices[6][2] = { { 0, 1 }, { 1, 2 }, { 0, 2 }, { 0, 3 }, { 1, 3 }, { 2, 3 } };	// indexes edge index to (sorted) vertex pair


class Edge{
private:
	Cell _cell;
	int _cellIndex[2];
	bool _isOpen, _isNC, _isInteriorOC, _inBoundary;	// TODO: condense states (to bitset?)

public:
	int state;	// DEBUG
	Edge(Cell cell, int index0, int index1){
		_isOpen = false;
		_isNC = false;
		_inBoundary = false;
		_isInteriorOC = false;
		_cell = cell;
		_cellIndex[0] = index0;
		_cellIndex[1] = index1;
	}

	// return a Delaunay halfedge for this edge
	DEdge dEdge(){
		return DEdge(_cell, _cellIndex[0], _cellIndex[1]);
	}

	// return Delaunay vertex
	inline const DVertex vertex(int index){
		return _cell->vertex(_cellIndex[index]);
	}

	// return index of Delaunay vertex
	// returns 2 if not in edge
	int index(DVertex vh){
		int i = 0;
		while ((i < 2) && (vertex(i) != vh))
			i++;
		return i;
	}

	// returns if edge is finite
	bool isFinite(){
		int i = 0;
		while ((i < 2) && !dt->is_infinite(vertex(i)))
			i++;
		return (i == 2);
	}

	// set edge classification
	void setOpen(bool isOpen){
		_isOpen = isOpen;
	}

	// get edge classification
	bool isOpen(){
		return _isOpen;
	}

	//set edge classification
	void setNC(bool isNC){
		_isNC = isNC;
	}

	// get edge classification
	bool isNC(){
		return _isNC;
	}

	// set edge classification
	void setInteriorOC(bool isInteriorOC){
		_isInteriorOC = isInteriorOC;
	}

	// get edge classification
	bool isInteriorOC(){
		return _isInteriorOC;
	}

	// set edge classification
	void setInBoundary(bool inBoundary){
		_inBoundary = inBoundary;
	}

	// get edge classification
	bool inBoundary(){
		return _inBoundary;
	}

	// return if edge exists (in triangulation), by having incident triangles
	bool exists();

	// DEBUG:
	// output edge as v0-v1
	friend ostream& operator<<(ostream& os, const Edge &edge){
		os << edge._cell->vertex(edge._cellIndex[0])->info().index() << "-" 
			<< edge._cell->vertex(edge._cellIndex[1])->info().index();
		return os;
	}

	// return index as int pair
	IntPair index(){
		return IntPair(vertex(0)->info().index(), vertex(1)->info().index());
	}

	// test if it contains these vertices
	bool equals(int v0, int v1){
		IntPair pair = index();
		return ((pair.first == v0) && (pair.second == v1)) || ((pair.first == v1) && (pair.second == v0));
	}

	// return length of edge
	float length(){
		return Vector3D::distance(v[vertex(0)->info().index()].vec, v[vertex(1)->info().index()].vec);
	}
};

class Triangle{
private:
	bool _exists;
	bool _isOpen;
	bool _isInRC;
	int _label;	// label of CC
	Edge *_edge[3];
	Cell _cell;
	int _indexInCell;	// index of its opposite vertex in cell

public:
	Triangle(Cell cell, int index){
		_exists = false;
		_isOpen = false;
		_isInRC = false;
		_label = -1;
		for (int i = 0; i < 3; i++)
			_edge[i] = NULL;
		_cell = cell;
		_indexInCell = index;
	}

	// return index of triangle in cell
	const int indexInCell(){
		return _indexInCell;
	}

	// return index of given vertex (equal to index of opposite edge)
	// returns 3 if not in triangle
	int index(DVertex currVertex){
		int i = 0;
		while ((i < 3) && (vertex(i) != currVertex))
			i++;
		return i;
	}

	// return index of given vertex (equal to index of opposite edge)
	// returns 3 if not in triangle
	int index(Cell cell, DVertex currVertex){
		int vIndex = index(currVertex);
		if (vIndex != 3){
			// test if in opposite cell
			if (cell != _cell)
				vIndex = 2 - vIndex;	// if so, swap orientation
		}
		return vIndex;
	}

	// set existence state of triangle
	void setExists(bool exists){
		_exists = exists;
	}

	// return if triangle exists
	inline bool exists(){
		return _exists;
	}

	// set if triangle is in RC
	void setInRC(bool isInRC){
		_isInRC = isInRC;
	}

	// return if triangle is in RC
	inline bool isInRC(){
		return _isInRC;
	}

	// return Delaunay vertex
	inline const DVertex vertex(int index){
		int vIndex = tetraTriVertexIndices[indexInCell()][index];
		return cell()->vertex(vIndex);
	}

	// return Delaunay vertex
	inline const DVertex vertex(Cell cell, int index){
		// test if in opposite cell
		if (cell != _cell)
			index = 2 - index;	// if so, swap orientation
		return vertex(index);
	}

	// set edge at given index
	void setEdge(int index, Edge *edge){
		_edge[index] = edge;
	}

	// get edge for given index
	// NOTE: triangle is not oriented
	inline Edge *edge(int index){
		return _edge[index];
	}

	// return index of edge in triangle [0..2], 3 if it is not contained in triangle
	int index(Edge *edge){
		int edgeIndex = 0;
		while ((edgeIndex < 3) && (_edge[edgeIndex] != edge))
			edgeIndex++;
		return edgeIndex;
	}

	// get edge for given index, for orientation of triangle in the cell
	inline Edge *edge(Cell cell, int index){
		// test if triangle in opposite cell
		if (cell != _cell)
			index = 2 - index;	// if so, swap orientation
		return edge(index);
	}

	// returns if triangle is finite
	const bool isFinite(){
		int i = 0;
		while ((i < 3) && !dt->is_infinite(vertex(i)))
			i++;
		return (i == 3);
	}

	// set edge classification
	void setOpen(bool isOpen){
		_isOpen = isOpen;
	}

	// get edge classification
	inline bool isOpen(){
		return _isOpen;
	}

	// set label
	void setLabel(int label){
		_label = label;
	}

	// get label
	int label(){
		return _label;
	}

	// returns one adjacent cell
	const inline Cell cell(){
		return _cell;
	}

	// return index as int triple
	const IntTriple index(){
		return IntTriple(IntPair(vertex(0)->info().index(), vertex(1)->info().index()), vertex(2)->info().index());
	}

	// returns a halftriangle for the triangle
	DTriangle halftriangle(){
		return DTriangle(cell(), indexInCell());
	}

	// DEBUG:
	// output triangle as v0/v1/v2
	friend ostream& operator<<(ostream& os, const Triangle &triangle){
		int i;
		for (i = 0; i < 3; i++)	{
			os << triangle._cell->vertex(tetraTriVertexIndices[triangle._indexInCell][i])->info().index();
			if (i < 2)
				os << "/";
		}
		return os;
	}

	// test if triangle contains exactly these vertex indices
	bool equals(int *vArray){
		int i, _vArray[3];
		for (i = 0; i < 3; i++)
			_vArray[i] = cell()->vertex(tetraTriVertexIndices[indexInCell()][i])->info().index();
		sort(_vArray, _vArray + 3);
		sort(vArray, vArray + 3);
		i = 0;
		while ((i < 3) && (vArray[i] == _vArray[i]))
			i++;
		return (i == 3);
	}
};

// returns triangle for halftriangle
Triangle *ht2Triangle(DTriangle ht){
	return ht.first->info().triangle(ht.second);
}


map<IntPair, Edge> dtEdgeMap;	// key are vertex indices sorted ascendingly
list<Triangle> dtTriangles;

const char* timer_names[] = { "total", "reconstruct", "del3d", "struct", "boundary-complex", "spaces", "detect", "conform", "segment", "hole-filling", "inflate", "sculpture", "smooth",
	"temp", "last" };




//########################################### BEGIN INTERESTING ######################################

list<Triangle*> boundaryTriangles;



set<int> BoundaryComplex::getNeighbors(int index){
	set<int> neighbors;
	for (list<Triangle>::iterator iter = dtTriangles.begin(); iter != dtTriangles.end(); iter++)
	{
		Triangle *currTri = &*iter;
		if (currTri->exists()) {
			bool indexFound = false;
			for (int i = 0; i < 3; i++){
				if(currTri->vertex(i)->info().index() == index)
					indexFound=true;
			}
			if(indexFound){
				for (int i = 0; i < 3; i++){
					if(currTri->vertex(i)->info().index() != index)
						neighbors.insert(currTri->vertex(i)->info().index());
				}
			}
		}
	}
	return neighbors;
}


void BoundaryComplex::doClustering(Solution *s){
	set<int> unlabeled;
	for(int i=0; i<s->clustering->size(); i++)
		if(s->clustering->at(i)==0)
			unlabeled.insert(i);
	
	while(!unlabeled.empty()){
		list<int> queue;
		set<int> cluster;

		set<int>::iterator seed = unlabeled.begin();
		queue.push_back(*seed);
		cluster.insert(*seed);
		unlabeled.erase(seed);
		
		while(!queue.empty()){
			list<int>::iterator curPoint = queue.begin();
			set<int> neighbors = getNeighbors(*curPoint);
			queue.pop_front();

			for(set<int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); iter++){
				set<int>::iterator neighbor = unlabeled.find(*iter);
				if(neighbor!=unlabeled.end()){
					Vector3D v1 = v[*curPoint].vec;
					Vector3D v2 = v[*neighbor].vec;
					float distance = sqrt(pow(v1[0]-v2[0],2)+
											pow(v1[1]-v2[1],2)+
											pow(v1[2]-v2[2],2));
					if(distance<=s->dist_threshold){
						queue.push_back(*neighbor);
						cluster.insert(*neighbor);
						unlabeled.erase(neighbor);
					}
				}
			}
		}

		if(cluster.size() >= s->cloud->size()/config::min_cluster_size){
			for(set<int>::iterator iter = cluster.begin(); iter!=cluster.end(); iter++){
				s->clustering->at(*iter)=s->cluster_count;
			}
			cout << "Cluster "<<s->cluster_count<<": " << cluster.size() << " data points." << endl;
			s->cluster_count++;
		}

	}
	

}

//########################################### END INTERESTING ########################################




Dt *construct3DDelaunayTriangulation(){
	int i;
	Dt *dt = new Dt();

	for (i = 0; i < vertexCount; i++){
		Point point = Point(v[i].vec[0], v[i].vec[1], v[i].vec[2]);
		DVertex vh = dt->insert(point);
		vh->info().setIndex(i);
	}

	//	cout << "DT3: finite: vertices=" << dt->number_of_vertices() << ", edges=" << dt->number_of_finite_edges() << ", faces=" << dt->number_of_finite_facets() << ", cells=" << dt->number_of_finite_cells() << endl;
	cout << "DT3: all: vertices=" << (dt->number_of_vertices() + 1) << ", edges=" << dt->number_of_edges() << ", faces=" << dt->number_of_facets() << ", cells=" << dt->number_of_cells() << endl;

	if ((int)dt->number_of_vertices() != vertexCount){
		cout << "Warning: point set contains identical points: will fail later because of indices larger than that count!" << endl;

		// adjust indices to account for removed duplicate points (required for consistency, and no indices >= vertexcount!)
		map<int, int> vertexMap;
		i = 0;

		for (Dt::Vertex_iterator vhIter = dt->vertices_begin(); vhIter != dt->vertices_end(); vhIter++){
			DVertex vh = vhIter;

			if (vh->info().index() != -1)
			{
				vertexMap[i] = vh->info().index();
				vh->info().setIndex(i);
				i++;
			}
		}

		cout << "vertices reduced from " << vertexCount << " to " << (int)dt->number_of_vertices() << endl;

		vertexCount = (int)dt->number_of_vertices();
		_Vertex *newV = new _Vertex[vertexCount];

		for (i = 0; i < vertexCount; i++){
			//			cout << i << ": " << vertexMap[i] << endl;
			newV[i].vec = v[vertexMap[i]].vec;
		}

		//		delete v;	// TODO: dirty
		v = newV;
	}
	return dt;
}


int criterionType = -1;	// 0=circumradius, 1=longest edge, 2=area, 3=aspect ratio
// calculate criterion for triangle (minimizing)
double getCriterionForTriangle(Triangle *triangle){
	int i, triIndex[3];

	for (i = 0; i < 3; i++)
		triIndex[i] = triangle->vertex(i)->info().index();

	if (criterionType == 0){
		// criterion: circumradius
		Vector3D v0 = v[triIndex[0]].vec;
		Vector3D cc;
		float r = calcCircumcenterRadiusDet(v0, v[triIndex[1]].vec, v[triIndex[2]].vec, cc);
		return r;
	}
	else
		if (criterionType == 1){
			// criterion: longest edge in triangle (prefers also acute angles, small triangles)
			float edge[3];

			for (i = 0; i < 3; i++)
				edge[i] = Vector3D::distance(v[triIndex[i]].vec, v[triIndex[(i + 1) % 3]].vec);

			sort(edge, edge + 3);	// determine longest edge

			return edge[2];
		}
		else
			if (criterionType == 2){
				// criterion: calculate triangle area
				return calcTriangleArea(v[triIndex[0]].vec, v[triIndex[1]].vec, v[triIndex[2]].vec);
			}
			else
				if (criterionType == 3){
					// criterion: calculate triangle aspect ratio
					return calcTriangleAspectRatio(v[triIndex[0]].vec, v[triIndex[1]].vec, v[triIndex[2]].vec);
				}
				// dummy
				return 0.0;
}


// creates edge and triangle structures and links them to existing vertex/tetrahedron structure, to permit storing attributes
void createAggregateDataStructure(){
	int i, j;

	// traverse all tetrahedra and create their triangles, edges and references to them
	stack<Cell> tetraStack;
	tetraStack.push(dt->infinite_cell());

	// while stack not empty
	while (tetraStack.size() > 0){
		// pop tetra from stack
		Cell ch = tetraStack.top();
		tetraStack.pop();
		Tetrahedron *currTetra = &ch->info();

		// DEBUG
		//		cout << "tetra " << COUT_CELL(ch) << endl;

		bool triangleCreated[4];
		Edge *edge[6];

		for (i = 0; i < 6; i++)
			edge[i] = NULL;

		// reference its four triangles (create if not yet existing)
		for (i = 0; i < 4; i++)	{
			triangleCreated[i] = false;

			// look up at neighbor tetrahedron
			Cell oppCH = ch->neighbor(i);
			int oppIndex = oppCH->index(ch);
			Tetrahedron *oppTetra = &oppCH->info();

			// test if triangle exists already
			Triangle *triangle = oppTetra->triangle(oppIndex);

			if (triangle == NULL){
				triangleCreated[i] = true;

				// create new triangle

				//int vIndex[3];

				//for (j = 0; j < 3; j++)
				//				//	vIndex[j] = (i + 1 + j) % 4;
				//	vIndex[j] = tetraTriVertexIndices[i][j];

				Triangle newTriangle(ch, i);
				dtTriangles.push_back(newTriangle);
				triangle = &dtTriangles.back();

				// also push tetrahedron on stack as unhandled
				tetraStack.push(oppCH);

				// DEBUG
				//				cout << "triangle " << *triangle << " created" << endl;
			}

			currTetra->setTriangle(i, triangle);	// reference triangle
			oppTetra->setTriangle(oppIndex, triangle);		// also in opposite tetrahedron
		}

		// reference its six edges (create if not yet existing)
		for (i = 0; i < 6; i++){
			int edgeVIndex[2];

			for (j = 0; j < 2; j++)
				edgeVIndex[j] = ch->vertex(tetraEdgeVertexIndices[i][j])->info().index();

			sort(edgeVIndex, edgeVIndex + 2);

			IntPair intPair(edgeVIndex[0], edgeVIndex[1]);
			map<IntPair, Edge>::iterator mapIter = dtEdgeMap.find(intPair);

			if (mapIter != dtEdgeMap.end())
				edge[i] = &mapIter->second;
			else
			{
				Edge newEdge(ch, tetraEdgeVertexIndices[i][0], tetraEdgeVertexIndices[i][1]);
				dtEdgeMap.insert(pair<IntPair, Edge>(intPair, newEdge));
				map<IntPair, Edge>::iterator mapIter = dtEdgeMap.find(intPair);
				edge[i] = &mapIter->second;
			}
		}

		// only set edges into newly created triangles
		for (i = 0; i < 4; i++)
			if (triangleCreated[i])
				for (j = 0; j < 3; j++)
					currTetra->triangle(i)->setEdge(j, edge[tetraTriEdgeIndices[i][j]]);
	}
	// DEBUG
	cout << "aggregate data structure: " << dtEdgeMap.size() << " edges, " << dtTriangles.size() << " triangles" << endl;
}


// calculate criterion for triangles and return sorted map with its references
void getSortedTriangleMap(multimap<double, Triangle *> &triMMap)
{
	// sort all triangles in delaunay graph by ascending triangle circumradius
	for (list<Triangle>::iterator triIter = dtTriangles.begin(); triIter != dtTriangles.end(); triIter++)
	{
		Triangle *triangle = &*triIter;

		if (triangle->isFinite())
		{
			double crit = getCriterionForTriangle(triangle);
			triMMap.insert(pair<double, Triangle *>(crit, triangle));
		}
	}
}



// create boundary-complex for DG(P)
// conditions: 1) all vertices interpolated, 2) all edges >=2 incident triangles, 3) one connected set
// repeat until all conditions fulfilled
// input: delaunay tetrahedralization
// output: all triangles in boundary-complex marked as existing
void createBoundaryComplex(Dt *dt)
{
	int i, insertedTriangleCount = 0;

	// initialize a tree for each vertex
	int vTreeCount = dt->number_of_vertices();
	//	DisjointSets dSet(vTreeCount);
	DisjointSets dSet(vertexCount);	// DIRTY! -> problems with false indices?

	// get sorted triangle map
	multimap<double, Triangle *> triMMap;
	getSortedTriangleMap(triMMap);

	// initialize a tree for each triangle
	DisjointSets tSet(triMMap.size());

	set<Edge *> externalEdgeSet;
	set<Triangle *> rejectedTriangleSet;

	// while more than one vertex tree or still external edges left
	while ((triMMap.size() > 0) && ((vTreeCount > 1) || (externalEdgeSet.size() > 0)))
	{
		// remove minimal triangle from set
		multimap<double, Triangle *>::iterator triIter = triMMap.begin();
		Triangle *triangle = triIter->second;
		int triIndex[3];

		for (i = 0; i < 3; i++)
			triIndex[i] = triangle->vertex(i)->info().index();

		triMMap.erase(triIter);

		// check if that triangle connects two yet unconnected trees
		int root[3], root01;

		for (i = 0; i < 2; i++)
			root[i] = dSet.FindSet(triIndex[i]);

		bool inserted = false;

		// if vertex #0 and #1 not in same tree, connect them and insert triangle into EMST
		if (root[0] != root[1])
		{
			dSet.Union(root[0], root[1]);
			inserted = true;
			vTreeCount--;
		}

		root01 = dSet.FindSet(root[0]);
		root[2] = dSet.FindSet(triIndex[2]);	// can have been incorporated into root0 or root1

		// same for joined tree and vertex #2 (triangle will not appear twice in set)
		if (root01 != root[2])
		{
			dSet.Union(root01, root[2]);
			inserted = true;
			vTreeCount--;
		}

		// check if triangle connects to an external edge
		list<Edge *> newNonExtEdges, newExtEdges;

		for (i = 0; i < 3; i++)
		{
			Edge *edge = triangle->edge(i);
			DEdge dEdge = edge->dEdge();
			Dt::Facet_circulator startFC = dt->incident_facets(dEdge);
			Dt::Facet_circulator currFC = startFC;
			int edgeTriCount = 0;

			do
			{
				DTriangle currDT = *currFC;
				Triangle *currTri = ht2Triangle(currDT);

				if ((currTri != triangle) && currTri->exists())
					edgeTriCount++;

				currFC++;
			} while (currFC != startFC);

			if (edgeTriCount == 0)
				newExtEdges.push_back(edge);
			else
				if (edgeTriCount == 1)
				{
					newNonExtEdges.push_back(edge);
					inserted = true;
				}
		}

		if (inserted)
		{
			insertedTriangleCount++;
			triangle->setExists(true);
			triangle->setInRC(true);

			for (list<Edge *>::iterator edgeIter = newNonExtEdges.begin(); edgeIter != newNonExtEdges.end(); edgeIter++)
				externalEdgeSet.erase(*edgeIter);

			for (list<Edge *>::iterator edgeIter = newExtEdges.begin(); edgeIter != newExtEdges.end(); edgeIter++)
			{
				Edge *edge = *edgeIter;
				externalEdgeSet.insert(edge);

				// re-insert triangles in evaluation queue if they are adjacent to a new external edge
				DEdge dEdge = edge->dEdge();
				Dt::Facet_circulator startFC = dt->incident_facets(dEdge);
				Dt::Facet_circulator currFC = startFC;

				do
				{
					DTriangle currDT = *currFC;
					Triangle *currTri = ht2Triangle(currDT);

					// if located in rejected set
					if ((currTri != triangle) && (rejectedTriangleSet.find(currTri) != rejectedTriangleSet.end()))
					{
						// move from there to evaluation queue
						set<Triangle *>::iterator tri2Iter = rejectedTriangleSet.find(currTri);
						double crit = getCriterionForTriangle(currTri);
						triMMap.insert(pair<double, Triangle *>(crit, currTri));
						rejectedTriangleSet.erase(tri2Iter);
					}

					currFC++;
				} while (currFC != startFC);
			}
		}
		else
			rejectedTriangleSet.insert(triangle);
	}

	cout << "boundary-complex: " << insertedTriangleCount << " triangles" << endl;
}



/*
* write triangulation to OFF-file
*/
void writeOFFFile()
{
	int i;
	string offFilenameStr = "boundaryComplex.off";
	const char *offFilename = offFilenameStr.c_str();
	ofstream fileOut(offFilename);

	if (fileOut.is_open())
	{
		// count triangles
		int triangleCount = 0;

		for (list<Triangle>::iterator iter = dtTriangles.begin(); iter != dtTriangles.end(); iter++)
		{
			Triangle *currTri = &*iter;

			if (currTri->exists())
				triangleCount++;
		}

		// write OFF header
		fileOut << "OFF" << endl;
		fileOut << (int)dt->number_of_vertices() << " " << triangleCount << " 0" << endl;

		// TODO: some vertex indices are higher than vertexCount: (up to 200 in ts-dragon): why? and if there are holes, need to compact them for triangle writing!

		// write vertex list
		for (Dt::Finite_vertices_iterator vhIter = dt->finite_vertices_begin(); vhIter != dt->finite_vertices_end(); vhIter++)
		{
			DVertex currVH = vhIter;
			int vIndex = currVH->info().index();

			if (vIndex != -1)
			{
				Vector3D vec = v[vIndex].vec;

				for (i = 0; i < 3; i++)
					fileOut << vec[i] << " ";

				fileOut << endl;

				// DEBUG
				if (vIndex >= (int)dt->number_of_vertices())
				{
					cout << "v #" << vIndex << ">= " << (int)dt->number_of_vertices() << "(vertexcount)!" << endl;
					assert(false);
				}
			}
		}

		// write triangle list
		for (list<Triangle>::iterator iter = dtTriangles.begin(); iter != dtTriangles.end(); iter++)
		{
			Triangle *currTri = &*iter;

			if (currTri->exists())
			{
				fileOut << "3 ";

				for (i = 0; i < 3; i++)
					fileOut << currTri->vertex(i)->info().index() << " ";

				fileOut << endl;
			}
		}

		fileOut.close();
	}
	else
		cout << "ERROR: could not write OFF file " << offFilenameStr << endl;
}


// reconstruct triangulation from point set
void BoundaryComplex::connect3D()
{
	dt = construct3DDelaunayTriangulation();

	createAggregateDataStructure();

	criterionType = 1;	// criterion: longest edge in triangle
	createBoundaryComplex(dt);

	// put all boundary triangles into the list
	for (list<Triangle>::iterator iter = dtTriangles.begin(); iter != dtTriangles.end(); iter++)
	{
		Triangle *currTri = &*iter;

		if (currTri->exists())
			boundaryTriangles.push_back(currTri);
	}

	writeOFFFile();
}


BoundaryComplex::BoundaryComplex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in){
	vertexCount = cloud_in->size();	// TODO: why? (for knot.pts)
	v = new _Vertex[vertexCount];
	_Vertex vertex;
	int i = 0;
	for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = cloud_in->points.begin(); it != cloud_in->points.end(); ++it){
		Vector3D vec;
		vec[0] = it->x;
		vec[1] = it->y;
		vec[2] = it->z;

		vertex.vec = vec;
		v[i] = vertex;
		i++;
	}

	connect3D();
}



BoundaryComplex::~BoundaryComplex(){

}