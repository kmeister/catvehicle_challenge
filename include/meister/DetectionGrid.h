/*
Author: Kurt Meister

Copyright (c) 2017, Kurt Meister
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

-------------------------------------------------------------------------------

Description:
    contains the definition for a class and ssociated utility structures which
    map point cloud data to an occupancy grid and then applies a connected components
    labeling algorithm to idenitfy groups of points representing individual objects
    the class can provide an axis aligned bounding box for each detected object in the
    form of a geometry_msgs::polygon

 */

#ifndef MEISTER_DETECTIONGRID_H
#define MEISTER_DETECTIONGRID_H


#include <vector>
#include <set>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ostream>
#include <ros/ros.h>
#include <sstream>

using namespace geometry_msgs;
using namespace std;


/** linked list utility structure for union find portion of the two pass connected component labeling algorithm
 *
 */
struct LabelSet {


    int value;
    std::shared_ptr<LabelSet> parent;


    LabelSet() : value(0) {

    }

    LabelSet(int v) : value(v) {

    }


    std::shared_ptr<LabelSet> find(){
        if (parent.get() == this){
            return parent;
        } else {
            return parent->find();
        }
    }

    static void Union(std::shared_ptr<LabelSet> l1, std::shared_ptr<LabelSet> l2){
        std::shared_ptr<LabelSet> root1 = l1->find();
        std::shared_ptr<LabelSet> root2 = l2->find();

        root1->parent = root2;
    }

    friend ostream &operator<<(ostream &os, const LabelSet &labelSet) {
        os << "value: " << labelSet.value << " parent: " << labelSet.parent;
        return os;
    }

    static std::shared_ptr<LabelSet> Create(int value = 0){
        std::shared_ptr<LabelSet> ptr(new LabelSet(value));
        ptr->parent = ptr;
        return ptr;
    }
};


typedef std::shared_ptr<LabelSet> LabelSetPtr; //!< utility typedef

/** utility structure for passing around grid indices
 */
struct CellId {
    int row;
    int col;

    /** make useable with ostream operator
     *
     * @param os output stream
     * @param id CellId to print
     * @return output stream
     */
    friend ostream &operator<<(ostream &os, const CellId &id) {
        os << "row: " << id.row << " col: " << id.col;
        return os;
    }
};

typedef vector< CellId > Blob;

/** represents a single cell in the detection grid
 *
 */
struct Cell {
    vector<Point32> points;
    int label;

    Cell(): label(0), points() {};
};


/** class for processing point cloud data to generate polygons to publish to the
 *  /detections topic
 */
class DetectionGrid {
public:


    /** Constructor
     *
     * @param points  point cloud to process
     * @param cell_size resolution of the grid to which the point cloud is mapped
     */
    DetectionGrid(const vector<geometry_msgs::Point32> &points, float cell_size);

    /**Constructor
     * @param cell_size resolution of the grid to which the point cloud is mapped
     */
    DetectionGrid(float cell_size);


    /** insert a single point into the occupancy grid
     *
     * @param point point to insert into the grid
     */
    void insert(const Point32 &point );

   /** clear the points in the grid and insert a new set of points
    *
    * @param points vector containing the points to insert into the grid
    */
    void setPoints(const vector<geometry_msgs::Point32> &points);


    /** utilty function to get a vector of neighboring cells to a given cell in the occupancy grid
     *
     * @param row row of cell to get the neighbors of
     * @param col column of cell to get the neighbors of
     * @return vector conataining the eight neighbors of cell(row,col)
     */
    vector<CellId> find_neighbors(int row, int col);

    /** verify that a given cell is within the bounds of the occupancy grid
     *
     * @param id CellId to evaluate
     * @return true id cell is within the grid, false if not
     */
    bool in_bounds(CellId id);

    /** get a point representing the northwest corner of a grid cell in the global coordinate system
     *
     * @param id CellId of the cell from which the point will be generated
     * @return Point32 representing the northwest corner of the cell
     */
    Point32 northwest_corner(const CellId &id);

    /** get a point representing the northeast corner of a grid cell in the global coordinate system
     *
     * @param id CellId of the cell from which the point will be generated
     * @return Point32 representing the northeast corner of the cell
     */
    Point32 northeast_corner(const CellId &id);

    /** get a point representing the southeast corner of a grid cell in the global coordinate system
     *
     * @param id CellId of the cell from which the point will be generated
     * @return Point32 representing the southeast corner of the cell
     */
    Point32 southeast_corner(const CellId &id);

    /** get a point representing the southwest corner of a grid cell in the global coordinate system
     *
     * @param id CellId of the cell from which the point will be generated
     * @return Point32 representing the southwest corner of the cell
     */
    Point32 southwest_corner(const CellId &id);

    /** get a point representing the center corner of a grid cell in the global coordinate system
     *
     * @param id CellId of the cell from which the point will be generated
     * @return Point32 representing the center corner of the cell
     */
    Point32 center(const CellId &id);

    /** generates an axis aligned bounding box for one of the detected objects
     *
     * @param label identifies the object
     * @return a polygon representing an axis aligned bounding rectangle for the object identified by label
     */
    geometry_msgs::Polygon get_axis_aligned_bounding_box(int label);

    /** cell debug info for this object
     *
     * @return data about the cells in this detection grid
     */
    std::string describe_cells();

    /** label debug infor for this object
     *
     * @return string description of the labels grid
     */
    std::string describe_labels();

    /** bounding debug info for this object
     *
     * @return string description of the detection grid's bounds
     */
    std::string describe_bounds();

    /** utility function to get the label of a given cell in the label grid
     *
     * @param id CellID of the desired cell
     * @return label of the cell
     */
    int get_label_by_id(CellId id);

    /** get the set of labels from the label grid
     *
     * @return set of all labels from the labels grid
     */
    set<int> label_names();

private:

    vector< vector <  Cell > > cells_; //!< occupancy grid with points mapped into it

    vector< vector < int > > labels_; //!<labeled components grid

    size_t rows_; //!< number of rows in the grid
    size_t cols_; //!< number of columns in the grid

    // grid bounds in point cloud coordinates
    float min_x_;
    float min_y_;
    float max_x_;
    float max_y_;
    float x_offset_;
    float y_offset_;
    float cell_size_;

    /** identifies blobs within the cell grid and labels the cells accordingly
     *
     */
    void label_components();


    /** sets constants above to map the grid coordinates to the coordinates of the points in the point cloud
     *
     * @param points a vector of points used to generated the grid to point mappings
     */
    void find_bounds(const vector<geometry_msgs::Point32> &points) ;
};


// inline methods
inline set<int> DetectionGrid::label_names(){
    set<int> label_names;

    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            if (labels_[r][c] > 0)
                label_names.insert(labels_[r][c]);
        }
    }

    return label_names;
}

inline Point32 DetectionGrid::northwest_corner(const CellId &id) {
    stringstream ss;

    ss << id;

    ROS_DEBUG("Coord: %s", ss.str().c_str());
    Point32 ret;
    ret.z = 1;

    ret.x = cell_size_ * id.col - x_offset_;
    ret.y = cell_size_ * (id.row + 1) - y_offset_;

    return ret;
}

inline Point32 DetectionGrid::northeast_corner(const CellId &id) {
    stringstream ss;

    ss << id;

    ROS_DEBUG("Coord: %s", ss.str().c_str());
    Point32 ret;
    ret.z = 1;

    ret.x = cell_size_ * (id.col + 1) - x_offset_;
    ret.y = cell_size_ * (id.row + 1) - y_offset_;

    return ret;
}

inline Point32 DetectionGrid::southeast_corner(const CellId &id) {
    stringstream ss;

    ss << id;

    ROS_DEBUG("Coord: %s", ss.str().c_str());
    Point32 ret;
    ret.z = 1;

    ret.x = cell_size_ * (id.col + 1 ) - x_offset_;
    ret.y = cell_size_ * (id.row ) - y_offset_;

    return ret;
}

inline Point32 DetectionGrid::southwest_corner(const CellId &id) {

    stringstream ss;

    ss << id;

    ROS_DEBUG("Coord: %s", ss.str().c_str());

    Point32 ret;
    ret.z = 1;

    ret.x = cell_size_ * id.col - x_offset_;
    ret.y = cell_size_ * id.row - y_offset_;

    return ret;
}

inline Point32 DetectionGrid::center(const CellId &id) {
    Point32 ret = southwest_corner(id);

    ret.x += ( cell_size_ / 2 );
    ret.y += ( cell_size_ / 2 );

    return ret;
}



#endif //MEISTER_DETECTIONGRID_H
