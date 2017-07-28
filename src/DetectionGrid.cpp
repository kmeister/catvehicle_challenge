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
    Contains the implementation of the DetectionGrid class.
 */

#include "DetectionGrid.h"
#include <limits>
#include <algorithm>
#include <ros/ros.h>

DetectionGrid::DetectionGrid(float cell_size) : cell_size_(cell_size) {};

DetectionGrid::DetectionGrid(const vector<geometry_msgs::Point32> &points, float cell_size) : cell_size_(cell_size) {
    find_bounds(points);
    cols_ = static_cast<size_t>(ceil((max_x_ - min_x_) / cell_size_));
    rows_ = static_cast<size_t>(ceil((max_y_ - min_y_) / cell_size_));


    // make sure everything is initialized properly
    cells_.resize(rows_);
    labels_.resize(rows_);

    for (int r = 0; r < rows_; r++){
        cells_[r].resize(cols_);
        labels_[r].resize(cols_);
        for (int c = 0; c < cols_; c++){
            cells_[r][c] = Cell();
            labels_[r][c] = 0;
        }
    }

    setPoints(points);
    label_components();

}

void DetectionGrid::insert(const Point32 &point) {
    int col;
    int row;

    if (point.x >= min_x_ && point.x <= max_x_ && point.y >= min_y_ && point.y <= max_y_) {
        col = static_cast<int>(floor((point.x + x_offset_) / cell_size_));
        row = static_cast<int>(floor((point.y + y_offset_) / cell_size_));

        if (!in_bounds(CellId{row,col})) {
            stringstream ss;

            ss << "Error in DetectionGrid::setPoints\n\t";
            ss << "x_offset: " << x_offset_ << "\n\t";
            ss << "y_offset: " << y_offset_ << "\n\t";
            ss << "point.x: " << point.x << "\n\t";
            ss << "point.y: " << point.y << "\n\t";
            ss << "rows: " << rows_ << "\n\t";
            ss << "cols: " << cols_ << "\n\t";
            ss << "cell_size: " << cell_size_;

            ROS_ERROR("Attempting to setPoints to cell that does not exist: (%d, %d)", row, col);
            //TODO: remove this exception once you're sure the bounds checking works
            throw ros::Exception(ss.str().c_str());

        } else {
            //ROS_WARN("Inserting Point at (%d,%d)", row, col);
            cells_[row][col].points.push_back(point);
        }
    }
}

//TODO: error checking for empty point cloud...
void DetectionGrid::setPoints(const vector<geometry_msgs::Point32> &points) {
    find_bounds(points);
    cols_ = static_cast<size_t>(ceil((max_x_ - min_x_) / cell_size_));
    rows_ = static_cast<size_t>(ceil((max_y_ - min_y_) / cell_size_));

    // make sure everything is initialized properly
    cells_.resize(rows_);
    labels_.resize(rows_);

    for (int r = 0; r < rows_; r++){
        cells_[r].resize(cols_);
        labels_[r].resize(cols_);
        for (int c = 0; c < cols_; c++){
            cells_[r][c] = Cell();
            labels_[r][c] = 0;
        }
    }

    for (int i = 0; i < points.size(); i++){
        insert(points[i]);
    }

    //ROS_WARN("%s", describe_bounds().c_str());
    //ROS_WARN("%s", describe_cells().c_str());
    //ROS_WARN("cell (0,0) : %d", cells_[0][0]);

    label_components();
}

void DetectionGrid::label_components(){
    ROS_DEBUG("DetectionGrid::label_components");

    int next_label = 1;
    vector<LabelSetPtr> linked;
    linked.push_back(LabelSet::Create(0));


    for (int r = 0; r < rows_; r++){
        for (int c = 0; c < cols_; c++){

            if (!cells_[r][c].points.empty()){ // not empty

                vector<CellId> neighbors = find_neighbors(r,c);
                vector<int> neighbor_labels;
                int min_label = next_label;

                for (int i = 0; i < neighbors.size(); i++){

                    int id = get_label_by_id(neighbors[i]);
                    if (id > 0) {
                        neighbor_labels.push_back(id);
                        if (id < min_label){
                            min_label = id;
                        }
                    }
                }

                if (neighbor_labels.empty()){
                    ROS_DEBUG("DetectionGrid::label_components, neighbors.empty");
                    linked.push_back(LabelSet::Create(next_label));
                    labels_[r][c] = next_label;
                    next_label++;
                } else {
                    ROS_DEBUG("DetectionGrid::label_components, !neighbors.empty");
                    // get labels from neighbors
                    //auto neighbors = find_neighbors(r, c);
                    ROS_DEBUG("min_label: %d", min_label);

                    labels_[r][c] = min_label;


                    // store equivalencies
                    for (int l = 0; l < neighbor_labels.size(); l++) {

                        LabelSet::Union(linked[neighbor_labels[l]], linked[min_label]);
                    }
                }

            }
        }
    }

    // update labels with equivalencies

    for (int r = 0; r < rows_; r++){
        for (int c = 0; c < cols_; c++) {
            if (!cells_[r][c].points.empty()){
                stringstream ss;
                ss << *linked[labels_[r][c]];
                ROS_DEBUG("%s",ss.str().c_str());

                labels_[r][c] = linked[labels_[r][c]]->find()->value;

            }
        }
    }

}


vector<CellId> DetectionGrid::find_neighbors(int row, int col) {
    ROS_DEBUG("DetectionGrid::find_neighbors(%d, %d)", row, col);

    CellId northwest{row + 1, col -1};
    CellId north{row + 1, col};
    CellId northeast{row + 1, col + 1};
    CellId east {row, col + 1};
    CellId southeast{ row - 1, col + 1};
    CellId south{ row - 1, col};
    CellId southwest{row - 1, col - 1};
    CellId west{ row , col - 1};

    vector<CellId> ret;

    if (in_bounds(northwest)) {
        ret.push_back(northwest);
    }

    if (in_bounds(north)){
        ret.push_back(north);
    }

    if (in_bounds(northeast)) {
        ret.push_back(northeast);
    }

    if (in_bounds(east)){
        ret.push_back(east);
    }

    if (in_bounds(southeast)){
        ret.push_back(southeast);
    }

    if (in_bounds(south)){
        ret.push_back(south);
    }

    if (in_bounds(southwest)){
        ret.push_back(southwest);
    }

    if (in_bounds(west)) {
        ret.push_back(west);
    }

    return ret;
}

bool DetectionGrid::in_bounds(CellId id) {
    return id.row >= 0 && id.row < rows_ && id.col >= 0 && id.col < cols_;
}

geometry_msgs::Polygon DetectionGrid::get_axis_aligned_bounding_box(int label) {
    float min_x = max_x_;
    float min_y = max_y_;
    float max_x = min_x_;
    float max_y= min_y_;
    float max_z = 0;

    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            if (labels_[r][c] == label) {

                for (vector<Point32>::iterator it = cells_[r][c].points.begin(); it != cells_[r][c].points.end(); ++it ){
                    max_x = max(max_x, it->x);
                    min_x = min(min_x, it->x);
                    max_y = max(max_y, it->y);
                    min_y = min(min_y, it->y);
                    max_z = max(max_z, it->z);
                }
            }
        }
    }

    //prevent polygon from having zero area
    // this will put detected points on the front edge of a bounding box with length = cell_size expanding the bounding box away from the catvehicle
    if (min_x == max_x){
        max_x += cell_size_;
    }


    if (min_y == max_y){

        if (min_y >= 0) {// put the detected point(s) on the right edge of a bounding box with width = cell_size
            max_y += cell_size_;
        } else { // put the detected points on the left edge
            min_y -= cell_size_;
        }
    }

    geometry_msgs::Polygon poly;

    poly.points.resize(4);

    poly.points[0].x = min_x;
    poly.points[0].y = min_y;
    poly.points[0].z = max_z;

    poly.points[1].x = max_x;
    poly.points[1].y = min_y;
    poly.points[1].z = max_z;

    poly.points[2].x = max_x;
    poly.points[2].y = max_y;
    poly.points[2].z = max_z;

    poly.points[3].x = min_x;
    poly.points[3].y = max_y;
    poly.points[3].z = max_z;

    return poly;
}

std::string DetectionGrid::describe_cells() {
    stringstream ss;
    //ROS_WARN("%s",describe_bounds().c_str());
    for (int r = 0; r < rows_; r++) {

        for (int c = 0; c < cols_; c++) {
            ss << static_cast<int>(!cells_[r][c].points.empty());
        }
        ss << "\n";
    }

    return ss.str();
}

std::string DetectionGrid::describe_bounds() {
    stringstream ss;
    ss << "DetectionGrid Bounds:\n\t";
    ss << "rows: " << cells_.size() << "\n\t";
    ss << "cols: " << cells_[0].size() << "\n\t";

    return ss.str();
}

int DetectionGrid::get_label_by_id(CellId id) {
    return labels_[id.row][id.col];
}

void DetectionGrid::find_bounds(const vector<geometry_msgs::Point32> &points) {

    // handle empty point cloud
    if (points.empty()){
      min_x_ = 0;
      min_y_ = 0;
      max_x_ = cell_size_;
      max_y_ = cell_size_;
      return;
    }

    min_x_ = std::numeric_limits<float>::max();
    min_y_ = std::numeric_limits<float>::max();
    max_x_ = std::numeric_limits<float>::min();
    max_y_ = std::numeric_limits<float>::min();

    for (int i = 0; i < points.size(); i++ ){
        geometry_msgs::Point32 point = points[i];

        if (point.x > max_x_)
            max_x_ = point.x;

        if (point.x < min_x_)
            min_x_ = point.x;

        if (point.y > max_y_)
            max_y_ = point.y;

        if (point.y < min_y_)
            min_y_ = point.y;

    }

    min_x_ = floor(min_x_);
    min_y_ = floor(min_y_);
    max_x_ = ceil(max_x_ + 1);
    max_y_ = ceil(max_y_ + 1);

    x_offset_ = 0 - min_x_;
    y_offset_ = 0 - min_y_;

    if (min_x_ == max_x_) {
        max_x_ = ceil(min_x_ + cell_size_);
    }

    if (min_y_ == max_y_) {
        max_y_ = ceil(max_y_ + cell_size_);
    }

}

std::string DetectionGrid::describe_labels() {
    stringstream ss;

    ss << "label_grid: \n";
    //ROS_WARN("%s",describe_bounds().c_str());
    for (int r = 0; r < rows_; r++) {

        for (int c = 0; c < cols_; c++) {
            ss << labels_[r][c];
        }
        ss << "\n";
    }

    return ss.str();
}
