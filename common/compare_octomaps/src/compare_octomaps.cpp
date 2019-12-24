/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/octomap.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>
#include "ros/ros.h"
#ifdef _MSC_VER // fix missing isnan for VC++
#define isnan(x) _isnan(x)  
#endif

// on MacOS, isnan is in std (also C++11)
using namespace std;

using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " tree1.ot tree2.ot\n\n";

  std::cerr << "Compare two octrees for accuracy / compression.\n\n";

  exit(0);
}

typedef struct volume_t {
	double shared_volume;
	double unshared_volume_different_val; // one tree hasn't incorperated this volume (i.e., one tree treats the volume as unknown)
	double unshared_volume_unknown_to_one; //  tree's don't have the same occupancy/free interpretation of the volume

	double free_free = 0;
	double free_occupied = 0;
	double free_unknown = 0;
	double occupied_free = 0;
	double occupied_occupied = 0;
	double occupied_unknown = 0;
	double unknown_free = 0;
	double unknown_occupied = 0;
	double uknown_unknown = 0;
	double total_volume = 0;
} Volume;

Volume calc_volume_overlap(OcTree* tree1, OcTree* tree2){
	typedef struct coord_volume_t {
		octomap::point3d coordinates;
		double volume;
	} CoordVolume;

	vector<CoordVolume> first_tree_coord_volume_vec, second_tree_coord_volume_vec;
	double shared_volume = 0;
	double unshared_volume_unknown_to_one = 0; // one tree hasn't incorperated this volume (i.e., one tree treats the volume as unknown)
	double unshared_volume_different_val = 0;  //  tree's don't have the same occupancy/free interpretation of the volume
	Volume volume_;


	// get all coordinates associated with each tree
	for (OcTree::leaf_iterator it = tree1->begin_leafs(),
      end = tree1->end_leafs();  it != end; ++it){

		first_tree_coord_volume_vec.push_back(CoordVolume{it.getCoordinate(), pow(it.getSize(),3)});
    }

    for (OcTree::leaf_iterator it = tree1->begin_leafs(),
      end = tree1->end_leafs();  it != end; ++it){
    	second_tree_coord_volume_vec.push_back(CoordVolume{it.getCoordinate(), pow(it.getSize(),3)});
    }


    //compare the two trees one coordinate at a time
    for (auto &second_tree_coord_volume: second_tree_coord_volume_vec){
    	if (!tree1->search(second_tree_coord_volume.coordinates)) {
    		if (tree2->isNodeOccupied(tree2->search(second_tree_coord_volume.coordinates))) {
    			volume_.unknown_occupied += second_tree_coord_volume.volume;
    			//unshared_volume_unknown_to_one += second_tree_coord_volume.volume;
    		}
    		else {
    			volume_.unknown_free += second_tree_coord_volume.volume;
    		}
    	}
    }

    for (auto &first_tree_coord_volume: first_tree_coord_volume_vec){
    	if (!tree2->search(first_tree_coord_volume.coordinates)) {
    		if (tree1->isNodeOccupied(tree1->search(first_tree_coord_volume.coordinates))) {
    			volume_.occupied_unknown += first_tree_coord_volume.volume;
    			//unshared_volume_unknown_to_one += second_tree_coord_volume.volume;
    		}
    		else {
    			volume_.free_unknown += first_tree_coord_volume.volume;
    		}
    	}
    }

    for (auto &coord_volume: first_tree_coord_volume_vec){
    	auto tree2_node = tree2->search(coord_volume.coordinates);
    	if (!tree2_node){ //if doesn't exist already counted
    		continue;
    	}
    	auto tree1_node = tree1->search(coord_volume.coordinates);
    	if (tree1->isNodeOccupied(tree1_node) &&  tree2->isNodeOccupied(tree2_node)){
    			volume_.occupied_occupied += coord_volume.volume;
    	}else if (!tree1->isNodeOccupied(tree1_node) &&  tree2->isNodeOccupied(tree2_node)){
    			volume_.free_occupied += coord_volume.volume;
    	}else if (tree1->isNodeOccupied(tree1_node) &&  !tree2->isNodeOccupied(tree2_node)){
    			volume_.occupied_free += coord_volume.volume;
     	}else if (!tree1->isNodeOccupied(tree1_node) &&  !tree2->isNodeOccupied(tree2_node)){
    			volume_.free_free += coord_volume.volume;
    	}
     }

    /*
   //calc overlapping coordinate similarities
    for (auto &coord_volume: first_tree_coord_volume_vec){
    	auto tree1_node = tree1->search(coord_volume.coordinates);
    	bool tree1_node_occupancy = tree1->isNodeOccupied(tree1_node);

    	auto tree2_node = tree2->search(coord_volume.coordinates);
    	if (!tree2_node){ //if doesn't exist already counted
    		continue;
    	}else{
    		bool tree2_node_occupancy = tree2->isNodeOccupied(tree2_node);
    		if (tree1_node_occupancy == tree2_node_occupancy){
    			shared_volume += coord_volume.volume;
    		}else{
    			unshared_volume_different_val += coord_volume.volume;
    		}
    	}
    }
    */

    volume_.total_volume = volume_.free_free + volume_.free_occupied + volume_.free_unknown +
    		volume_.occupied_free + volume_.occupied_occupied + volume_.occupied_unknown +
		   volume_.unknown_free + volume_.unknown_occupied;

    return volume_;
}


double compare_maps_kld(string first_map, string second_map) {

  std::string filename1 = first_map;
  std::string filename2 = second_map;

  cout << "\nReading octree files...\n";

  OcTree* tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
  OcTree* tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));

  if (fabs(tree1->getResolution()-tree2->getResolution()) > 1e-6){
    OCTOMAP_ERROR("Error: Tree resolutions don't match!");
    exit(-1);
  }


  cout << "Expanding octrees... \n";
  // expand both to full resolution:
  tree1->expand();
  tree2->expand();


  if (tree1->getNumLeafNodes() != tree2->getNumLeafNodes()){
      ROS_ERROR_STREAM("Octrees have different size: " << tree1->getNumLeafNodes() << "!=" <<tree2->getNumLeafNodes() << endl);
      ROS_ERROR_STREAM("this can be problematic, but for now we skip this issue. For points that doesn't exist in both map, we simply skip them");
  }

  cout << "Expanded num. leafs: " << tree1->getNumLeafNodes() << endl;

  // check bbx:
  double x1, x2, y1, y2, z1, z2;
  tree1->getMetricSize(x1, y1, z1);
  tree2->getMetricSize(x2, y2, z2);

  if ((fabs(x1-x2) > 1e-6)
      || (fabs(y1-y2) > 1e-6)
      || (fabs(z1-z2) > 1e-6))
  {
    OCTOMAP_WARNING("Trees span over different volumes, results may be wrong\n");
    ROS_ERROR_STREAM("this can be problematic, but for now we skip this issue");
  }

  double kld_sum = 0.0;
  cout << "Comparing trees... \n";
  for (OcTree::leaf_iterator it = tree1->begin_leafs(),
      end = tree1->end_leafs();  it != end; ++it)
  {
    OcTreeNode* n = tree2->search(it.getKey());
    if (!n){
      OCTOMAP_ERROR("Could not find coordinate of 1st octree in 2nd octree\n");
    } else{
      // check occupancy prob:
      double p1 = it->getOccupancy();
      double p2 = n->getOccupancy();
      if (p1 < 0.0 || p1 > 1.0)
        OCTOMAP_ERROR("p1 wrong: %f", p1);
      if (p2 < 0.0 || p2 > 1.0)
        OCTOMAP_ERROR("p2 wrong: %f", p2);

//      if (p1 > 0.1 || p2 > 0.1)
      if (p1 > 0.001 && p2 < 0.001)
        OCTOMAP_WARNING("p2 near 0, p1 > 0 => inf?");
      if (p1 < 0.999 && p2 > 0.999)
         OCTOMAP_WARNING("p2 near 1, p1 < 1 => inf?");

      double kld = 0;
      if (p1 < 0.0001)
        kld =log((1-p1)/(1-p2))*(1-p1);
      else if (p1 > 0.9999)
        kld =log(p1/p2)*p1;
      else
        kld +=log(p1/p2)*p1 + log((1-p1)/(1-p2))*(1-p1);

#if __cplusplus >= 201103L
      if (std::isnan(kld)){
#else
      if (isnan(kld)){
#endif
        OCTOMAP_ERROR("KLD is nan! KLD(%f,%f)=%f; sum = %f", p1, p2, kld, kld_sum);
        exit(-1);
      }

      kld_sum+=kld;

      //if (p1 <)
//      if (fabs(p1-p2) > 1e-6)
//        cout << "diff: " << p1-p2 << endl;
    }


  }

  cout << "KLD: " << kld_sum << endl;



  delete tree1;
  delete tree2;
  
  return kld_sum;
}


void compare_maps(string first_map, string second_map, string mode) {

  std::string filename1 = first_map;
  std::string filename2 = second_map;

  cout << "\nReading octree files...\n";

  OcTree* tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
  OcTree* tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));

  if (fabs(tree1->getResolution()-tree2->getResolution()) > 1e-6){
    OCTOMAP_ERROR("Error: Tree resolutions don't match!");
    exit(-1);
  }


  cout << "Expanding octrees... \n";
  // expand both to full resolution:
  tree1->expand();
  tree2->expand();

  if (mode == "volume_overlap") {
	  auto volume = calc_volume_overlap(tree1, tree2);
	  //std::cout<<"shared_volume"<<volume.shared_volume<<std::endl;;
	  //std::cout<<"unshared_volume unknown to one "<<volume.unshared_volume_unknown_to_one<<std::endl;
	  //std::cout<<"unshared_volume different occupancy perception"<<volume.unshared_volume_different_val<<std::endl;
	  //std::cout<<"total unshared_volume"<<volume.unshared_volume_different_val + volume.unshared_volume_unknown_to_one<<std::endl;
	  std::cout<<"free_free %"<<volume.free_free/volume.total_volume<<std::endl;;
	  std::cout<<"free_occupied %"<<volume.free_occupied/volume.total_volume<<std::endl;;
	  std::cout<<"free_uknown %"<<volume.free_unknown/volume.total_volume<<std::endl;;
	  std::cout<<"occupied_free %"<<volume.occupied_free/volume.total_volume<<std::endl;;
	  std::cout<<"occupied_occupied %"<<volume.occupied_occupied/volume.total_volume<<std::endl;;
	  std::cout<<"occupied_uknown %"<<volume.occupied_unknown/volume.total_volume<<std::endl;;
	  std::cout<<"uknown_free %"<<volume.unknown_free/volume.total_volume<<std::endl;;
	  std::cout<<"unknown_occupied %"<<volume.unknown_occupied/volume.total_volume<<std::endl;;
	  std::cout<<"total_volume"<<volume.total_volume<<std::endl;;
  }else if (mode == "kld"){
	  auto kld_sum = compare_maps_kld(first_map, second_map);
	  std::cout<<"kld_sum"<<kld_sum <<std::endl;
  }else{
	  std::cout<<"mode"<<mode<<"  is note defined"<<endl;
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "compare_octomaps"); 
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::string ns = ros::this_node::getName();
  string first_map, second_map, mode;

  //---get params
  if(!ros::param::get(ns+"/first_map", first_map)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/first_map").c_str());
      return -1; 
  } 

 if(!ros::param::get(ns+"/second_map", second_map)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s", 
              (ns + "/second_map").c_str());
      return -1; 
  } 

  if(!ros::param::get(ns+"/mode", mode)){
      ROS_FATAL("Could not compare octomaps. Parameter missing! Looking for %s",
              (ns + "/mode").c_str());
      return -1;
  }


  ros::Duration(10).sleep();

  ros::Rate loop_rate(10);
  while(ros::ok()){  
      ros::spinOnce();
      //ros::shutdown();   
      loop_rate.sleep();
      compare_maps(first_map, second_map, mode);
      ros::shutdown();
  }
}

