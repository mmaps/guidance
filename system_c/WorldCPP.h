#ifndef WORLD_H
#define WORLD_H

#include <systemc.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

class World{
  
  public:
      //Define the vertex
      struct Vertex{
        int northWall=5;
        int southWall=5;
        int eastWall=NULL;
        int westWall=NULL;
      }
      //Defining the type of graph of world
      typedef boost:adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Vertex> worldGraph;
      worldGraph floorplan;
      //Name and add vertices
      worldGraph::vertex_descriptor v0 = boost::add_vertex(floorplan);
      worldGraph::vertex_descriptor v1 = boost::add_vertex(floorplan);
      worldGraph::vertex_descriptor v2 = boost::add_vertex(floorplan);
      worldGraph::vertex_descriptor v3 = boost::add_vertex(floorplan);
      worldGraph::vertex_descriptor v4 = boost::add_vertex(floorplan);
      //Name and add edges
      bool added;
      boost::tie(worldGraph::edge_descriptor e0, added) = boost::add_edge(v0, v1, floorplan);
      boost::tie(worldGraph::edge_descriptor e1, added) = boost::add_edge(v1, v2, floorplan);
      boost::tie(worldGraph::edge_descriptor e2, added) = boost::add_edge(v2, v3, floorplan);
      boost::tie(worldGraph::edge_descriptor e3, added) = boost::add_edge(v3, v4, floorplan);

      // Declare some methods
      //int* get_wall(worldGraph g, worldGraph::vertex_descriptor currentPosition, int userYaw);
      int* get_wall(int userYaw);

      //Constructor
      World();

};

World::World(){
  worldGraph::vertex_descriptor currentPosition = v0;
}
#endif

