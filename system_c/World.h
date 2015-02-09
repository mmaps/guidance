#ifndef WORLD_H
#define WORLD_H

#include <systemc.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

SC_MODULE(World){

  //Ports
  //Inputs from the user: receives true if user moves forward
  sc_in <sc_bit> userMove;
  //Inputs from sensors: Trigger wall ouputs
  sc_in <sc_bit> leftShoulderSensor;
  sc_in <sc_bit> rightShoulderSensor;
  //Outputs to sensor: writes distance to walls
  sc_out<sc_uint<16>> leftWall;
  sc_out<sc_uint<16>> rightWall;
  sc_out<sc_uint<16>> aheadWall;
  sc_out<sc_uint<16>> backWall;

  sc_out<sc_uint<16>> out;

  //Create Le Channels for Le Ports to talk
  //TODO

  //Yocal Local
  int userX = 0;
  int userY = 0;
  int userYaw = 0;

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

  //Process Declarations
  //void get_walls(int node, int* x, int* y);
  //void get_northWall(worldGraph g, worldGraph::vertex_descriptor currentPosition);
  //void get_southWall(worldGraph g, worldGraph::vertex_descriptor currentPosition);
  //void get_eastWall(worldGraph g, worldGraph::vertex_descriptor currentPosition);
  //void get_westWall(worldGraph g, worldGraph::vertex_descriptor currentPosition);
  void get_wall(worldGraph g, worldGraph::vertex_descriptor currentPosition, int userYaw);
  void userMoved(void);

  //Constructor
  SC_CTOR(World){

    worldGraph::vertex_descriptor currentPosition = v0;

    SC_METHOD(userMoved);
      sensitive << userMove;

    SC_METHOD(get_wall);
      sensitive << leftShoulderSensor;
      sensitive << rightShoulderSensor;
    /*
    SC_METHOD(get_southWall);
      sensitive << rightShoulderSensor;
    
    SC_METHOD(get_eastWall);
      sensitive << leftShoulderSensor;

    SC_METHOD(get_westWall);
      sensitive << rightShoulderSensor;
    */
  }
};

#endif

