#include "World.h"

int* World::get_wall(worldGraph graph, worldGraph::vertex_descriptor position, int yaw){
  int walls [4];
  // I need to make this more elegant
  if(yaw == 0){
    walls[3]=graph[position].northWall;
    walls[1]=graph[position].southWall;
    walls[0]=graph[position].westWall;
    walls[2]=graph[position].southWall;
  }
  if(yaw == 90){
    walls[3]=graph[position].westWall;
    walls[1]=graph[position].eastWall;
    walls[0]=graph[position].northWall;
    walls[2]=graph[position].southWall;
  }
  if(yaw == 180){
    walls[3]=graph[position].southWall;
    walls[1]=graph[position].northWall;
    walls[0]=graph[position].westWall;
    walls[2]=graph[position].eastWall;
  }
  if(yaw == 270){
    walls[3]=graph[position].eastWall;
    walls[1]=graph[position].westWall;
    walls[0]=graph[position].southWall;
    walls[2]=graph[position].northWall;
  }

  return walls;
}
/*
void World::get_northWall(worldGraph graph, worldGraph::vertex_descriptor position){
  out.write(graph[position].northWall);
}
  
void World::get_southWall(worldGraph graph, worldGraph::vertex_descriptor position){
  out.write(graph[position].southWall);
}

void World::get_eastWall(worldGraph graph, worldGraph::vertex_descriptor position){
  out.write(graph[position].southWall);
}

void World::get_westWall(worldGraph graph, worldGraph::vertex_descriptor position){
  out.write(graph[position].southWall);
}
*/
