/*main.cpp*/

//
// Prof. Joe Hummel
// U. of Illinois, Chicago
// CS 251: Spring 2020
// Project #07: open street maps, graphs, and Dijkstra's alg
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:  
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <limits>
#include <queue>
#include <utility>
#include <fstream>
#include <stack>
#include "graph.h"
#include "tinyxml2.h"
#include "dist.h"
#include "osm.h"

using namespace std;
using namespace tinyxml2;
double INF = numeric_limits<double>::max();

//////////////////////////////////////////////////////////////////
//
// main
//

class prioritize{
   public:
      bool operator()(const pair<long long, double>& p1,
            const pair<long long, double>& p2) const{
            if(p1.second == p2.second)
               return p1.first>p2.first;
            else
               return p1.second > p2.second;
      }
};

vector<long long> Dijkstra(graph<long long, double>& G, 
  long long startV, 
  map<long long, double>& distances, map<long long, double>& predV)
{
  vector<long long>  visited;
  
  pair<long long, double> curr;
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;
  
  vector<long long> Verticies;
  Verticies = G.getVertices();
  
  set<long long> Neighbors;
  for(long long currentV: Verticies) 
  {
     distances[currentV] = INF; 
     unvisitedQueue.push(make_pair(currentV, INF)); 
      predV[currentV]= 0;
  }
  
  // startV has a distance of 0 from itself
  distances[startV] = 0;
  unvisitedQueue.push(make_pair(startV, 0));
  
  
  while(!unvisitedQueue.empty())
  {
     bool flag = false;
     curr = unvisitedQueue.top();
     unvisitedQueue.pop();
     
     for(long long S: visited){
        if(S == curr.first)
            flag = true;
     }
     
     if(curr.second == INF){
        break;
     }else if(flag == true){
        continue;
     }else{
        visited.push_back(curr.first);
     }
     
     Neighbors = G.neighbors(curr.first);
     
     for(long long adjV: Neighbors)
     {
        double weight;
        G.getWeight(curr.first, adjV, weight);
        double alternativePathDistance;
        
        alternativePathDistance = distances[curr.first] + weight;
        
        if(alternativePathDistance <  distances[adjV])
        {
           distances[adjV] = alternativePathDistance;
           unvisitedQueue.push(make_pair(adjV, alternativePathDistance));
           predV[adjV] = curr.first;
        }
     }
  }
        

  return visited;
}
int main()
{
  map<long long, Coordinates>  Nodes;     // maps a Node ID to it's coordinates (lat, lon)
  vector<FootwayInfo>          Footways;  // info about each footway, in no particular order
  vector<BuildingInfo>         Buildings; // info about each building, in no particular order
  XMLDocument                  xmldoc;
  graph<long long, double>     G;         
  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "")
  {
    filename = def_filename;
  }

  //
  // Load XML-based map file 
  //
  if (!LoadOpenStreetMap(filename, xmldoc))
  {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }
  
  //
  // Read the nodes, which are the various known positions on the map:
  //
  unsigned int nodeCount = ReadMapNodes(xmldoc, Nodes);
   
  //
  // Read the footways, which are the walking paths:
  //
  unsigned int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  unsigned int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == Nodes.size());
  assert(footwayCount == Footways.size());
  assert(buildingCount == Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  //
  // TODO: build the graph, output stats:
  //
  for (auto& elem:Nodes){
      G.addVertex(elem.first);
  }
  int fsize = Footways.size();
  double lat1,lat2,lon1,lon2, wegt;
  for (int i = 0; i < fsize; i++){
      for (unsigned int j = 0; j < Footways[i].Nodes.size()-1;j++){
          lat1 = Nodes[Footways[i].Nodes[j]].Lat;
          lon1 = Nodes[Footways[i].Nodes[j]].Lon;
          lat2 = Nodes[Footways[i].Nodes[j+1]].Lat;
          lon2 = Nodes[Footways[i].Nodes[j+1]].Lon;
          wegt = distBetween2Points(lat1,lon1,lat2,lon2);
          G.addEdge(Footways[i].Nodes[j], Footways[i].Nodes[j+1], wegt);
          G.addEdge(Footways[i].Nodes[j+1], Footways[i].Nodes[j], wegt);
          }
  }
  
  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;
  
  //
  // Navigation from building to building
  //
  string startBuilding, destBuilding;

  cout << "Enter start (partial name or abbreviation), or #> ";
  getline(cin, startBuilding);

  while (startBuilding != "#")
  {
    cout << "Enter destination (partial name or abbreviation)> ";
    getline(cin, destBuilding);
    
    //
    // TODO: lookup buildings, find nearest start and dest nodes,
    // run Dijkstra's alg, output distance and path to destination:
    //
    double startlat, startlon, destlat, destlon;
    double nlat, nlon, sdistance, ddistance; 
    map<double,long long>startDistances;
    map<double,long long>destDistances;
    vector<long long> path;
    string startfullname, destfullname,startabbrev,destabbrev;
    int bsize = Buildings.size();
    bool startFound=false;
    bool destFound = false;
    for (int i = 0; i < bsize; i++){
        if(Buildings[i].Abbrev == startBuilding){
           startlat = Buildings[i].Coords.Lat;
           startlon = Buildings[i].Coords.Lon;
           startFound = true;
           startfullname = Buildings[i].Fullname;
           startabbrev = Buildings[i].Abbrev;
        }
    }
    vector<BuildingInfo>::iterator it;
    if (startFound == false){
        for(int i = 0; i < bsize; i++){
            if(Buildings[i].Fullname.find(startBuilding)!=string::npos){
            startlat = Buildings[i].Coords.Lat;
            startlon = Buildings[i].Coords.Lon;
            startFound = true;
            startfullname = Buildings[i].Fullname;
            startabbrev = Buildings[i].Abbrev;
        }
        }
    }
    if (startFound==true){
        for (int i = 0; i < bsize; i++){
           if(Buildings[i].Abbrev == destBuilding){
           destlat = Buildings[i].Coords.Lat;
           destlon = Buildings[i].Coords.Lon;
           destFound = true;
           destfullname = Buildings[i].Fullname;
           destabbrev = Buildings[i].Abbrev;
        }
    }
    }
    if (startFound==true && destFound == false){
        for (int i = 0; i < bsize; i++){
           if((Buildings[i].Fullname).find(destBuilding)!=string::npos){
           destlat = Buildings[i].Coords.Lat;
           destlon = Buildings[i].Coords.Lon;
           destFound = true;
           destfullname = Buildings[i].Fullname;
           destabbrev = Buildings[i].Abbrev;
        }
    }
    }
    if(startFound==false){
        cout << "Start building not found" << endl<<endl;
        cout << "Enter start (partial name or abbreviation), or #> ";
        getline(cin, startBuilding);
    }
    else if(destFound==false){
        cout << "Destination building not found" << endl<<endl;
        cout << "Enter start (partial name or abbreviation), or #> ";
        getline(cin, startBuilding);
    }
    else if(startFound && destFound){
        cout<<"Starting point: "<<endl<<" "<<startfullname<<endl;
        cout<<" ("<<startlat<<", "<<startlon<<")"<<endl;
        cout<<"Destination point: "<<endl<<" "<<destfullname<<endl;
        cout<<" ("<<destlat<<", "<<destlon<<")"<<endl;
        for(unsigned int i = 0; i < Footways.size();i++){
            for (unsigned int j = 0; j  < Footways[i].Nodes.size();j++){
                nlat = Nodes[Footways[i].Nodes[j]].Lat;
                nlon = Nodes[Footways[i].Nodes[j]].Lon;
                sdistance = distBetween2Points(startlat, startlon, nlat, nlon);
                startDistances[sdistance] = Footways[i].Nodes[j];
                ddistance = distBetween2Points(destlat, destlon, nlat,nlon);
                destDistances[ddistance] = Footways[i].Nodes[j];
            }
        }
        long long startNodeID, destNodeID;
        map<long long, double> distances;
        vector<long long> visited;
        map<long long, double> predV;
        stack<long long> path;
        startNodeID = startDistances.begin()->second;
        cout<<endl<<"Nearest start node: "<<endl;
        cout<<" "<<Nodes[startNodeID].ID<<endl;
        cout<<" ("<<Nodes[startNodeID].Lat<<", "<<Nodes[startNodeID].Lon<<")"<<endl;
        destNodeID = destDistances.begin()->second;
        cout<<"Nearest destination node: "<<endl;
        cout<<" "<<Nodes[destNodeID].ID<<endl;
        cout<<" ("<<Nodes[destNodeID].Lat<<", "<<Nodes[destNodeID].Lon<<")"<<endl;
        cout<<endl<<"Navigating with Dijkstra..."<<endl;
        visited = Dijkstra(G, Nodes[startNodeID].ID, distances, predV);
        if (distances[Nodes[destNodeID].ID]!=INF){
            cout<<"Distance to dest: "<<distances[Nodes[destNodeID].ID];
            cout<<" miles"<<endl;
            cout<<"Path: ";
            long long x = destNodeID;
            while (x!=startNodeID){
                path.push(x);
                x = predV.at(x);
            }
            path.push(startNodeID);
            cout<<path.top();
            path.pop();
            while (!path.empty()){
                long long pathNodes = path.top();
                cout<<"->";
                cout<<pathNodes;
                path.pop();
            }
            cout<<endl;
        }
        else{
            cout<<"Sorry, destination unreachable"<<endl;
        }
        cout <<endl<< "Enter start (partial name or abbreviation), or #> ";
        getline(cin, startBuilding);
    } 
    
    
    //
    // another navigation?
    //
    
  }

  //
  // done:
  //
  cout << "** Done **" << endl;

  return 0;
}
