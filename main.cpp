// main.cpp
//
// ICS 46 Winter 2022
// Project #5: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.

#include "InputReader.hpp"
#include "RoadMap.hpp"
#include "RoadMapReader.hpp"
#include "TripReader.hpp"
#include "Trip.hpp"
#include "TripMetric.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>  

int main()
{
    InputReader ir  = InputReader(std::cin);
	RoadMapReader rmr;
	RoadMap rmap = rmr.readRoadMap(ir);
    
    if(!rmap.isStronglyConnected()){
        std::cout << "Disconnected Map" << std::endl;
        return 0;
    }
    
    TripReader tr;
	std::vector<Trip> trips = tr.readTrips(ir);

    for(int i = 0; i< trips.size(); i++){
        Trip t = trips[i];

        //shortestPath map
        std::map<int,int> shortestPath;
        if(t.metric == TripMetric::Distance){
            shortestPath = rmap.findShortestPaths(t.startVertex, 
            [](RoadSegment rs){return rs.miles;});
        }else{
            shortestPath = rmap.findShortestPaths(t.startVertex, 
            [](RoadSegment rs){return rs.miles/rs.milesPerHour;});
        }
        
        //find shortest path to end Vertex 
        //start with end vertex and popcorn style read backwards 
        std::vector<int> orderedPath; 
        int nextVertex = shortestPath[t.endVertex];
        orderedPath.insert(orderedPath.begin(), t.endVertex);
        
        while(nextVertex != t.startVertex){
            orderedPath.insert(orderedPath.begin(), nextVertex);
            nextVertex = shortestPath[nextVertex];
        }
        orderedPath.insert(orderedPath.begin(), t.startVertex);

        //print statements=====================================================
        
        if(t.metric == TripMetric::Distance){
            double totalMiles = 0;
            //[PRINT] Shortest distance from [start] to [end]=======================
            std::cout<< "Shortest distance from " + rmap.vertexInfo(t.startVertex) +
            " to " + rmap.vertexInfo(t.endVertex) << std::endl;

            //Begin at [start]
            std::cout <<"  Begin at " + rmap.vertexInfo(t.startVertex) << std::endl;
            //======================================================================

            for(int j = 0; j< orderedPath.size()-1; j++){
                int current = orderedPath[j];
                int next = orderedPath[j+1];

                //Continue to [intermediate] ([x] miles) 
                std::cout << "  Continue to " + rmap.vertexInfo(next) +" (" 
                << std::fixed << std::setprecision(1) 
                << rmap.edgeInfo(current, next).miles << " miles)" << std::endl;   
                //==================================================================
                
                totalMiles += rmap.edgeInfo(current, next).miles;
                
                current = next;
                next = shortestPath[next];
            }

            //Total distance: [x] miles
            std::cout << "Total distance: " 
            << std::fixed << std::setprecision(1) 
            << totalMiles << " miles" <<std::endl;
       
        }else{
            int totalHours = 0;
            int totalMinutes = 0;
            double totalSeconds = 0.0;

            //[PRINT] Shortest distance from [start] to [end]=======================
            std::cout<< "Shortest driving time from " + rmap.vertexInfo(t.startVertex) +
            " to " + rmap.vertexInfo(t.endVertex) << std::endl;

            //Begin at [start]
            std::cout <<"  Begin at " + rmap.vertexInfo(t.startVertex) << std::endl;
            //======================================================================

            for(int j = 0; j<orderedPath.size()-1; j++){
                int current = orderedPath[j];
                int next = orderedPath[j+1];

                //Continue to [intermediate] ([x] miles) ===========================
                double distance = rmap.edgeInfo(current, next).miles;
                double speed = rmap.edgeInfo(current, next).milesPerHour;
                int hours = std::floor(distance/speed);
                int minutes = std::floor(((distance/speed) - hours)*60.0);
                double seconds = ((((distance/speed) - hours)*60.0) - minutes)*60.0;

                if(hours == 0 && minutes!=0){
                    std::cout << "  Continue to " + rmap.vertexInfo(next) +" (" 
                    << std::fixed << std::setprecision(1) 
                    << rmap.edgeInfo(current, next).miles << " miles @ " 
                    << std::fixed << std::setprecision(1)
                    << speed <<"mph = "  
                    << minutes <<" mins "
                    << std::fixed << std::setprecision(1) 
                    << seconds << " secs)"<<std::endl;
                }else if(hours == 0 && minutes == 0){
                    std::cout << "  Continue to " + rmap.vertexInfo(next) +" (" 
                    << std::fixed << std::setprecision(1) 
                    << rmap.edgeInfo(current, next).miles << " miles @ " 
                    << std::fixed << std::setprecision(1)
                    << speed <<"mph = " 
                    << std::fixed << std::setprecision(1) 
                    << seconds << " secs)"<<std::endl;
                }else{
                    std::cout << "  Continue to " + rmap.vertexInfo(next) +" (" 
                    << std::fixed << std::setprecision(1) 
                    << rmap.edgeInfo(current, next).miles << " miles @ " 
                    << std::fixed << std::setprecision(1)
                    << speed <<"mph = " 
                    << hours << " hrs " 
                    << minutes <<" mins "
                    << std::fixed << std::setprecision(1) 
                    << seconds << " secs)"<<std::endl; 
                }  
                //==================================================================
                //add edge time to total time
                totalHours += hours;
                totalMinutes += minutes;
                totalSeconds += seconds;
                
                //reorganize second
                if(totalSeconds >= 60){
                    int addMin = std::floor(totalSeconds/60);
                    //add to total minutes 
                    totalMinutes += addMin;
                    //update seconds to get remaining seconds
                    totalSeconds = totalSeconds - (addMin*60);
                }

                //reorganize minutes 
                if(totalMinutes >= 60){
                    int addHour = std::floor(totalMinutes/60);
                    //add to total Hours 
                    totalHours += addHour;
                    //update seconds to get remaining seconds
                    totalMinutes = totalMinutes - (addHour*60);
                }

                current = next;
                next = shortestPath[next];
            }

            //Total time: ...
            if(totalHours == 0 && totalMinutes != 0){
                std::cout << "Total time: "  
                << totalMinutes <<" mins "
                <<std::fixed << std::setprecision(1)
                << totalSeconds << " secs"<<std::endl;
            }else if(totalHours == 0 && totalMinutes == 0){
                std::cout << "Total time: " 
                <<std::fixed << std::setprecision(1)
                << totalSeconds << " secs"<<std::endl;
            }else{
                std::cout << "Total time: " 
                << totalHours << " hrs " 
                << totalMinutes <<" mins "
                <<std::fixed << std::setprecision(1)
                << totalSeconds << " secs"<<std::endl;
            }
            

        }
        
    }
    return 0;
}

