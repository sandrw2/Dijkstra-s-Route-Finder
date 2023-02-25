// Digraph.hpp
//
// ICS 46 Winter 2022
// Project #5: Rock and Roll Stops the Traffic
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph.  The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <exception>
#include <functional>
#include <list>
#include <map>
#include <utility>
#include <vector>

#include <limits>
#include <queue>


// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException : public std::runtime_error
{
public:
    DigraphException(const std::string& reason);
};


inline DigraphException::DigraphException(const std::string& reason)
    : std::runtime_error{reason}
{
}



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a struct template.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a struct template.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The move constructor initializes a new Digraph from an expiring one.
    Digraph(Digraph&& d) noexcept;

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph() noexcept;

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // The move assignment operator assigns the contents of an expiring
    // Digraph into "this" Digraph.
    Digraph& operator=(Digraph&& d) noexcept;

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const noexcept;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const noexcept;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the predecessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.
    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> vertexMap;

    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.
    void DFTr(int vertex, std::vector<int>& visted) const;
};



// You'll need to implement the member functions below.  There's enough
// code in place to make them compile, but they'll all need to do the
// correct thing instead.

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
    //nothing to do here; already empty map
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
    vertexMap = d.vertexMap;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(Digraph&& d) noexcept
{
    vertexMap.swap(d.vertexMap);
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph() noexcept
{
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
    if(this!= &d){
        vertexMap = d.vertexMap;
    }
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(Digraph&& d) noexcept
{
    if(this!= &d){
        vertexMap.swap(d.vertexMap);
    }
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
    std::vector<int> verticies;
    for (auto it = vertexMap.begin(); it != vertexMap.end(); it++){
        verticies.push_back(it -> first);
    }
    return verticies;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    std::vector<std::pair<int, int>> edges;
    for(auto it = vertexMap.begin(); it != vertexMap.end(); it++){
        for(auto n = it-> second.edges.begin(); n!= it-> second.edges.end(); n++){
            edges.push_back(std::make_pair(n-> fromVertex, n->toVertex));
        }
    }
    return edges;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
    std::vector<std::pair<int, int>> edges;
    auto it = vertexMap.find(vertex);
    if(it!=vertexMap.end()){
        for(auto n = it -> second.edges.begin(); n!=it -> second.edges.end(); n++){
            edges.push_back(std::make_pair(n-> fromVertex, n->toVertex));
        }    
    }else{
        throw DigraphException("Invalid vertex");
    }
    return edges;
}


template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    auto it = vertexMap.find(vertex);
    if(it != vertexMap.end()){
        return it -> second.vinfo;
    }else{
        throw DigraphException("Invalid Vertex");
    }
}


template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    auto it = vertexMap.find(fromVertex);
    auto n = vertexMap.find(toVertex);
    if(it != vertexMap.end() && n!= vertexMap.end()){
        for(auto n = it-> second.edges.begin(); n!= it-> second.edges.end(); n++){
            if(n -> fromVertex == fromVertex && n -> toVertex == toVertex){
                return n-> einfo;
            }
        }
        throw DigraphException("Edge Does Not Exist");
    }else{
        throw DigraphException("Invalid Vertex");
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
    DigraphVertex<VertexInfo, EdgeInfo> diVertex;
    diVertex.vinfo = vinfo;
    auto newPair = std::make_pair(vertex, diVertex);
    auto returnVal = vertexMap.insert(newPair);
    if(returnVal.second == false){
        throw DigraphException("Vertex already exists");
    }
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{
    DigraphEdge<EdgeInfo> newEdge;
    newEdge.fromVertex = fromVertex;
    newEdge.toVertex = toVertex;
    newEdge.einfo = einfo;
    auto newPair = std::make_pair(fromVertex, toVertex);
    if(vertexMap.count(fromVertex) == 0 || vertexMap.count(toVertex) == 0){
        throw DigraphException("Edge uses Invalid Vertex");
    }
    auto allEdges = edges();
    if(std::find(allEdges.begin(), allEdges.end(), newPair) == allEdges.end()){
        vertexMap[fromVertex].edges.push_back(newEdge);
    }else{
        throw DigraphException("Edge already exists");
    }
    
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
    auto it = vertexMap.find(vertex);
    if(it != vertexMap.end()){
        vertexMap.erase(it);
    }else{
        throw DigraphException("Invalid Vertex");
    }
    
    
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
    if(vertexMap.count(fromVertex) == 0 || vertexMap.count(toVertex) == 0){
        throw DigraphException("Invalid Vertex");
    }else{
        auto it = vertexMap.find(fromVertex);
        for(auto n = it-> second.edges.begin(); n!= it-> second.edges.end(); n++){
            if(n -> fromVertex == fromVertex && n -> toVertex == toVertex){
                it-> second.edges.erase(n);
                return;
            }
        }
        throw DigraphException("Edge Does Not Exist");
    }
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const noexcept
{
    return vertexMap.size();
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const noexcept
{
    return edges().size();
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
    auto it = vertexMap.find(vertex);
    if(it!= vertexMap.end()){
        return it -> second.edges.size();
    }else{
        throw DigraphException("Invalid vertex");
    }
    
    
}


template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    std::vector<int> visited;
    for(auto it = vertexMap.begin(); it != vertexMap.end(); it++){
        DFTr(it-> first, visited);

        if(visited.size() != vertexMap.size()){
            return false;
        }
        visited.clear();
    }
    return true;
    
}

template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::DFTr(int vertex, std::vector<int>& visited) const{
    //visit 
    if(std::find(visited.begin(), visited.end(), vertex) == visited.end()){
        visited.push_back(vertex);
    }
    //for each vertex w such that the edge v->w exists:
    std::vector<std::pair<int, int>> vertexEdges = edges(vertex);
    for(int i = 0; i<vertexEdges.size(); i++){
        int toVertex = vertexEdges[i].second;
        if(std::find(visited.begin(), visited.end(), toVertex) == visited.end()){
            DFTr(toVertex, visited);
        }
    }


}


template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
    if(vertexMap.count(startVertex) == 0){
        throw DigraphException("Invalid Vertex");
    }
    std::map<int, double> shortestDist;
    std::map<int, int> previous;
    std::map<int, bool> known;


    for(auto it = vertexMap.begin(); it!= vertexMap.end(); it++)
    {
        //set k to false --> {vertex: false}
        //set p to unknown (or none, if v is the start vertex) {vertex: -1}
        //set d to ∞ (or 0, if v is the start vertex)
        known.insert(std::make_pair(it->first, false));
        previous.insert(std::make_pair(it->first, it->first));
        shortestDist.insert(std::make_pair(it->first, std::numeric_limits<int>::max()));
    }
    //for startVertex: set p = startVertex, d = 0
    previous[startVertex] = startVertex;
    shortestDist[startVertex] = 0;

    //define lambda function c: distance map
    auto func = [shortestDist](int a, int b) 
    {return shortestDist.find(a)->second > shortestDist.find(b)->second;}; 

    //create queue using func
    std::priority_queue<int, std::vector<int>, decltype(func)> pq(func);
    pq.push(startVertex);

    while(pq.size() != 0){
        //define vertex v to be smallest from queue
        int v = pq.top();
        //remove top element 
        pq.pop();

        //if(k is false)
        if(known[v] == false){
            //set k[v] == true 
            known[v] = true;

            //enqueue its outgoing edges
            //for each vertex w such that v-> w exists
            auto n =  vertexMap.find(v) -> second;
            for(auto it = n.edges.begin(); it!= n.edges.end(); it++){
                int w  = it->toVertex;
                if(shortestDist[w] > shortestDist[v] + edgeWeightFunc(it -> einfo)){
                    shortestDist[w] = shortestDist[v] + edgeWeightFunc(it -> einfo);
                    previous[w] = v;
                    pq.push(w);
                }
            }

        }
    }


    return previous;

}



#endif

