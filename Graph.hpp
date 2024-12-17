/*
 Name: Nathan Hubbell
 Email: nchubbell@crimson.ua.edu
 Course Section: Fall 2024 CS 201
 Homework #: 4
 To Compile: g++ graphDemo.cpp Graph.hpp
 To Run: ./a input1.txt
*/

#ifndef _GRAPH_HPP_
#define _GRAPH_HPP_

#include <iostream>
#include <string>
#include <sstream>
#include <list>
#include <limits>
#include <queue>

using namespace std;

class Vertex {
public:
	bool visited;
	int distance;
	int previous;
	int finish;
	std::list<int> adj;
};

class Graph {
public:
	Graph(int V, int E, std::pair<int, int> *edges) {
		_V = V;
		_E = E;
		vertices = new Vertex[_V];
		for (int i = 0; i < _V; i++) {
                    vertices[i].visited = false;
                    vertices[i].distance = std::numeric_limits<int>::max();
                    vertices[i].previous = -1;
		}
		for (int i = 0; i < _E; i++) {
		    addEdge(edges[i].first, edges[i].second);
		}
	}

	virtual ~Graph() {
		for (int i=0; i<_V; ++i) {
		    auto adj = vertices[i].adj;
		    adj.clear(); // clear list
		}

		delete[] vertices; // delete array of vertices
	}

	int V() {
		return _V;
	}

	int E() {
		return _E;
	}

	void addEdge(int u, int v) {
		vertices[u].adj.push_back(v);
	}

	std::list<int> getEdges(int u) {
		return vertices[u].adj;
	}

	int degree(int u) {
		return vertices[u].adj.size();
	}

	void bfs(int s) {
    for (int i = 0; i < _V; i++) { // initialize vertexes
        vertices[i].visited = false; // set all to not visited
        vertices[i].distance = numeric_limits<int>::max();  // set distance to infinity initially
        vertices[i].previous = -1;  // no previous vertexes
    }

    vertices[s].visited = true; // set source to true
    vertices[s].distance = 0;  // distance to source is 0
    vertices[s].previous = -1;  // no previous vertex 
    
    // create queue
    queue<int> Q;
    Q.push(s);  // enqueue start vertex

    // continue the rest of bfs process
    while (!Q.empty()) {
        int u = Q.front();  // dequeue front vertex
        Q.pop();  // remove front vertex from the queue

        for (int i : vertices[u].adj) { // go through current vertexes' neighbors
            if (!vertices[i].visited) { // visit unvisited neighbors
                vertices[i].visited = true;  // mark vertex as visited
                vertices[i].distance = vertices[u].distance + 1;  // set distance from start vertex
                vertices[i].previous = u;  // set the previous vertex to current 
                Q.push(i);  // enqueue adjacent vertex
            }
        }
    }
}

void dfs() {
    for (int i = 0; i < _V; i++) {
        vertices[i].visited = false; // set all to not visited
        vertices[i].previous = -1;  // no previous for all vertices
    }

    int time = 0;

    for (int i = 0; i < _V; i++) { 
        if (!vertices[i].visited) { // if vertex not visited
            dfs_visit(i, time);  // start dfs process
        }
    }
}

void dfs_visit(int u, int& time) {
    time++;  // increase time
    vertices[u].distance = time;  // vertex is discovered
    vertices[u].visited = true;  // mark vertex as visited

    cout << "Discovered vertex " << u << endl;

    // visit neighbors of vertex
    for (int v : vertices[u].adj) {
        if (!vertices[v].visited) {
            vertices[v].previous = u;  // set previous vertex to u
            dfs_visit(v, time);  // recursively visit adjacent vertex v
        }
    }

    vertices[u].visited = true;  // ensure vertex is marked visited 

    time++;  // add time after all visited
    vertices[u].finish = time;  // set finish time for the vertex
    cout << "Finished vertex " << u << endl;
}

	void print_path(int s, int v) {
		if (v == s)
		   std::cout << s;
		else if (vertices[v].previous == -1)
		   std::cout << "not connected";
		else {
		   print_path(s,vertices[v].previous);
		   std::cout << "->" << v;
		}
}

	std::string toString() {
		std::stringbuf buffer;
		std::ostream os(&buffer);
		os << "Vertices = " << _V << ", Edges = " << _E << std::endl;
		for (int i = 0; i < _V; ++i) {
		    os << i << "(" << degree(i) << "): ";
		    for (const auto& l : vertices[i].adj) 
			os << l << " ";
		    os << std::endl;
		}

		return buffer.str();
	}
private:
	int _V; // no. of vertices
	int _E; // no. of edges
	Vertex *vertices; // array of vertices
};

#endif
