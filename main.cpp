#include <iostream>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "shortest_path.h"
#include "fluent_api.h"

using std::cout;
using std::endl;
using std::get;
using std::make_tuple;
using std::mt19937;
using std::random_device;
using std::string;
using std::to_string;
using std::tuple;
using std::vector;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

using coderodde::astar;
using coderodde::dijkstra;
using coderodde::DirectedGraphNode;
using coderodde::DirectedGraphWeightFunction;
using coderodde::EuclideanMetric;
using coderodde::HeuristicFunction;
using coderodde::LayoutMap;
using coderodde::Point3D;
using coderodde::find_shortest_path;

/*******************************************************************************
* Randomly selects an element from a vector.                                   *
*******************************************************************************/
template<class T>
T& choose(vector<T>& vec, mt19937& rnd_gen)
{
    uniform_int_distribution<size_t> dist(0, vec.size() - 1);
    return vec[dist(rnd_gen)];
}

/*******************************************************************************
* Creates a random point in a plane.                                           *
*******************************************************************************/
static Point3D<double>* create_random_point(const double xlen,
                                            const double ylen,
                                            mt19937& random_engine)
{
    uniform_real_distribution<double> xdist(0.0, xlen);
    uniform_real_distribution<double> ydist(0.0, ylen);

    return new Point3D<double>(xdist(random_engine),
                               ydist(random_engine),
                               0.0);
}

/*******************************************************************************
* Creates a random directed, weighted graph.                                   *
*******************************************************************************/
static tuple<vector<DirectedGraphNode*>*,
             DirectedGraphWeightFunction*,
             LayoutMap<DirectedGraphNode, double>*>
    create_random_graph(const size_t length,
                        const double area_width,
                        const double area_height,
                        const float arc_load_factor,
                        const float distance_weight,
                        mt19937 random_gen)
{
    vector<DirectedGraphNode*>* p_vector = new vector<DirectedGraphNode*>();
    LayoutMap<DirectedGraphNode, double>* p_layout =
    new LayoutMap<DirectedGraphNode, double>();

    for (size_t i = 0; i < length; ++i)
    {
        DirectedGraphNode* p_node = new DirectedGraphNode(to_string(i));
        p_vector->push_back(p_node);
    }

    for (DirectedGraphNode* p_node : *p_vector)
    {
        Point3D<double>* p_point = create_random_point(area_width,
                                                       area_height,
                                                       random_gen);
        (*p_layout)(p_node) = p_point;
    }

    DirectedGraphWeightFunction* p_wf = new DirectedGraphWeightFunction();
    EuclideanMetric<double> euclidean_metric;

    size_t arcs = arc_load_factor > 0.9 ?
    length * (length - 1) :
    (arc_load_factor < 0.0 ? 0 : size_t(arc_load_factor * length * length));

    while (arcs > 0)
    {
        DirectedGraphNode* p_head = choose(*p_vector, random_gen);
        DirectedGraphNode* p_tail = choose(*p_vector, random_gen);

        Point3D<double>* p_head_point = (*p_layout)(p_head);
        Point3D<double>* p_tail_point = (*p_layout)(p_tail);

        const double cost = euclidean_metric(*p_head_point,
                                             *p_tail_point);


        (*p_wf)(p_tail, p_head) = distance_weight * cost;
        p_tail->connect_to(p_head);

        --arcs;
    }

    return make_tuple(p_vector, p_wf, p_layout);
}

/*******************************************************************************
* Returns the amount of milliseconds since Unix epoch.                         *
*******************************************************************************/
static unsigned long long get_milliseconds()
{
    return duration_cast<milliseconds>(system_clock::now()
                                       .time_since_epoch()).count();
}

/*******************************************************************************
* Checks that a path has all needed arcs.                                      *
*******************************************************************************/
static bool is_valid_path(vector<DirectedGraphNode*>* p_path)
{
    for (size_t i = 0; i < p_path->size() - 1; ++i)
    {
        if (!(*p_path)[i]->is_connected_to((*p_path)[i + 1]))
        {
            return false;
        }
    }

    return true;
}

/*******************************************************************************
* Computes the length (cost) of a path.                                        *
*******************************************************************************/
static double compute_path_length(vector<DirectedGraphNode*>* p_path,
                                  DirectedGraphWeightFunction* p_wf)
{
    double cost = 0.0;

    for (size_t i = 0; i < p_path->size() - 1; ++i)
    {
        cost += (*p_wf)(p_path->at(i), p_path->at(i + 1));
    }

    return cost;
}

/*******************************************************************************
* The demo.                                                                    *
*******************************************************************************/
int main(int argc, const char * argv[]) {
    random_device rd;
    mt19937 random_gen(rd());

    cout << "Building graph..." << endl;

    tuple<vector<DirectedGraphNode*>*,
          DirectedGraphWeightFunction*,
          LayoutMap<DirectedGraphNode, double>*> graph_data =
    create_random_graph(20000,
                        1000.0,
                        700.0,
                        0.001f,
                        1.2f,
                        random_gen);

    DirectedGraphNode *const p_source = choose(*std::get<0>(graph_data),
                                               random_gen);

    DirectedGraphNode *const p_target = choose(*std::get<0>(graph_data),
                                               random_gen);

    cout << "Source: " << *p_source << endl;
    cout << "Target: " << *p_target << endl;

    EuclideanMetric<double> em;

    unsigned long long ta = get_milliseconds();

//        vector<DirectedGraphNode*>* p_path1 = astar(p_source,
//                                                    p_target,
//                                                    *get<1>(graph_data),
//                                                    *get<2>(graph_data),
//                                                    em);
    vector<DirectedGraphNode*>* p_path1 = 
            find_shortest_path<DirectedGraphNode, double>()
            .from(p_source)
            .to(p_target)
            .with_weight_function(get<1>(graph_data))
            .using_astar()
            .with_layout(get<2>(graph_data))
            .with_metric(&em);

    unsigned long long tb = get_milliseconds();

    cout << endl;
    cout << "A* path:" << endl;

    if (!p_path1)
    {
        cout << "No path for A*!" << endl;
        return 0;
    }

    for (DirectedGraphNode* p_node : *p_path1)
    {
        cout << *p_node << endl;
    }

    cout << "Time elapsed: " << tb - ta << " ms." << endl;
    cout << std::boolalpha;
    cout << "Is valid path: " << is_valid_path(p_path1) << endl;
    cout << "Cost: " << compute_path_length(p_path1, get<1>(graph_data)) << endl;

    cout << endl;
    cout << "Dijkstra path:" << endl;

    ta = get_milliseconds();

    vector<DirectedGraphNode*>* p_path2 = 
            find_shortest_path<DirectedGraphNode, double>()
                              .from(p_source)
                              .to(p_target)
                              .with_weight_function(get<1>(graph_data))
                              .using_dijkstras_algorithm();
    tb = get_milliseconds();

    if (!p_path2)
    {
        cout << "No path for Dijkstra's algorithm!" << endl;
        return 0;
    }

    for (DirectedGraphNode* p_node : *p_path2)
    {
        cout << *p_node << endl;
    }

    cout << "Time elapsed: " << tb - ta << " ms." << endl;
    cout << "Is valid path: " << is_valid_path(p_path1) << endl;
    cout << "Cost: " << compute_path_length(p_path2, get<1>(graph_data)) << endl;

    vector<coderodde::DirectedGraphNode*>* p_vec = get<0>(graph_data);

    while (!p_vec->empty())
    {
        delete p_vec->back();
        p_vec->pop_back();
    }

    delete get<0>(graph_data);
    delete get<1>(graph_data);
    delete get<2>(graph_data);

    return 0;
}
