#ifndef SHORTEST_PATH_H
#define	SHORTEST_PATH_H

#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace coderodde {

    template<class NodeType>
    class AbstractGraphNode {
    protected:
        
        using Set = std::unordered_set<NodeType*>;
        
    public:

        AbstractGraphNode(std::string name) : m_name{name} {}
        virtual void connect_to(NodeType* other) = 0;
        virtual bool is_connected_to(NodeType* other) const = 0;
        virtual void disconnect_from(NodeType* other) = 0;
        virtual typename Set::iterator begin() const = 0;
        virtual typename Set::iterator end() const = 0;

        class ParentIterator {
        public:
            
            ParentIterator() : mp_set{nullptr} {}
            
            typename Set::iterator begin() 
            {
                return mp_set->begin();
            }
            
            typename Set::iterator end()  
            {
                return mp_set->end();
            }
            
            void set_list(Set* p_list)
            {
                this->mp_set = p_list;
            }
            
        private:
            
            std::unordered_set<NodeType*>* mp_set;
        };

        virtual ParentIterator* parents() = 0;

        bool operator==(const NodeType& other) const
        {
            return m_name == other.m_name;
        }

        std::string& get_name() {return m_name;}
        
    protected:

        std::string m_name;
    };

    template<class T, class FloatType = double>
    class AbstractWeightFunction {
    public:

        virtual FloatType& operator()(T* p_node1, T* p_node2) = 0;
    };

    template<class FloatType>
    class Point3D {
    private:
        const FloatType m_x;
        const FloatType m_y;
        const FloatType m_z;

    public:
        Point3D(const FloatType x = FloatType(),
                const FloatType y = FloatType(),
                const FloatType z = FloatType())
                                    :
                                    m_x{x},
                                    m_y{y},
                                    m_z{z} {}

        FloatType x() const {return m_x;}
        FloatType y() const {return m_y;}
        FloatType z() const {return m_z;}
    };

    template<class FloatType>
    class AbstractMetric {
    public:

        virtual FloatType operator()(coderodde::Point3D<FloatType>& p1,
                                     coderodde::Point3D<FloatType>& p2) = 0;
    };

    template<class FloatType>
    class EuclideanMetric : public coderodde::AbstractMetric<FloatType> {
    public:

        FloatType operator()(coderodde::Point3D<FloatType>& p1,
                             coderodde::Point3D<FloatType>& p2) {
            const FloatType dx = p1.x() - p2.x();
            const FloatType dy = p1.y() - p2.y();
            const FloatType dz = p1.z() - p2.z();

            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    };

    template<class T, class FloatType = double>
    class LayoutMap {
    public:

        virtual coderodde::Point3D<FloatType>*& operator()(T* key)
        {
            return m_map[key];
        }

        ~LayoutMap() 
        {
            typedef typename std::unordered_map<T*, 
                             coderodde::Point3D<FloatType>*>::iterator it_type;
            for (it_type iterator = m_map.begin(); 
                    iterator != m_map.end(); iterator++)
            {
                delete iterator->second;
            }
        }

    private:

        std::unordered_map<T*, coderodde::Point3D<FloatType>*> m_map;
    };

    template<class NodeType, class DistanceType = double>
    class HeapNode {
    public:
        HeapNode(NodeType* p_node, DistanceType distance) :
            mp_node{p_node},
            m_distance{distance} {}

        NodeType* get_node()
        {
            return mp_node;
        }

        DistanceType get_distance()
        {
            return m_distance;
        }

    private:
        NodeType*    mp_node;
        DistanceType m_distance;
    };

    template<class NodeType, class DistanceType = double>
    class HeapNodeComparison {
    public:

        bool operator()(HeapNode<NodeType, DistanceType>* p_first,
                        HeapNode<NodeType, DistanceType>* p_second)
        {
            return p_first->get_distance() > p_second->get_distance();
        }
    };

    template<class NodeType, class FloatType = double>
    class DistanceMap {
    public:

        FloatType& operator()(const NodeType* p_node)
        {
            return m_map[p_node];
        }

    private:

        std::unordered_map<const NodeType*, FloatType> m_map;
    };

    template<class NodeType>
    class ParentMap {
    public:

        NodeType*& operator()(const NodeType* p_node)
        {
            return m_map[p_node];
        }

        bool has(NodeType* p_node)
        {
            return m_map.find(p_node) != m_map.end();
        }

    private:

        std::unordered_map<const NodeType*, NodeType*> m_map;
    };

    template<class NodeType>
    std::vector<NodeType*>* traceback_path(NodeType* p_touch,
                                           ParentMap<NodeType>* parent_map1,
                                           ParentMap<NodeType>* parent_map2 = nullptr)
    {
        std::vector<NodeType*>* p_path = new std::vector<NodeType*>();
        NodeType* p_current = p_touch;

        while (p_current != nullptr)
        {
            p_path->push_back(p_current);
            p_current = (*parent_map1)(p_current);
        }

        std::reverse(p_path->begin(), p_path->end());
        
        if (parent_map2 != nullptr)
        {
            p_current = (*parent_map2)(p_touch);
            
            while (p_current != nullptr)
            {
                p_path->push_back(p_current);
                p_current = (*parent_map2)(p_current);
            }
        }
        
        return p_path;
    }
    
    template<class T, class FloatType = double>
    class HeuristicFunction {

    public:
        HeuristicFunction(T* p_target_element,
                          LayoutMap<T, FloatType>& layout_map,
                          AbstractMetric<FloatType>& metric)
        :
        mp_layout_map{&layout_map},
        mp_metric{&metric},
        mp_target_point{layout_map(p_target_element)}
        {

        }

        FloatType operator()(T* element)
        {
            return (*mp_metric)(*(*mp_layout_map)(element), *mp_target_point);
        }

    private:
        coderodde::LayoutMap<T, FloatType>*   mp_layout_map;
        coderodde::AbstractMetric<FloatType>* mp_metric;
        coderodde::Point3D<FloatType>*        mp_target_point;
    };

    template<class NodeType, class WeightType = double>
    std::vector<NodeType*>* 
    astar(NodeType* p_source,
          NodeType* p_target,
          coderodde::AbstractWeightFunction<NodeType, WeightType>& w,
          coderodde::LayoutMap<NodeType, WeightType>& layout_map,
          coderodde::AbstractMetric<WeightType>& metric)
    {
        std::priority_queue<HeapNode<NodeType, WeightType>*,
                            std::vector<HeapNode<NodeType, WeightType>*>,
                            HeapNodeComparison<NodeType, WeightType>> OPEN;

        std::unordered_set<NodeType*> CLOSED;

        coderodde::HeuristicFunction<NodeType,
                                     WeightType> h(p_target,
                                                   layout_map,
                                                   metric);
        DistanceMap<NodeType, WeightType> d;
        ParentMap<NodeType> p;

        OPEN.push(new HeapNode<NodeType, WeightType>(p_source, WeightType(0)));
        p(p_source) = nullptr;
        d(p_source) = WeightType(0);
        
        while (!OPEN.empty())
        {
            HeapNode<NodeType, WeightType>* p_heap_node = OPEN.top();
            NodeType* p_current = p_heap_node->get_node();
            OPEN.pop();
            delete p_heap_node;

            if (*p_current == *p_target)
            {
                // Found the path.
                return traceback_path(p_target, &p);
            }

            CLOSED.insert(p_current);

            // For each child of 'p_current' do...
            for (NodeType* p_child : *p_current)
            {
                
                if (CLOSED.find(p_child) != CLOSED.end())
                {
                    // The optimal distance from source to p_child is known.
                    continue;
                }

                WeightType cost = d(p_current) + w(p_current, p_child);

                if (!p.has(p_child) || cost < d(p_child))
                {
                    WeightType f = cost + h(p_child);
                    OPEN.push(new HeapNode<NodeType, WeightType>(p_child, f));
                    d(p_child) = cost;
                    p(p_child) = p_current;
                }
            }
        }

        // p_target not reachable from p_source.
        return nullptr;
    }

    template<class T, class FloatType>
    class ConstantLayoutMap : public coderodde::LayoutMap<T, FloatType> {
    public:
        
        ConstantLayoutMap() : mp_point{new Point3D<FloatType>()} {}
        
        ~ConstantLayoutMap() 
        {
            delete mp_point;
        }
        
        Point3D<FloatType>*& operator()(T* key)
        {
            return mp_point;
        }
            
    private:
        
        Point3D<FloatType>* mp_point;
    };
    
    /***************************************************************************
    * This function template implements Dijkstra's shortest path algorithm.    *
    ***************************************************************************/
    template<class NodeType, class WeightType = double>
    std::vector<NodeType*>*
    dijkstra(NodeType* p_source,
             NodeType* p_target,
             coderodde::AbstractWeightFunction<NodeType, WeightType>& w)
    {
        ConstantLayoutMap<NodeType, WeightType> layout;
        EuclideanMetric<WeightType> metric;
        
        return astar(p_source,
                     p_target,
                     w,
                     layout,
                     metric);
    }

    template<class NodeType, class WeightType = double>
    std::vector<NodeType*>*
    bidirectional_dijkstra(
        NodeType* p_source,
        NodeType* p_target,
        coderodde::AbstractWeightFunction<NodeType, WeightType>& w) 
    {
        std::priority_queue<HeapNode<NodeType, WeightType>*,
                            std::vector<HeapNode<NodeType, WeightType>*>,
                            HeapNodeComparison<NodeType, WeightType>> OPENA;
        
        std::priority_queue<HeapNode<NodeType, WeightType>*,
                            std::vector<HeapNode<NodeType, WeightType>*>,
                            HeapNodeComparison<NodeType, WeightType>> OPENB;
        
        std::unordered_set<NodeType*> CLOSEDA;
        std::unordered_set<NodeType*> CLOSEDB;
        
        DistanceMap<NodeType, WeightType> DISTANCEA;
        DistanceMap<NodeType, WeightType> DISTANCEB;
        
        ParentMap<NodeType> PARENTA;
        ParentMap<NodeType> PARENTB;
        
        OPENA.push(new HeapNode<NodeType, WeightType>(p_source, 0.0));
        OPENB.push(new HeapNode<NodeType, WeightType>(p_target, 0.0));
        
        DISTANCEA(p_source) = WeightType(0);
        DISTANCEB(p_target) = WeightType(0);
        
        PARENTA(p_source) = nullptr;
        PARENTB(p_target) = nullptr;
        
        NodeType* p_touch = nullptr;
        WeightType best_cost = std::numeric_limits<WeightType>::max();
        
        while (!OPENA.empty() && !OPENB.empty())
        {
            if (OPENA.top()->get_distance() + 
                OPENB.top()->get_distance() >= best_cost)
            {
                return traceback_path(p_touch, &PARENTA, &PARENTB);
            }
            
            if (OPENA.top()->get_distance() < OPENB.top()->get_distance())
            {
                HeapNode<NodeType, WeightType>* p_heap_node = OPENA.top();
                NodeType* p_current = p_heap_node->get_node();
                OPENA.pop();
                delete p_heap_node;
                
                CLOSEDA.insert(p_current);
                
                for (NodeType* p_child : *p_current)
                {
                    if (CLOSEDA.find(p_child) != CLOSEDA.end()) 
                    {
                        continue;
                    }
                    
                    WeightType g = DISTANCEA(p_current) + w(p_current, p_child);
                    
                    if (!PARENTA.has(p_child) || g < DISTANCEA(p_current))
                    {
                        OPENA.push(new HeapNode<NodeType, 
                                                WeightType>(p_child, g));
                        DISTANCEA(p_child) = g;
                        PARENTA(p_child) = p_current;
                        
                        if (CLOSEDB.find(p_child) != CLOSEDB.end())
                        {
                            WeightType path_len = g + DISTANCEB(p_child);
                            
                            if (best_cost > path_len)
                            {
                                best_cost = path_len;
                                p_touch = p_child;
                            }
                        }
                    }
                }
            } 
            else
            {
                HeapNode<NodeType, WeightType>* p_heap_node = OPENB.top();
                NodeType* p_current = p_heap_node->get_node();
                OPENB.pop();
                delete p_heap_node;
                
                CLOSEDB.insert(p_current);
                
                typename coderodde::AbstractGraphNode<NodeType>::ParentIterator*
                        p_iterator = p_current->parents();
                
                for (NodeType* p_parent : *p_iterator)
                {
                    if (CLOSEDB.find(p_parent) != CLOSEDB.end()) 
                    {
                        continue;
                    }
                    
                    WeightType g = DISTANCEB(p_current) + 
                                   w(p_parent, p_current);
                    
                    if (!PARENTB.has(p_parent) || g < DISTANCEB(p_parent))
                    {
                        OPENB.push(new HeapNode<NodeType, 
                                                WeightType>(p_parent, g));
                        DISTANCEB(p_parent) = g;
                        PARENTB(p_parent) = p_current;
                        
                        if (CLOSEDA.find(p_parent) != CLOSEDA.end())
                        {
                            WeightType path_len = g + DISTANCEA(p_parent);
                            
                            if (best_cost > path_len)
                            {
                                best_cost = path_len;
                                p_touch = p_parent;
                            }
                        }
                    }
                }
            }
        }
        
        return nullptr;
    }
    
    class DirectedGraphNode : public coderodde::AbstractGraphNode<DirectedGraphNode> {
    public:

        DirectedGraphNode(std::string name) :
                coderodde::AbstractGraphNode<DirectedGraphNode>(name)
        {
            this->m_name = name;
        }

        void connect_to(coderodde::DirectedGraphNode* p_other)
        {
            m_out.insert(p_other);
            p_other->m_in.insert(this);
        }

        bool is_connected_to(coderodde::DirectedGraphNode* p_other) const
        {
            return m_out.find(p_other) != m_out.end();
        }

        void disconnect_from(coderodde::DirectedGraphNode* p_other)
        {
            m_out.erase(p_other);
            p_other->m_in.erase(this);
        }
        
        ParentIterator* parents() 
        {
            m_iterator.set_list(&m_in);
            return &m_iterator;
        }

        typename Set::iterator begin() const
        {
            return m_out.begin();
        }

        typename Set::iterator end() const
        {
            return m_out.end();
        }

        friend std::ostream& operator<<(std::ostream& out,
                                        DirectedGraphNode& node) 
        {
            return out << "[DirectedGraphNode " << node.get_name() << "]";
        }

    private:
        Set m_in;
        Set m_out;
        ParentIterator m_iterator;
    };

    class DirectedGraphWeightFunction :
    public AbstractWeightFunction<coderodde::DirectedGraphNode, double> {

    public:

        double& operator()(coderodde::DirectedGraphNode* node1,
                           coderodde::DirectedGraphNode* node2)
        {
            if (m_map.find(node1) == m_map.end())
            {
                m_map[node1] =
                new std::unordered_map<coderodde::DirectedGraphNode*,
                                       double>();
            }

            return (*m_map.at(node1))[node2];
        }

    private:

        std::unordered_map<coderodde::DirectedGraphNode*,
        std::unordered_map<coderodde::DirectedGraphNode*, double>*> m_map;
    };
}

#endif // SHORTEST_PATH_H