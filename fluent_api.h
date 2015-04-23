#ifndef FLUENT_API_H
#define	FLUENT_API_H

#include "shortest_path.h"

namespace coderodde {
    
    template<class NodeType, class FloatType>
    class MetricSelector {
    public:
        
        void set_source(NodeType* p_source)
        {
            this->p_source = p_source;
        }
        
        void set_target(NodeType* p_target)
        {
            this->p_target = p_target;
        }
        
        void set_weight_function(coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf)
        {
            this->p_wf = p_wf;
        }
        
        void set_layout(coderodde::LayoutMap<NodeType, FloatType>* p_layout)
        {
            this->p_layout = p_layout;
        }
        
        std::vector<NodeType*>* with_metric(coderodde::AbstractMetric<FloatType>* p_metric)
        {
            return coderodde::astar(p_source,
                                    p_target,
                                    *p_wf,
                                    *p_layout,
                                    *p_metric);
        }
        
    private:
        NodeType* p_source;
        NodeType* p_target;
        coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf;
        coderodde::LayoutMap<NodeType, FloatType>* p_layout;
    };
    
    template<class NodeType, class FloatType>
    class LayoutSelector {
    public:
        void set_source(NodeType* p_source)
        {
            this->p_source = p_source;
        }
        
        void set_target(NodeType* p_target)
        {
            this->p_target = p_target;
        }
        
        void set_weight_function(coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf)
        {
            this->p_wf = p_wf;
        }
        
        MetricSelector<NodeType, FloatType>& 
        with_layout(coderodde::LayoutMap<NodeType, FloatType>* p_layout)
        {
            metric_selector.set_source(p_source);
            metric_selector.set_target(p_target);
            metric_selector.set_weight_function(p_wf);
            metric_selector.set_layout(p_layout);
            return metric_selector;
        }
        
    private:
        NodeType* p_source;
        NodeType* p_target;
        coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf;
        coderodde::LayoutMap<NodeType, FloatType>* p_layout;
        MetricSelector<NodeType, FloatType> metric_selector;
    };
    
    template<class NodeType, class FloatType>
    class AlgorithmSelector {
    public:
        void set_source(NodeType* p_source) 
        {
            this->p_source = p_source;
        }
        
        void set_target(NodeType* p_target) 
        {
            this->p_target = p_target;
        }
        
        void set_weight_function(coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf) 
        {
            this->p_wf = p_wf;
        }
        
        std::vector<NodeType*>* using_dijkstras_algorithm()
        {
            return coderodde::dijkstra(p_source, p_target, *p_wf);
        }
        
        std::vector<NodeType*>* using_bidirectional_dijkstras_algorithm() 
        {
            return coderodde::bidirectional_dijkstra(p_source, p_target, *p_wf);
        }
        
        LayoutSelector<NodeType, FloatType>& using_astar() 
        {
            layout_selector.set_source(p_source);
            layout_selector.set_target(p_target);
            layout_selector.set_weight_function(p_wf);
            return layout_selector;
        }
        
    private:
        NodeType* p_source;
        NodeType* p_target;
        coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf;
        LayoutSelector<NodeType, FloatType> layout_selector;
    };
    
    template<class NodeType, class FloatType>
    class WeightFunctionSelector {
    public:
        
        void set_source(NodeType* p_source) 
        {
            this->p_source = p_source;
        }
        
        void set_target(NodeType* p_target) 
        {
            this->p_target = p_target;
        }
        
        AlgorithmSelector<NodeType, FloatType>
        with_weight_function(coderodde::AbstractWeightFunction<NodeType, FloatType>* p_wf) 
        {
            algorithm_selector.set_source(p_source);
            algorithm_selector.set_target(p_target);
            algorithm_selector.set_weight_function(p_wf);
            return algorithm_selector;
        }
        
    private:
        AlgorithmSelector<NodeType, FloatType> algorithm_selector;
        NodeType* p_source;
        NodeType* p_target;
    };
    
    template<class NodeType, class FloatType>
    class TargetSelector {
    public:
        WeightFunctionSelector<NodeType, FloatType>& to(NodeType* p_target) 
        {
            weight_function_selector.set_source(p_source);
            weight_function_selector.set_target(p_target);
            return weight_function_selector;
        }
        
        void set_source(NodeType* p_source)
        {
            this->p_source = p_source;
        }
        
    private:
        WeightFunctionSelector<NodeType, FloatType> weight_function_selector;
        NodeType* p_source;
    };
    
    template<class NodeType, class FloatType>
    class SourceSelector {
    public:
        TargetSelector<NodeType, FloatType>& from(NodeType* p_source)
        {
            target_selector.set_source(p_source);
            return target_selector;
        }
            
    private:
        TargetSelector<NodeType, FloatType> target_selector;      
    };
    
    template<class NodeType, class FloatType>
    SourceSelector<NodeType, FloatType>& find_shortest_path()
    {
        static SourceSelector<NodeType, FloatType> source_selector;
        
        return source_selector;
    }
}

#endif // FLUENT_API_H 