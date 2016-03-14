#ifndef CIRCULARSMOOTHERPARAMETERS_H_
#define CIRCULARSMOOTHERPARAMETERS_H_
#include <openrave/planner.h>

namespace or_circularsmoother {

class CircularSmootherParameters
    : public OpenRAVE::PlannerBase::PlannerParameters {
public:
    CircularSmootherParameters()
        : is_processing_(false)
        , integration_step_(0.001)
        , interpolation_step_(0.01)
    {
        _vXMLParameters.push_back("integration_step");
        _vXMLParameters.push_back("interpolation_step");
    }

    virtual void copy(boost::shared_ptr<PlannerParameters const> r)
    {
        boost::shared_ptr<CircularSmootherParameters const> p
            = boost::dynamic_pointer_cast<CircularSmootherParameters const>(r);
        
        if (p)
        {
            is_processing_ = p->is_processing_;
            integration_step_ = p->integration_step_;
            interpolation_step_ = p->interpolation_step_;
        }

        PlannerParameters::copy(r);
    }

    bool is_processing_;    
    double integration_step_;
    double interpolation_step_;

protected:
    virtual bool serialize(std::ostream &stream) const
    {
        if (!PlannerParameters::serialize(stream))
            return false;

        stream << "<integration_step>" << integration_step_ << "</integration_step>\n"
               << "<interpolation_step>" << interpolation_step_ << "</interpolation_step>\n";

        return !!stream;
    }

    ProcessElement startElement(
        std::string const &name,
        std::list<std::pair<std::string, std::string> > const &atts)
    {
        if (is_processing_)
            return PE_Ignore;

        switch (PlannerParameters::startElement(name, atts))
        {
        case PE_Pass:
            break;
        case PE_Support:
            return PE_Support;
        case PE_Ignore:
            return PE_Ignore;
        }

        is_processing_ =
             name == "integration_step"
          || name == "interpolation_step";
        return is_processing_ ? PE_Support : PE_Pass;
    }

    virtual bool endElement(std::string const &name)
    {
        using boost::format;
        using boost::str;

        if (is_processing_)
        {
            if (name == "integration_step")
            {
                _ss >> time_limit_;
            }
            else if (name == "interpolation_step")
            {
                _ss >> time_limit_;
            }
            else
            {
                RAVELOG_WARN(str(format("Unknown tag '%s'.\n") % name));
            }
            is_processing_ = false;
            return false;
        }

        return PlannerParameters::endElement(name);
    }
};

typedef boost::shared_ptr<CircularSmootherParameters> CircularSmootherParametersPtr;
typedef boost::shared_ptr<CircularSmootherParameters const> CircularSmootherParametersConstPtr;

} // namespace or_circularsmoother

#endif
