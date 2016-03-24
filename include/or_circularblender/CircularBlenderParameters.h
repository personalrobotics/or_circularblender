#ifndef CIRCULARBLENDERPARAMETERS_H_
#define CIRCULARBLENDERPARAMETERS_H_

#include <openrave/planner.h>

namespace or_circularblender
{

class CircularBlenderParameters
    : public OpenRAVE::PlannerBase::PlannerParameters
{
public:
    CircularBlenderParameters()
        : check_collision_(false)
        , is_processing_(false)
        , integration_step_(0.01)
        , interpolation_step_(0.01)
        , max_deviation_(0.1)
    {
        _vXMLParameters.push_back("check_collision");
        _vXMLParameters.push_back("integration_step");
        _vXMLParameters.push_back("interpolation_step");
        _vXMLParameters.push_back("max_deviation");
    }

    virtual void copy(boost::shared_ptr<PlannerParameters const> r)
    {
        boost::shared_ptr<CircularBlenderParameters const> p
            = boost::dynamic_pointer_cast<CircularBlenderParameters const>(r);
        
        if (p)
        {
            is_processing_ = p->is_processing_;
            check_collision_ = p->check_collision_;
            integration_step_ = p->integration_step_;
            interpolation_step_ = p->interpolation_step_;
            max_deviation_ = p->max_deviation_;
        }

        PlannerParameters::copy(r);
    }

    bool is_processing_;    
    bool check_collision_;
    double integration_step_;
    double interpolation_step_;
    double max_deviation_;

protected:
    virtual bool serialize(std::ostream &stream) const
    {
        if (!PlannerParameters::serialize(stream))
            return false;

        stream << "<check_collision>" << integration_step_ << "</check_collision>\n"
               << "<integration_step>" << integration_step_ << "</integration_step>\n"
               << "<interpolation_step>" << interpolation_step_ << "</interpolation_step>\n"
               << "<max_deviation>" << max_deviation_ << "</max_deviation>\n";

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
             name == "check_collision"
          || name == "integration_step"
          || name == "interpolation_step"
          || name == "max_deviation";
        return is_processing_ ? PE_Support : PE_Pass;
    }

    virtual bool endElement(std::string const &name)
    {
        using boost::format;
        using boost::str;

        if (is_processing_)
        {
            if (name == "check_collision")
                _ss >> check_collision_;
            else if (name == "integration_step")
                _ss >> integration_step_;
            else if (name == "interpolation_step")
                _ss >> interpolation_step_;
            else if (name == "max_deviation")
                _ss >> max_deviation_;
            else
                RAVELOG_WARN(str(format("Unknown tag '%s'.\n") % name));

            is_processing_ = false;
            return false;
        }

        return PlannerParameters::endElement(name);
    }
};

typedef boost::shared_ptr<CircularBlenderParameters> CircularBlenderParametersPtr;
typedef boost::shared_ptr<CircularBlenderParameters const> CircularBlenderParametersConstPtr;

} // namespace or_circularblender

#endif // CIRCULARBLENDERPARAMETERS_H_
