#include "CircularBlender.h"

#include <boost/make_shared.hpp>
#include <openrave/plugin.h>

using namespace OpenRAVE;
using or_circularblender::CircularBlender;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream &sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == "kunzcircularblender") {
        return boost::make_shared<CircularBlender>(penv);
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_Planner].push_back("KunzCircularBlender");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
