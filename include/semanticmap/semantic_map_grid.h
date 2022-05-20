#ifndef SEMANTICMAP_SEMANTIC_MAP_GRID_DISPLAY_H
#define SEMANTICMAP_SEMANTIC_MAP_GRID_DISPLAY_H

#include <rviz/display.h>


namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty
class IntProperty;
}

namespace semantic_map_grid
{

class SemanticMapGrid: public rviz::Display
{
    SemanticMapGrid();
    ~SemanticMapGrid();
};

}


#endif
