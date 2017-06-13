#include <maze_navigator/expl_map.h>

int Explmap::getLabel(int y, int x)
{
    return aux_map_[y][x];
}

void Explmap::setLabel(int y, int x, int val)
{
    aux_map_[y][x] = val;
}

bool Explmap::isVisited(int y, int x)
{
    return aux_map_ & LAYER_VIS;
}

bool Explmap::isRobotReachable(int y, int x)
{
    return aux_map_ & LAYER_ROBOT_REACHABLE;
}

bool Explmap::isTargetCell(int y, int x)
{
    return aux_map_ & LAYER_TARGET_CELL;
}

void Explmap::markVisited(int y, int x)
{
    aux_map_[y][x] |= LAYER_VIS;
}

void Explmap::clearVisited()
{
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
            if (aux_map_[y][x] & LAYER_VIS)
                aux_map_[y][x] ^= LAYER_VIS;
}

void addLayer(int **layer_map, int height, int width, int layer_flag)
{
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++){
            if(layer_map[y][x])
                aux_map_[y][x] |= layer_flag;
            else if(aux_map_[y][x] & layer_flag)
                aux_map_[y][x] ^= layer_flag;
        }
}