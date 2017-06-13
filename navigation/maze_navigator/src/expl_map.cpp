#include <maze_navigator/expl_map.h>

bool TargetCell::isSimpleCell()
{
    return true;
}

int TargetCell::getId(){
    return y_ * MAX_SIZE + x_;
}

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
    return (aux_map_[y][x] & LAYER_VIS) != 0;
}

bool Explmap::isRobotReachable(int y, int x)
{
    return (aux_map_[y][x] & LAYER_ROBOT_REACHABLE) != 0;
}

bool Explmap::isTargetCell(int y, int x)
{
    return (aux_map_[y][x] & LAYER_TARGET_CELL) != 0;
}

bool Explmap::isVisitedTargetCell(int y, int x){
    return (aux_map_[y][x] & LAYER_TARGET_REACHED) != 0;
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

void Explmap::markVisitedTargetCell(int y,int x){
    aux_map_[y][x] |= LAYER_TARGET_REACHED;
}

TargetCell Explmap::getTargetCell(int y,int x){
    return t_cells_[cell_idx_[y][x]];
}

void Explmap::addLayer(int **layer_map, int height, int width, int layer_flag)
{
    if (layer_flag == LAYER_TARGET_CELL)
    {
        addTargetLayer(layer_map);
        return;
    }

    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
        {
            if (layer_map[y][x])
                aux_map_[y][x] |= layer_flag;
            else if (aux_map_[y][x] & layer_flag)
                aux_map_[y][x] ^= layer_flag;
        }
}

void Explmap::addTargetLayer(int **layer_map)
{
    t_cells_.clear();
    int t_cell_idx = 0;
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
        {
            if (layer_map[y][x])
            {
                aux_map_[y][x] |= LAYER_TARGET_CELL;
                cell_idx_[y][x] = t_cell_idx;

                t_cells_.push_back(TargetCell(y, x));

                t_cell_idx++;
            }
            else if (aux_map_[y][x] & LAYER_TARGET_CELL)
                aux_map_[y][x] ^= LAYER_TARGET_CELL;
        }
}