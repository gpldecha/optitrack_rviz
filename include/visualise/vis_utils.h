#ifndef OPTITRACK_RVIZ_VIS_UTILS_H_
#define OPTITRACK_RVIZ_VIS_UTILS_H_

#include <vector>
#include <array>

namespace opti_rviz{


typedef std::vector<std::array<float,4> > acolor;

enum class COLORS{RED,BLUE,GREEN};

void inline set_all_colors(acolor& colors,const std::array<float,4>& color){
    for(std::size_t i = 0; i < colors.size();i++){
        colors[i] = color;
    }
}

void inline print(const acolor& colors,int range = -1)
{
    if(range < 0)
    {
        range = colors.size();
    }
    for(int i = 0; i < range;i++){
        std::cout<<  colors[i][0]  << " " << colors[i][1] << " " << colors[i][2] << " " << colors[i][3] << std::endl;
    }

}

void inline set_color(std::array<float,4>& color, double alpha, COLORS color_type)
{
    color[0] = alpha;
    switch(color_type)
    {
    case COLORS::RED:
    {
        // RGB
        color[1] = 1; color[2] = 0; color[3] = 0;
        break;
    }
    case COLORS::GREEN:
    {
        // RGB
        color[1] = 0; color[2] = 1; color[3] = 0;
        break;
    }
    case COLORS::BLUE:
    {
        // RGB
        color[1] = 0; color[2] = 0; color[3] = 1;
        break;
    }
    default:
    {
        color[1] = 1; color[2] = 0; color[3] = 0;
        break;
    }

    }

}



}


#endif
