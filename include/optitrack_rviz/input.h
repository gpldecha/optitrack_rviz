#ifndef INPUT_H_
#define INPUT_H_

// ROS

#include <ros/ros.h>

// STL

#include <vector>
#include <string>
#include <map>
#include <boost/regex.hpp>


namespace opti_rviz{

class Input{
public:

    typedef std::array<float,3> fVec3;
    typedef std::array<float,4> fVec4;

public:

    static bool process_input(int argc, char** argv,std::map<std::string,std::string>& input);


    static bool process_input(int argc, char **argv,std::map<std::string,std::string>& input,
                              fVec3& origin,fVec4& orientation,fVec3& offset);


    static void print_input_options(const std::map<std::string,std::string>& input);

    static void segment_string_space(const std::string& in, std::vector<std::string>& out);


private:

    static int find_index(int argc,const std::vector<std::string>& argv,std::string str);

    static std::string num2str(float num);

};

}


#endif
