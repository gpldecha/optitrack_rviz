#include "optitrack_rviz/input.h"



namespace opti_rviz{


bool Input::process_input(int argc, char** argv,std::map<std::string,std::string>& input){
    if(argc < static_cast<int>(input.size())){
        std::string error_msg = "options ";
        for(auto it = input.begin(); it != input.end();it++){
            error_msg += it->first + " ";
        }
        error_msg += "not defined!";
        ROS_ERROR("%s",error_msg.c_str());
        return false;
    }else{

        int index = 0;
        std::string empty = "";

        std::vector<std::string> input_args(argc);
        for(int i = 0; i < argc;i++){
            input_args[i] = std::string(argv[i]);
        }


        for(auto it = input.begin(); it != input.end();it++){
            index = find_index(argc,input_args,it->first);

            if(index == -1 && (it->second == empty)){
                ROS_ERROR("%s [arg] not specified",it->first.c_str());
                return false;
            }else if(index != -1){
               (it->second) =  std::string(argv[index + 1]);
            }
        }

        return true;
    }
}



bool Input::process_input(int argc, char **argv,std::map<std::string,std::string>& input,
                          std::array<float,3>& origin,std::array<float,4>& orientation,std::array<float,3>& offset){

    if(argc < static_cast<int>(input.size())){
        std::string error_msg = "options ";
        for(auto it = input.begin(); it != input.end();it++){
            error_msg += it->first + " ";
        }
        error_msg += "not defined!";
        ROS_ERROR("%s",error_msg.c_str());
        return false;
    }else{

        int index = 0;
        std::string empty = "";

        std::vector<std::string> input_args(argc);
        for(int i = 0; i < argc;i++){
            input_args[i] = std::string(argv[i]);
        }


        for(auto it = input.begin(); it != input.end();it++){
            index = find_index(argc,input_args,it->first);

            if(index == -1 && (it->second == empty)){
                ROS_ERROR("%s [arg] not specified",it->first.c_str());
                return false;
            }else if(index != -1){
                if(it->first == "-origin"){
                    origin[0] = boost::lexical_cast<float>(input_args[index+1]);
                    origin[1] = boost::lexical_cast<float>(input_args[index+2]);
                    origin[2] = boost::lexical_cast<float>(input_args[index+3]);

                    (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3];
                }else if(it->first == "-orientation"){
                    orientation[0] = boost::lexical_cast<float>(input_args[index+1]);
                    orientation[1] = boost::lexical_cast<float>(input_args[index+2]);
                    orientation[2] = boost::lexical_cast<float>(input_args[index+3]);
                    orientation[3] = boost::lexical_cast<float>(input_args[index+4]);
                    (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3] + " " + input_args[index+4];
                }else if(it->first == "-offset"){
                    offset[0] = boost::lexical_cast<float>(input_args[index+1]);
                    offset[1] = boost::lexical_cast<float>(input_args[index+2]);
                    offset[2] = boost::lexical_cast<float>(input_args[index+3]);
                    (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3];

                }else{
                    (it->second) =  std::string(argv[index + 1]);
                }
            }
        }

        return true;
    }
}


int Input::find_index(int argc,const std::vector<std::string>& argv,std::string str){
    for(int i = 0; i < argc;i++){
        if(argv[i] == str){
            return i;
        }
    }
    return -1;
}

void Input::print_input_options(const std::map<std::string,std::string>& input){
    for(auto it = input.begin(); it != input.end();it++){
        ROS_INFO("%s\t%s",it->first.c_str(),it->second.c_str());
    }
}

std::string Input::num2str(float num){
   return boost::lexical_cast<std::string>(num);
}


void Input::segment_string_space(const std::string& in, std::vector<std::string>& out){

         std::string rgx_str = "\\s+";
         boost::regex rgx (rgx_str);
         boost::sregex_token_iterator iter(in.begin(), in.end(), rgx, -1);
         boost::sregex_token_iterator end;

         out.clear();
         while (iter != end)  {
             out.push_back(*iter);
             ++iter;
         }
}



}
