#ifndef CONFIG_H
#define CONFIG_H

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

class Config{

    public:
        static shared_ptr<Config> config_;
        
        FileStorage file_;

        Config(){}

        ~Config();

        static void setParameterFile(const string& filename)

        template<typename T>
        static T get(const string& key) {
            return T(Config::config_->file_[key]);
        }
};

#endif
