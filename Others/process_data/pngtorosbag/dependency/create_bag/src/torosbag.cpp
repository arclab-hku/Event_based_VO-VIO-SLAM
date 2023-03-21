#include <string>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
 
void GetFileNames(string path,vector<string>& filenames, string con);
void GetFileNamesByGlob(cv::String path,vector<cv::String>& filenames, string con);
bool read_images(string path, vector<string> &image_files);
int main(int argc, char **argv)
{
    //输入文件和输出文件路径
    string base_dir = "/home/cpy/catkin_ws_pngtorosbag/src/create_bag/data/";
    string img_dir = base_dir + "right_img/";
    std::cout<<"image path is"<<img_dir<<std::endl;
    string output_bag=base_dir +"Image_right.bag";
    string img_format = ".png";//格式
    vector<string> img_names;
    //GetFileNames(img_dir, img_names,".jpg");
    read_images(img_dir, img_names);
    cout<<"图片读取完成"<<endl;
 
    cout<<"----"<<endl;
 
    ros::Time::init();
    rosbag::Bag bag;
    bag.open(output_bag, rosbag::bagmode::Write);
    int seq = 0;
    vector<string>::iterator it;
    for(it = img_names.begin(); it != img_names.end();it++)//todo 之后改成图片数量的多少
    {
        string tmp = *it;
        std::cout<<"tmp path is"<<tmp<<std::endl;
        //cout<<tmp<<endl;
        //string strImgFile =  img_dir + tmp + img_format;
        string strImgFile = tmp;
        // usleep(200000);//4hz
        usleep(13200);//20hz
        ros::Time timestamp_ros = ros::Time::now();
        
        // --- for image ---//
        cv::Mat img = cv::imread(strImgFile);
        if (img.empty())
            cout<<"图片为空: "<<strImgFile<<endl;
        cv_bridge::CvImage ros_image;
        sensor_msgs::ImagePtr ros_image_msg;
 
        ros_image.image = img;
        ros_image.encoding = "bgr8";
        //cout<<"debug_______"<<endl;
        //ros::Time timestamp_ros2 = ros::Time::now();
        ros_image_msg = ros_image.toImageMsg();
        ros_image_msg->header.seq = seq;
        ros_image_msg->header.stamp = timestamp_ros;
        ros_image_msg->header.frame_id = "/image_raw";
 
        bag.write("/camera/right/image_mono", ros_image_msg->header.stamp, ros_image_msg);
        cout<<"write frame: "<<seq<<endl;
        seq++;
    }
 
    cout<<"---end---"<<endl;
 
    return 0;
}
 
//con:文件格式 form:文件命名形式
void GetFileNames(string path,vector<string>& filenames, string con)
{
    DIR *pDir;
    struct dirent* ptr;
    string filename, format, name, name2;
 
    if(!(pDir = opendir(path.c_str())))
        return;
    int num=0;
    while((ptr = readdir(pDir))!=0) 
    {
        //跳过.和..文件
        if(strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        filename = ptr->d_name;
        format = filename.substr(filename.find("."), filename.length());
        //name = filename.substr(0, filename.find("."));
        name = filename.substr(0, filename.find("."));
        cout<<filename<<"\t"<<name<<"\t"<<format<<endl;
 
        if(format == con)//也可以添加对文件名的要求
        {
            filenames.push_back(name);
            
            num++;
        }        
    }
    std::cout<<"file size of:"<<filenames.size()<<"****"<<num<<std::endl;
    closedir(pDir);
}
 
//cv::glob(路径，要放置路径下文件定义的容器，false)
/*find_first_of()和find_last_of() 
执行简单的模式匹配，如在字符串中查找单个字符c:函数find_first_of() 查找在字符串中第1个出现的字符c，而函数find_last_of()查找最后一个出现的c。匹配的位置是返回值。如果没有匹配发生，则函数返回-1*/
//复制子字符串substr(所需的子字符串的起始位置,默认值为0 ， 复制的字符数目)返回值：一个子字符串，从其指定的位置开始
//按图片名升序排列
bool read_images(string path, vector<string> &image_files)
{
    //fn存储path目录下所有文件的路径名称，如../images/0001.png
	vector<cv::String> fn;
    cv::glob(path, fn, false);
    size_t count_1 = fn.size();
    if (count_1 == 0)
    {
        cout << "file " << path << " not  exits"<<endl;
        return -1;
    }
    //v1用来存储只剩数字的字符串
    vector<string> v1;
    for (int i = 0; i < count_1; ++i)
    {
        //cout << fn[i] << endl;
        //1.获取不带路径的文件名,000001.jpg(获取最后一个/后面的字符串）
        string::size_type iPos = fn[i].find_last_of('/') + 1;
        string filename = fn[i].substr(iPos, fn[i].length() - iPos);
        //cout << filename << endl;
        //2.获取不带后缀的文件名,000001
        string name = filename.substr(0, filename.rfind("."));
        //cout << name << endl;
        v1.push_back(name);
    }
    //把v1升序排列
    sort(v1.begin(), v1.end(),[](string a, string b) {return stoi(a) < stoi(b); });
    
    string v = ".png";
    size_t count_2 = v1.size();
    for(int j = 0; j < count_2; ++j)
    {
        string z = path + v1[j] + v;
        image_files.push_back(z);//把完整的图片名写回来
    }
	return true;
}
