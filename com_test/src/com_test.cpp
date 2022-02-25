#include <com_test/com_test.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"COM_test");

    COM_TEST com_test;

    while (ros::ok())
    {
        ros::spinOnce();
    }
    
}