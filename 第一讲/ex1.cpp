
//zhoushuailin 2020/2/21------更新验证，当给出一个三维的旋转小量w时，这个小量可以认为是四元数或者李代数，按这两种方法进行更新统一成旋转矩阵相差很小，基本相同，不再区分；
//R<---Rexp(w^)  ，注意在实际代码的过程中要注意转成Sophus的李群，不然没有operator *的重载函数
//q<---q*[1,1/2*w], 其中，[1,1/2*w]是四元数的时间导数，要记住
#include <iostream>
#include <eigen3/Eigen/Core>
//此处原来是#include <Eigen/Core>，但无法找到该文件；经过locate eigen发现位于eigen3/Eigen下。
#include <eigen3/Eigen/Geometry>
#include<eigen3/Eigen/Dense>
//#include <sophus/so3.hpp>
#include <sophus/so3.hpp>//顾名思义，用来构造李群SO3及相关运算
#include <iomanip>//控制数据输出的位数
 

using namespace std;
int main(int argc,char** argv)
{
    //定义待选转的旋转矩阵、单位四元数,使用的数据类型要处处相同，不然需要通过cast进行显示转换；同时相乘时要数据维度相同，数据类型相同
 //float和double类型一般可以选择double类型，精度更高，消耗略高于float
 //旋转向量
    Eigen::Vector3d v1(1,2,3);//旋转向量要归一化
    Eigen::AngleAxisd a1(M_PI/4,v1/v1.norm());//旋转向量

//旋转矩阵
    Eigen::Matrix3d R1=a1.toRotationMatrix();//由旋转向量得到旋转矩阵
    cout<<"R1:"<<R1 <<endl;

//so3
Sophus::SO3d R1_so3(R1);//由旋转矩阵得到so3;
cout<<"R1_so3:"<<R1_so3.log().transpose()<<endl;//输出李代数的行向量
cout<<"R1_so3:"<<R1_so3.matrix()<<endl;//李群SO3需要转换成matrix进行输出,即是变换矩阵R，只是数据类型不同
//四元数
    Eigen::Quaterniond  q1(a1);
    q1.normalize();//四元数也要进行归一化
    cout<<"q1:"<<q1.coeffs().transpose()<<endl;//没有直接的transponse,先coeffs，coeffs的 顺 序 是(x,y,z,w),w为 实 部 , 前 三 者 为 虚 部
 
    /*Eigen::Quaternionf delta_q(1,1/2*0.01,1/2*0.02,1/2*0.03);
    Eigen::Matrix3f delta_R=delta_q.toRotationMatrix();
    这一段就产生了错误，因为四元数那边有个1/2；但是旋转矩阵就没有
    */
     Eigen::Vector3d  w(0.01,0.02,0.03);
    
    //Eigen::Matrix3f delta_R(w);
   //  Eigen::Matrix3f R2=R1*delta_R;

//so3李代数进行更新
Sophus::SO3d R2_SO3=R1_so3*Sophus::SO3d::exp(w);//小量w对应李代数
cout<<"R2_SO3"<<R2_SO3.matrix()<<endl;//输出更新后的李群
 
//Eigen::Matrix3f r3=R1*Sophus::SO3f::exp(w);TONOTE 数据类型要相同，eigen下的matrix并没有重载operator*的操作，要先转换成Sophus的李群，才能进行相乘

   //四元数进行更新
     Eigen::Quaterniond delta_q(1,w(0)/2,w(1)/2,w(2)/2);
     delta_q.normalize();
    Eigen::Quaterniond q2=q1*delta_q;//note:对更新量单独归一化或者更新完进行归一化都可以，结果一致
    cout<<"q2_matrix"<<q2.toRotationMatrix()<<endl;
    cout<<q2.coeffs().transpose()<<endl;//输出更新的四元数

//error
cout<<"error="<<R2_SO3.matrix()-q2.toRotationMatrix()<<endl;
   
    return 0; 
}