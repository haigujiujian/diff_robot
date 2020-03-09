#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

using namespace  std;
using namespace  Eigen;
void printvector(vector<int>& v)
{
    for(vector<int>::iterator it=v.begin();it<v.begin()+v.size();it++)
    /*for(vector<int>::iterator it=v.begin();it!=v.end();it++)*/
    {
        cout<<*it<<" ";
    }
    cout<<endl;
}

void print(int v)
{
    cout<<v<<" "<<endl;
}

void print_matrix(vector<vector<int>>& m)
{
    for(int i=0;i<m.size();i++)
    {
       for(int j=0;j<m[i].size();j++)
       {
           cout<<m[i][j]<<" ";
       }
       cout<<endl;
    }
}

void  test01()
{
    vector<int> v;
    for(int i=0;i<10;i++)
    {
        v.push_back(i);
    }

    vector<int>::iterator pBegin=v.begin();
    vector<int>::iterator pEnd=v.end();
    for_each(pBegin,pEnd,print);
    
}
void test02()
{
    int arr[]={100,200,300,400};
    vector<int> v1(arr,arr+sizeof(arr)/sizeof(int));
    vector<int> v2(&arr[0],&arr[4]);
    
    printvector(v1);
    printvector(v2);
    
}
void test03()
{
    vector<int> v4;
    for(int i=0;i!=100000;i++)
    {
        v4.push_back(i);
    }
   // printvector(v4);
    cout<<v4.size()<<endl;
    cout<<v4.capacity()<<endl;
    

}
void test04()
{
   vector<vector<int>> matrix;
   vector<int> v1;
   vector<int> v2;
   vector<int> v3;
   for(int i=0;i<3;i++)
   {
       v1.push_back(i);
       v2.push_back(i+3);
       v3.push_back(i+6);
   }
   matrix.push_back(v1);
   matrix.push_back(v2);
   matrix.push_back(v3);
   print_matrix(matrix);
}
void test05()
{
    int n=3;
    int m=4;
    vector<vector<int>> v1(n,vector<int> (m));
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {
            v1[i][j]=i+j;
        }
    }
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<m;j++)
        {
            cout<<v1[i][j]<<" ";
        }
        cout<<endl;
    }

}
void test06()
{
    MatrixXf a(4, 1);//必须要进行初始化
	a = MatrixXf::Zero(4, 1);//初始化为0
	cout << "初始化为0" << endl << a << endl;
	a = MatrixXf::Ones(4, 1);//初始化为1，矩阵大小与初始化相关，因为是动态矩阵
	cout << "初始化为1" << endl << a << endl;
	a.setZero();//矩阵置零
    cout<<"矩阵置为零"<<endl<<a<<endl;
    MatrixXf b(1,4);
    b.setRandom();
    MatrixXf c(3,3);
    c.setIdentity();
	cout << "置单位矩阵：" << endl << c << endl;
	c.setRandom();
	MatrixXf d = c;
	d = d.inverse();
	cout << "矩阵c：" << endl << c << endl;
	cout << "矩阵a：" << endl << a << endl;
	cout << "矩阵b：" << b << endl;
	cout << "访问a(0)：" << endl << a(0) << endl;
	cout << "矩阵相乘：" << endl << a*b << endl;
	cout << "矩阵数乘：" << endl << 2 * a << endl;
	cout << "矩阵c求逆d：" << endl << d << endl;
	cout << "逆矩阵回乘：" << endl << d*c << endl;
	cout << "逆矩阵d转置：" << endl << d.transpose() << endl;
	Vector3d v(1, 2, 3);
	Vector3d w(1, 0, 0);
	cout << "向量相加：" << endl << v + w << endl;

}
int main(int argc, char *argv[])
{
    //test01();
    //test02();
    //test03();
    //test04();
    //test05();
    test06();
    /* code for main function */
    return 0;
}

