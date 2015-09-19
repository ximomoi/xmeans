#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <complex>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <stdlib.h>
#include <time.h>
//#include <nag.h>
//#include <nagg05.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#define obstacle_max 100

using namespace Eigen;

class Xmeans{
public:
  Xmeans(){
    scan_sub = nh.subscribe("/scan", 10, &Xmeans::scanCallBack, this);

    obstacle_cluster_pub = nh.advertise<sensor_msgs::PointCloud>("/obstacle_cluster", 1000);

    ros::Rate loop_rate(10);
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
    //objects_info(x, y, k_dif, k_number, likelihood)
    objects_info = MatrixXf::Zero(6, 100);

    float deg = -45, rad;
    for(int i = 0; i < obstacle_max; i++){
      rad = deg * M_PI / 180;
      objects_info(1, i+1) = scan->ranges[i] * cos(rad);
      objects_info(2, i+1) = scan->ranges[i] * sin(rad);
      deg += 0.25;
    }
  }
/*
  //BIC_k
  void BIC1(){
    int q = 2 * (2 + 3) / 2;
    double bic1 = -2 * log(k_likelihood) + q * log(obstacle_max);
  }

  //BIC_k+1
  void BIC2(){

    //0.5 / 確率密度関数
    double alpha = 0.5 / (exp(- pow(beta, 2) / 2) / sqrt(2 * M_PI));

    double bic2 = -2 * (cluster_max * log(alpha) + log(likelihood(j)) + log(likelihood(j+1))) + (2 * q) * log(obstacle_max);
  }
*/

  //kmeans
  void kmeans(int k){
    int i, j, num, decision = 0, count;
    float cluster_dif, x_add, y_add;
    //cluster_info(x, y, number, calc, cov_det_add)
    MatrixXf cluster_info = MatrixXf::Zero(5, k);
    //適当にクラスタの座標を設定,urgの範囲が30m
    for(j = 1; j <= k; j++){
      cluster_info(1, j) = (((float)rand() / RAND_MAX) * 60) - 30;
      cluster_info(2, j) = (((float)rand() / RAND_MAX) * 60) - 30;
    }
    while(true){
      //各ノードと各クラスタの距離を計算し、ノードの属するクラスタを決める
      for(i = 1; i <= obstacle_max; i++){
        for(j = 1; j <= k; j++){
          cluster_info(4, j) = hypotf(objects_info(1, i) - cluster_info(1, j), objects_info(2, i) - cluster_info(2, j));
          if(j == 1){
            cluster_dif = cluster_info(4, j);
            num = j;
          }else{
            if(cluster_info(4, j - 1) > cluster_info(4, j)){
              cluster_dif = cluster_info(4, j);
              num = j;
            }
          }
        }
        //各クラスタと全ノードの距離が変化しているかどうか
        if(objects_info(3, i) == cluster_dif && objects_info(4, i) == num){
          decision += 1;
        }else{
          objects_info(3, i) = cluster_dif;
          objects_info(4, i) = num;
        }
      }
      //各クラスタと全ノードとの距離が変化しなくなった場合
      if(decision == obstacle_max){
        break;
      }
      //決めたノードの重心へクラスタを移動
      for(j = 1; j <= k; j++){
        count = 0;
        for(i = 1; i <= obstacle_max; i++){
          if(cluster_info(3, j) == objects_info(4, i)){
            x_add += objects_info(1, i);
            y_add += objects_info(2, i);
            count += 1;
          }
        }
        cluster_info(1, j) = x_add / count;
        cluster_info(2, j) = y_add / count;
      }
    }
    for(j = 1; j <= k; j++){
      obstacle_cluster.points[j].x = cluster_info(1, j);
      obstacle_cluster.points[j].y = cluster_info(2, j);
    }
    ROS_INFO("x:%f, y:%f \n", cluster_info(1, k), cluster_info(2, k));
  }
/*
  //尤度計算
  void Likelihood(){
    int i, j, count;
    //共分散行列
    cov = Matrix2d::Zero;
    for(j = 1; j <= k; j++){
      count = 0;
      for(i = 1; i <= obstacle_max; i++){
        if(cluster_info(3, j) == objects_info(4, i)){
          cov(1, 1) += pow(cluster_info(1, j) - objects_info(1, i), 2);
          cov(2, 2) += pow(cluster_info(2, j) - objects_info(2, i), 2);
          cov(1, 2) += (cluster_info(1, j) - objects_info(1, i)) * (cluster_info(2, j) - objects_info(2, i));
          count += 1;
        }
      }
      cov(1, 1) = cov(1, 1) / count;
      cov(2, 2) = cov(2, 2) / count;
      cov(1, 2) = cov(1, 2) / count;
      cov(2, 1) = cov(1, 2);

      cluster_info(5, j) += cov.determinant();

      //多変量正規分布
      f = Vector1080d::Zero;
      x = Vector2d::Zero;
      ave = Vector2d::Zero;
      ave(1) = cluster_info(1, j);
      ave(2) = cluster_info(2, j);
      for(i = 1; i <= obstacle_max; i++){
        x(1) = i;
        x(2) = i;
        f(i) = 1 / ((double)sqrt(pow(2 * M_PI, 2) * cov.determinant()) * (double)exp(- (x - ave).transpose() * cov.inverse() * (x - ave) / 2);
      }

      //尤度関数
      likelihood(j) = f.sum();

      //確認用
      ROS_INFO("likelihood%d:%f \n", j, likelihood(j));
    }
    ROS_INFO("---\n");
    //double k_likelihood = likelihood.sum();
  }
*/
/*
  //BICの比較
  void decision(){
    if(bic1 < bic2){
      k += 1;
      return false;
    }else{
      return true;
    }
  }
*/
  //run
  void run(){
    while(ros::ok()){
      int k = 1;
//      do{
//      for(i = 0; i < 2; i++){
      kmeans(k);
//        Likelihood();
//        k += 1;
//      }
//        BIC();
//        desicsion();
//      }while(decision == true);
      ros::spinOnce();
//      break;
    }
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber scan_sub;
  ros::Publisher obstacle_cluster_pub;

  sensor_msgs::PointCloud obstacle_cluster;

  //objects_info(x, y, k_dif, k_number, likelihood)
  MatrixXf objects_info;
  MatrixXf cov;
//  Vector1080d f;
  Vector2f x;
  Vector2f ave;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "xmeans");
  Xmeans dynamic_obstacle;
  dynamic_obstacle.run();

  return 0;
}
