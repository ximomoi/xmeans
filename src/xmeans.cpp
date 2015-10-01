#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <time.h>
#include <sstream>
#include <complex>
#include <stdlib.h>
#define obstacle_max 720

using namespace Eigen;

class Xmeans{
public:
  Xmeans(){
    scan_sub = nh.subscribe("/scan", 10, &Xmeans::scanCallBack, this);
    obstacle_cluster_pub = nh.advertise<sensor_msgs::PointCloud>("/obstacle_cluster", 1000);

    //objects_info(x, y, k_distance, cluster_number, likelihood)
    objects_info = MatrixXf::Zero(5, obstacle_max);
    laser = VectorXf::Zero(obstacle_max);

    cluster.points.resize(obstacle_max);

    obstacle_last_num = obstacle_max;

    ros::Rate loop_rate(10);
  }

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
    int h = 0, i;
    float deg = 0, rad;
    obstacle_last_num = obstacle_max;
    ROS_INFO("-------------hogehoge---------------");
    for(i = 0; i < obstacle_max; i++){
      laser(i) = scan->ranges[i];
      if(laser(i) > 29){
        obstacle_last_num += -1;
      }else{
        rad = deg * M_PI / 180;
        objects_info(0, h) = scan->ranges[i] * cos(rad);
        objects_info(1, h) = scan->ranges[i] * sin(rad);
        h += 1;
      }
      deg += 0.25;
    }
  }

  void kmeans(int k, int kakunin){
    int h, i, j, count, error = 0;
    float deg_random, rad_random, range_random, distance, cluster_old_x[k], cluster_old_y[k], x_add, y_add, center_coordinate_x = 0, center_coordinate_y = 0, center_distance, center_distance_max;
    //cluster_info(x, y, number, 未使用, cov_det_add)
    cluster_info = MatrixXf::Zero(5, k);

    for(i = 0; i < obstacle_max; i++){
      if(laser(i) == 0){
        error += 1;
      }
    }
    if(error == obstacle_max){
      ROS_ERROR("Laser scan value is zero");
    }else if(obstacle_last_num == 0){
      ROS_ERROR("There is no dynamic obstacle");
    }else{
      //取得した障害物の座標を元にランダムクラスタを配置する
      for(h = 0; h < obstacle_last_num; h++){
        center_coordinate_x += objects_info(0, h);
        center_coordinate_y += objects_info(1, h);
      }
      center_coordinate_x = center_coordinate_x / obstacle_last_num;
      center_coordinate_y = center_coordinate_y / obstacle_last_num;
      for(h = 0; h < obstacle_last_num; h++){
        center_distance = hypotf(objects_info(0, h) - center_coordinate_x, objects_info(1, h) - center_coordinate_y);
        if(h == 0){
          center_distance_max = center_distance;
        }else if(center_distance_max < center_distance){
          center_distance_max = center_distance;
        }
      }

      srand(time(NULL));
      for(j = 0; j < k; j++){
        deg_random = (float)rand() / RAND_MAX * 360;
        rad_random = deg_random * M_PI / 180;
        range_random = (float)rand() / RAND_MAX * center_distance_max;
        cluster_info(0, j) = range_random * cos(rad_random) + center_coordinate_x;
        cluster_info(1, j) = range_random * sin(rad_random) + center_coordinate_y;
        cluster_info(2, j) = j;

        ROS_INFO("rand_clus_x:%f, rand_clus_y:%f", cluster_info(0, j), cluster_info(1, j));
      }

      int syuusoku = 0;
      while(true){
        int decision = 0;
        //クラスタの割り当て
        for(h = 0; h < obstacle_last_num; h++){
          for(j = 0; j < k; j++){
            distance = hypotf(objects_info(0, h) - cluster_info(0, j), objects_info(1, h) - cluster_info(1, j));
            //
//            ROS_INFO("k:%d, distance:%f", j, distance);
            if(j == 0){
              objects_info(2, h) = distance;
              objects_info(3, h) = j;
            }else if(objects_info(2, h) > distance){
              objects_info(2, h) = distance;
              objects_info(3, h) = j;
            }
          }
//          ROS_INFO("select_k:%f", objects_info(3, h));
        }
        //確認
        ROS_INFO("1-----");
        ROS_INFO("clus_x1:%f, clus_y1:%f", cluster_info(0, 0), cluster_info(1, 0));
        ROS_INFO("clus_x2:%f, clus_y2:%f", cluster_info(0, 1), cluster_info(1, 1));
        ROS_INFO("obje_rang180:%f, obje_rang360:%f, obje_rang540:%f", laser(180), laser(360), laser(540));
        ROS_INFO("range_rand:%f, deg_rand:%f", range_random, deg_random);
        ROS_INFO("------");

        //クラスタの移動
        for(j = 0; j < k; j++){
          //
          cluster_old_x[j] = cluster_info(0, j);
          cluster_old_y[j] = cluster_info(1, j);

          x_add = 0;
          y_add = 0;
          count = 0;
          for(h = 0; h < obstacle_last_num; h++){
            if(cluster_info(2, j) == objects_info(3, h)){
              x_add += objects_info(0, h);
              y_add += objects_info(1, h);
              count += 1;
            }
          }
          if(count == 0){
            ROS_INFO("cluster No:%d does not move", j);
          }else{
            cluster_info(0, j) = x_add / count;
            cluster_info(1, j) = y_add / count;
          }
          //確認
          ROS_INFO("count:%d, k:%d", count, j);
        }
        //確認
        ROS_INFO("2-----");
        ROS_INFO("clus_old_x1:%f, clus_old_y1:%f", cluster_old_x[0], cluster_old_y[0]);
        ROS_INFO("clus_old_x2:%f, clus_old_y2:%f", cluster_old_x[1], cluster_old_y[1]);
        ROS_INFO("------");


        //今回のクラスタ座標と前回のクラスタ座標を比較
        for(j = 0; j < k; j++){
          if(cluster_info(0, j) == cluster_old_x[j] && cluster_info(1, j) == cluster_old_y[j]){
            decision += 1;
          }
        }
        if(decision == k){
          break;
        }
        //確認
        ROS_INFO("3-----");
        ROS_INFO("clus_x1:%f, clus_y1:%f", cluster_info(0, 0), cluster_info(1, 0));
        ROS_INFO("clus_x2:%f, clus_y2:%f", cluster_info(0, 1), cluster_info(1, 1));
        ROS_INFO("loop:%d, kaisuu:%d, last_num:%d", kakunin, syuusoku, obstacle_last_num);
        ROS_INFO("------");

        syuusoku += 1;

      }

      for(j = 0; j < k; j++){
        cluster.points[j].x = cluster_info(0, j);
        cluster.points[j].y = cluster_info(1, j);

//        ROS_INFO("x:%f, y:%f, num:%d \n", cluster_info(0, j), cluster_info(1, j), j);
      }
      obstacle_cluster_pub.publish(cluster);
    }
    ROS_INFO("--------------finish");
  }

  void run(){
    int kakunin = 0;
    while(ros::ok()){
      int k = 2;
      kmeans(k, kakunin);
      ros::spinOnce();
      kakunin += 1;
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber scan_sub;
  ros::Publisher obstacle_cluster_pub;

  sensor_msgs::PointCloud cluster;

  MatrixXf objects_info;
  MatrixXf cluster_info;
  VectorXf laser;
  int obstacle_last_num;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "xmeans");
  Xmeans dynamic_obstacle;
  dynamic_obstacle.run();

  return 0;
}
