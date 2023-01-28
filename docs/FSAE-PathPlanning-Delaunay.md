# 基于 Delaunay 三角剖分的 FSAE 路径规划算法（MATLAB实现）

> 本文为项目 [FSAE-PathPlanning-Delaunay](https://github.com/muziing/FSAE-PathPlanning-Delaunay) 的对应文档，完整代码见 <https://github.com/muziing/FSAE-PathPlanning-Delaunay>

## 引言

在 FSAE（大学生方程式赛车）无人车比赛中，赛车需要根据传感器返回的作为路径边界标识的交通锥桶的信息（颜色、位置坐标），进行路径规划，进而为下一刻的控制提供依据。良好的路径规划算法应能够根据离散的锥桶坐标，快速规划出一条光滑、连续、合理靠近车道中心线的目标路径。

![Plan the path through the cones](images/22oct3_1.png)

本文介绍了一种基于 Delaunay 三角剖分的路径规划算法：将锥桶位置视为离散点，构建 Delaunay 三角，然后以位于车道内部的那些三角形边的中点为依据进行插值，最终得到期望路径。经验证，该算法基本可以完成路径规划任务。

## 背景知识：Delaunay三角剖分

## 创建Delaunay三角剖分

## 移除外部三角形

## 获取内部边中点

## 中心点插值
