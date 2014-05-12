### 项目简介
﻿从三维建筑物点云中获取其隐式参数，例如建筑物的面一般为矩形，可以用其中3个顶点来表示。但是本项目是获取"矩形"的四个点，因为该"矩形"并非严格上的矩形，只是凸四边形，对每个建筑物平面也做同样处理。
提取出每个建筑物平面中的若干个窗户，然后结合墙面重构出整个墙面的原貌。
本项目是基于PCL开源三维点云处理库编程。PCL点云库官方网站：http://www.pointclouds.org/

在控制台下执行:
        segment_plane.exe
        please execute command line：segment_plane.exe  -particular=false -win=false -workdir=pcd_0
        -pcdfile=0.pcd   -xmlfile=archive_vecpntcldpln.xml
    或
    segment_plane.exe -help
    -ar (either archive xml file or mot) type: bool default: false
    -execfile (executable file path name) type: string default: ""
    -particular (either generate xml file particularly or mot) type: bool
      default: false
    -pcdfile (pcd file name) type: string default: ""
    -win (either windows model or mot) type: bool default: false
    -workdir (work directory name) type: string default: ""
    -xmlfile (xml file name) type: string default: ""
便可以获取使用方法。

通过定义_SCAN或_CLUSTER宏来决定是采用扫描方法还是聚类方法建立窗户模型
