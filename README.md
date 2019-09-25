# SLAM_Pepper
slam for pepper robotic

# Pepper SLAM 文档

仓库链接：https://github.com/chaowe/SLAM_Pepper.git
	
服务器路径：/home/weichao/workspace/workspace_python/SLAM_Pepper

## deeplab

仓库链接：https://github.com/chaowe/SLAM_Pepper/tree/master/segmentation/deeplab

服务器路径：/home/weichao/workspace/workspace_python/SLAM_Pepper/segmentation/deeplab

在`.bashrc`中添加环境变量：

```
RESEARCH_HOME=/home/weichao/workspace/workspace_python/SLAM_Pepper/segmentation
 
export PYTHONPATH="/home/weichao/workspace/workspace_python/deeplearning3:$RESEARCH_HOME:$RESEARCH_HOME/slim:$PYTHONPATH"
```

### 数据预处理 `SLAM_Pepper/segmentation/deeplab/dataset`中：

1. copy nyu_v2、pepper 数据集 至 SLAM_Pepper/segmentation/deeplab/dataset/
2. 数据集较大，没有上传github，请在从目录拷贝：/home/weichao/workspace/workspace_python/SLAM_Pepper/segmentation/deeplab/datasets

```
python build_nyuv2_8_data.py --image_format=jpg
```


### 训练


```
make train crop=641 iters=40000 bs=2 dataset=nyuv2_8 classes=9 // NYUv2-8
make train crop=641 iters=50000 bs=2 dataset=pepper classes=9 // Pepper
```

可使用 tensorboard 查看训练情况：

```
tensorboard --logdir datasets/nyuv2_8/exp/train_on_trainval_set/ --port 2333
tensorboard --logdir datasets/pepper/exp/train_on_trainval_set/ --port 2333
```

### 导出训练好的模型

固定推断的裁剪大小为 641：

```
make export crop=641 iters=40000 bs=2 dataset=nyuv2_8 classes=9 // NYUv2-8
make export crop=641 iters=50000 bs=2 dataset=pepper classes=9 // Pepper
```

### 使用测试集评估

- 不缩放，使用 641 的裁剪大小进行评估：

  ```
  make eval dataset=nyuv2 crop=641
  ```

- 将所有图像长和宽的最大值缩放到 720，使用 721 的裁剪大小进行评估：

  ```
  make eval dataset=nyuv2 crop=721 resize=720
  ```

### 推断与评估(有真实标签)

将所有图像长和宽的最大值缩放到 800，使用 801 的裁剪大小进行评估，需与 export 时的裁剪大小一致：

```
make infer_nyuv2 dataset=nyuv2_8 crop=641 resize=640 // NYUv2-8
make infer_pepper dataset=pepper crop=801 resize=800 // Pepper
```

准确率结果会输出，结果会放在 output 目录，每张图像的准确率会输出到 report.csv，可删除准确率计算的部分以便在没有真实标签的情况下执行推断。

### 推断(无真实标签)

使用训练好的 Pepper 模型，推断位于 `<pepper_left_image_dir>` 里的所有图像，所有图像缩放到 800，着色策略为 pepper：

```
python3 infer.py datasets/pepper/exp/train_on_trainval_set/export/frozen_inference_graph.pb <pepper_left_image_dir> -s 800 -c=pepper
```

### 结果

位于 /home/jiayuekai/Pepper/SemanticSegmentation/results 目录：

- all_results.txt：推断时采用不同裁剪大小与缩放大小对准确率的影响；
- final_nyuv2_640：使用 640 的缩放大小与 641 的裁剪大小时 NYUv2-8 测试集的推断结果；
- final_nyuv2_720：使用 720 的缩放大小与 721 的裁剪大小时 NYUv2-8 测试集的推断结果；
- final_pepper_800：使用 800 的缩放大小与 801 的裁剪大小时 Pepper 测试集的推断结果；
- final_nyuv2_pepper_560：使用直接在 NYUv2-8 数据集上训练的模型跑 Pepper 测试集，560 的缩放大小与 801 的裁剪大小；
- final_pepper_all0_800：使用 800 的缩放大小与 801 的裁剪大小，Pepper 数据集视频 0 全部 74 帧的推断结果。
- final_pepper_all1_800：使用 800 的缩放大小与 801 的裁剪大小，Pepper 数据集视频 1 全部 280 帧的推断结果。

## ORB-SLAM2

https://github.com/equation314/ORB_SLAM2

服务器路径：/home/jiayuekai/Pepper/ORB_SLAM2

说明：用 ORB-SLAM2 估计相机位姿

copy以下目录 Vocabulary, data 至 ORB-SLAM2(数据文件较大，没上传github):

	/home/weichao/workspace/workspace_python/deeplearning3/Pepper/ORB_SLAM2/Vocabulary
	/home/weichao/workspace/workspace_python/deeplearning3/Pepper/ORB_SLAM2/data


### 编译

```
make all
```

### 运行

输出每帧位姿 CameraTrajectory.txt 和关键帧位姿 KeyFrameTrajectory.txt，并可以实时看看三维点云

- 用 RGB-D 模式测试 TUM 数据集

  ```
  make test
  ```

- 用 RGB-D 模式测试 Pepper 数据集

  ```
  make rgbd
  ```

- 用 Stereo 模式测试 Pepper 数据集

  ```
  make stereo
  ```

- 测试 NYUv2 数据集

  ```
  make nyu_a
  ```

### 性能测试

会在 Pepper 数据集上跑 10 次，并计算绝对轨迹误差，自己改相关路径。

```
./run.sh
```

## PepperSLAM

仓库链接：https://github.com/equation314/PepperSLAM

服务器路径：/home/jiayuekai/Pepper/PepperSLAM

说明：Pepper 数据集以及其他实用脚本

### 从 Pepper 上采集图像

```
python get_images.py --ip=<IP>
```

### 相机标定

- 单目标定：

  ```
  python3 calibration/calibrate.py <base_dir>
  ```

  其他参数详见代码，输出文件为 camera_calib.yml。

- 双目标定：

  ```
  python3 calibration/calibrate_stereo.py <base_dir> --left_dir=left --right_dir=right
  ```

  其他参数详见代码，输出文件为 camera_calib.yml。

### 简单的立体匹配(SGBM)

- 一对双目图像：

  ```
  python3 stereo/stereo_match.py <left_image> <right_image>
  ```

- 多对双目图像：

  - 不使用 WLS-filter：

    ```
    python3 stereo/stereo_match_images.py data/test1/ --min-disp=2 --num-disp=32 -bs=5 -m=hh --disp_dir=disparity2
    ```

  - 使用 WLS-filter：

    ```
    python3 stereo/stereo_match_images.py data/test1/ --min-disp=2 --num-disp=32 -bs=5 -m=hh --disp_dir=disparity2_f -f
    ```

### 计算绝对轨迹误差

自动寻找最佳缩放因子：

```
python3 evalution/evaluate_ate.py data/test1/path.txt CameraTrajectory.txt -f -v
```

### 可视化轨迹

使用给定的缩放因子：

```
python3 visualization/show_trajectory.py data/test1/path.txt CameraTrajectory.txt -s 1.0
```

### 可视化点云

- 输出 PLY 格式的点云文件：

  ```
  python3 visualization/export_pointcloud.py -c camera_calib.yml --rgb=<rgb_image> --disp=<disp_data>
  ```

  输出文件为 output.ply，使用 MeshLab 等软件查看。

- 直接看点云：

  ```
  python3 visualization/show_pointcloud.py -c camera_calib.yml --rgb=<rgb_image> --disp=<disp_data>
  ```

### PointCloundView

- 编译：

  ```
  cd build
  cmake ..
  make
  ```

- 可视化点云：

  ```
  ./show_pointcloud <rgb_image> <disp_data> ../config.yml
  ```

- 使用给定的相机位姿生成点云：

  ```
  ./merge_pointclouds /Users/equation/Desktop/final_pepper_all_800 ../../../data/test1/disparity_mesh_stereo48/ ../../../CameraTrajectory.txt ../../../KeyFrameTrajectory.txt ../config.yml
  ```

- 美化点云：

  ```
  ./beautify_pointcloud <pcd_file> ../config.ymll
  ```

### 数据

- data/calibration：标定时采集的数据；
- data/old：视频 0；
- data/test1：视频 1；
- data/test2：视频 2；
- data/test3：视频 3；
- data/test4：视频 4；

### 输出结果

- data/test1/disparity2：SGBM 算法输出视差图；
- data/test1/disparity2_f：SGBM 算法 + WLS filter 输出视差图；
- data/test1/disparity_mesh_stereo48：MeshStereo 算法输出视差图；
- data/test1/sps-stereo：SPS-Stereo 算法输出视差图；
- data/test1/sgm：SPS-Stereo 自带的 SGM 算法输出视差图；
- data/test1/segmentation：deeplabv3+ 语义分割结果；
- data/test1/left1, data/test1/right1：畸变矫正后 320x180 的左右图像；
- data/test1/left, data/test1/right：畸变矫正和裁剪后 280x180 的左右图像；
- data/test1/left_d, data/test1/right_d：未畸变矫正的 280x180 的原始图像；
- data/test1/depth：Pepper 自带立体匹配算法得到的深度图；
- camera_calib.yml：双目相机标定结果；
- camera_calib2.yml：左相机单目标定结果；

## NAOqiManager

仓库链接：https://github.com/denggroup/NAOqiManager，可能需要常文泰给你访问权限

服务器路径：/home/jiayuekai/Pepper/NAOqiManager

说明：基于共享内存的主控程序(初步)

详见 README。

## Stereo

仓库链接：https://github.com/siposcsaba89/sps-stereo，https://github.com/aasharma90/MeshStereo-Linux

服务器路径：/home/jiayuekai/Pepper/Stereo

说明：SPS-Stereo 和 MeshStereo 立体匹配算法

### SPS-Stereo

图像路径都放到 images.txt 里。

```
cd build
cmake ..
make
./spsstereo ../images.txt
```

### MeshStereo

```
cd build
cmake ..
make
./run.sh
```

## PepperController

仓库链接：https://github.com/euniciaruiz/pepperController/tree/master/pepperController

服务器路径：/home/jiayuekai/Pepper/pepperController

说明：网页版简易机器人控制程序

直接打开 index.html 输入机器人 IP 即可，需要与机器人同属一局域网。

