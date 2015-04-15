# KinectV1-Marker-based-AR

マーカー(Marker.png)を印刷して実行してください。
マーカーの中心までの距離の表示と、Depth値を表示します。

実行環境
 - VisualStudio2013 (C++)
 - OpenCV 2.4.10
 - Kinect v1 (Kinect for Windows SDK v1.8)

マーカーの実際のサイズを入れてください。9cm×9cmなら0.45fを用います。
~~~
std::vector<cv::Point3f> markerCorners3d;
bool centerOrigin = true;
if (centerOrigin)
{
	markerCorners3d.push_back(cv::Point3f(-0.45f, -0.45f, 0));
	markerCorners3d.push_back(cv::Point3f(+0.45f, -0.45f, 0));
	markerCorners3d.push_back(cv::Point3f(+0.45f, +0.45f, 0));
	markerCorners3d.push_back(cv::Point3f(-0.45f, +0.45f, 0));
}
~~~

