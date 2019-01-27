
陈黑手远程调试，小qq和天力为debugger
步骤：
- 在Rm_infan1下打开terminal
- git add .
- git commit -m "wtf"
- git reset --hard HEAD^
- git pull
- git checkout yuancheng

Lets try and err!


#Debug  2019.1.11

- 增加了颜色过滤。
main.cpp中，HaarD.color_type定义敌方装甲颜色，并在使用haar 分类器时候适用。

如果编译有bug,及时通知陈奕州。 如果编译通过，则调整judge_color中通道值来选择地方颜色。


# 2019.1.20
- if we want to use trackers like KCF, we should recompile opencv with contrib module.
https://github.com/opencv/opencv_contrib
- tutorials
https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/