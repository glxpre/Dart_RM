# STM32H7DaMiaom模板

##本模板已测试can 串口 pwm imu 均可正常运行

模板使用cubeMX的cubeide模式导出，注意这一点，这很重要

注意编译选项

<https://rm.tinywang.space/JunLong_Chen/STM32H723_DaMiao_new/src/branch/main/example/build_.png>
<https://rm.tinywang.space/JunLong_Chen/STM32H723_DaMiao_new/src/branch/main/example/build.png>

注意导出选项
由于cmake模式下导出可能会生成另一套cmakelist文件，因此一定使用cubeide模式导出，并且提前复制根目录下cmakelist文件，在导出后粘贴回去，这样文件结构就不会改变
<https://rm.tinywang.space/JunLong_Chen/STM32H723_DaMiao_new/src/branch/main/example/out.png>