* 使用stm32f405单片机读取mlx90393磁力计
* SPI的读取比较坑，请严格按照时序图上的来走,片选端该拉低的时候一定要拉低
* CPOL 和 CPHA初始化配置要弄好，按照程序的来进行配置
* 读取效果和磁铁有关系，这块芯片非常的敏感。如果磁铁的磁性很强，读回来的数会是个方波
* 需要距离磁铁5mm以上，才能产生正弦波
