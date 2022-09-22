### A start 

#### workflow

initGridMap->setObs->AstarGraphSearch->getPath->resetUsedGrids

***Noets:*** 

1. map 原点是地图的正中心，即xy边长的一半处，因此在地图表示下（coord）min点为负数。

而在Astart索引（index）中，是以0为起始索引。

二者的换算公式如下：

index = coord * _inv_resolution + lower

2. 在A start实现中统一使用index作为距离判断，而不是coord。