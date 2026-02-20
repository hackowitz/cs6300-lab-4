![](resource/drive-strategy.png)

The design principle is that each point ($r, \theta$) obstructs some arc ($\theta\pm\phi$) that matches the car's width ($w$) at that range:

$\phi = sin^{-1}(w/r)$

The effective driveable distance in any direction is the minimum range of any point which obstructs that drive angle. We use matrix math in `numpy` to do this quickly and conveniently, taking the minimum range of any relevant obstruction:

![](./resource/obstruction-map.png)
