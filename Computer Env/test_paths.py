import numpy as np
from Simulator import PathSegment

def test_1(dt_sim):
    return [
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0)
    ]

def test_2(dt_sim):
    return [
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(90), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(30), 0],
            rho_speed=0.0)
    ]

def test_3(dt_sim):
    return [
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(0), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0)
    ]

def test_4(dt_sim):
    return [
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(45)],
            end  =[2, np.deg2rad(0), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(30), 0],
            rho_speed=0.0)
    ]

def test_5(dt_sim):
    return [
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(30)],
            end  =[2, np.deg2rad(180), np.deg2rad(60)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(60)],
            end  =[2, np.deg2rad(0), np.deg2rad(30)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.0)
    ]

def test_6(dt_sim):
    return [
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(90), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(270), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(360), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0)
    ]

def test_7(dt_sim):
    return [
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(90), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(20), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(270), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(25), 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(360), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(30), 0],
            rho_speed=0.0)
    ]

def test_8(dt_sim):
    return [
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(30)],
            end  =[4, np.deg2rad(90), np.deg2rad(60)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(60)],
            end  =[4, np.deg2rad(180), np.deg2rad(30)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(30)],
            end  =[4, np.deg2rad(270), np.deg2rad(60)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(60)],
            end  =[4, np.deg2rad(360), np.deg2rad(30)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.0)
    ]
#---Bonus Tests---
def test_9(dt_sim):
    return [
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(30)],
            end  =[4, np.deg2rad(90), np.deg2rad(60)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5)],
            rho_speed=0.2),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(60)],
            end  =[2, np.deg2rad(180), np.deg2rad(30)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(20), np.deg2rad(5)],
            rho_speed=0.2),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(30)],
            end  =[4, np.deg2rad(270), np.deg2rad(60)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(25), np.deg2rad(5)],
            rho_speed=0.2),
        PathSegment("spherical",
            start=[2, np.deg2rad(0), np.deg2rad(60)],
            end  =[2, np.deg2rad(360), np.deg2rad(30)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(30), np.deg2rad(5)],
            rho_speed=0.2)
    ]

def test_10(dt_sim):
    return [
        PathSegment("spherical",
            start=[1*5, np.deg2rad(0), np.deg2rad(10)],
            end  =[2*5, np.deg2rad(90), np.deg2rad(15)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[2*5, np.deg2rad(0), np.deg2rad(0)],
            end  =[3*5, np.deg2rad(180), np.deg2rad(20)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[3*5, np.deg2rad(0), np.deg2rad(30)],
            end  =[4*5, np.deg2rad(270), np.deg2rad(25)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[4*5, np.deg2rad(0), np.deg2rad(60)],
            end  =[5*5, np.deg2rad(360), np.deg2rad(30)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[5*5, np.deg2rad(0), np.deg2rad(30)],
            end  =[4*5, np.deg2rad(90), np.deg2rad(25)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[4*5, np.deg2rad(0), np.deg2rad(25)],
            end  =[3*5, np.deg2rad(180), np.deg2rad(20)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[3*5, np.deg2rad(0), np.deg2rad(20)],
            end  =[2*5, np.deg2rad(270), np.deg2rad(15)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6),
        PathSegment("spherical",
            start=[2*5, np.deg2rad(0), np.deg2rad(15)],
            end  =[1*5, np.deg2rad(360), np.deg2rad(10)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(5/6)],
            rho_speed=5/6)
    ]

def test_11(dt_sim):
    return [
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(10)],
            end  =[4, np.deg2rad(0), np.deg2rad(90)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(10)],
            rho_speed=1/6),
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(90)],
            end  =[4, np.deg2rad(0), np.deg2rad(170)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(10)],
            rho_speed=1/6),
        PathSegment("spherical",
            start=[4, np.deg2rad(90), np.deg2rad(170)],
            end  =[4, np.deg2rad(60), np.deg2rad(90)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(10)],
            rho_speed=1/6),
        PathSegment("spherical",
            start=[4, np.deg2rad(60), np.deg2rad(90)],
            end  =[4, np.deg2rad(-60), np.deg2rad(90)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(2*360), np.deg2rad(2*360)],
            rho_speed=1/6),    
        PathSegment("spherical",
            start=[4, np.deg2rad(30), np.deg2rad(90)],
            end  =[4, np.deg2rad(-180), np.deg2rad(10)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), np.deg2rad(10)],
            rho_speed=1/6),
    ]

def test_12(dt_sim):
    return [
        PathSegment("spherical",
            start=[200, np.deg2rad(90), np.deg2rad(45)],
            end  =[200, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(5), 0],
            rho_speed=0.0),
            PathSegment("spherical",
            start=[200, np.deg2rad(90), np.deg2rad(45)],
            end  =[200, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
            PathSegment("spherical",
            start=[200, np.deg2rad(180), np.deg2rad(45)],
            end  =[200, np.deg2rad(90), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0),
            PathSegment("spherical",
            start=[200, np.deg2rad(90), np.deg2rad(45)],
            end  =[200, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[np.deg2rad(15), 0],
            rho_speed=0.0)
    ]

def test_circle(dt_sim, angular_speed=np.deg2rad(15)):
    return [
        PathSegment("spherical",
            start=[4, np.deg2rad(0), np.deg2rad(45)],
            end  =[4, np.deg2rad(90), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[angular_speed, 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(90), np.deg2rad(45)],
            end  =[4, np.deg2rad(180), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[angular_speed, 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(180), np.deg2rad(45)],
            end  =[4, np.deg2rad(270), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[angular_speed, 0],
            rho_speed=0.0),
        PathSegment("spherical",
            start=[4, np.deg2rad(270), np.deg2rad(45)],
            end  =[4, np.deg2rad(360), np.deg2rad(45)],
            dt=dt_sim,
            angular_speed=[angular_speed, 0],
            rho_speed=0.0)
]
