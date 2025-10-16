<!-- 
pip install build
python -m build
pip install twine
twine upload dist/*
-->



# pytrafficflow

A Python library for traffic flow simulation with PDE models, particle-based models, and real-world data pipelines


![car image](https://github.com/tiagomonteiro0715/pytrafficflow/blob/main/traffic_image.png)


## Architecture

```
pytrafficflow/
├── __init__.py
├── core/
│   ├── base_model.py
│   ├── vehicle.py
│   └── velocity.py
├── data/
│   ├── loader.py
│   ├── preprocess.py
│   └── synthetic/
│       └── one_road.py
├── models/
│   └── particle.py
├── tests/
│   └── lwr_tests.py
└── utils/
    └── visualization.py
```

## Citation


## Acknowledgments

This project builds upon the foundational research of Dr. Nadim Saad, whose PhD thesis at Stanford, "Nonlinear Coupled Systems of PDEs for Modeling of Multi-Lane Traffic Flow Problems," provided the framework for this Python Library.

## Project Founders

Tiago Monteiro - monteiro.t@northeastern.edu
Julian Zhou - zhou.zher@northeastern.edu
Muhammed Bilal - Bilal.muh@northeastern.edu
Joey Ebrahim - ebrahim.y@northeastern.edu
Zhidian - shdityr03@gmail.com
