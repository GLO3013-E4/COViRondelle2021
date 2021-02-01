# COViRondelle2021 station

Station Python application for COViRondelle2021

 - Communicates with [`robot`](../robot) using ROS
 - Communicates with `frontend` using a Flask API

## Installation

With Docker :
```shell
docker build -t station .
```

Without Docker :
```shell
pip install -r requirements.txt
```

## Usage

With Docker :
```shell
docker run station
```

Without Docker : 
```shell
python script_to_execute.py
```

## Contributing

Before contributing to the project, please read our [contribution guide](../CONTRIBUTING.md).

Check code style of a single file
```shell
pylint module/script_to_check.py
```

Check code style of a module
```shell
pylint module
```

Run single test file
```shell
pytest tests/test_file.py
```

Run all test files
```shell
pytest tests
```