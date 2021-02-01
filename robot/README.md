# COViRondelle2021 robot

Robot Python application for COViRondelle2021

 - Communicates with [`station`](../station) using ROS

## Installation

With Docker : 
```shell
docker build -t robot .
```

Without Docker : 
```shell
pip install -r requirements.txt
```

## Usage

With Docker :
```shell
docker run robot
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