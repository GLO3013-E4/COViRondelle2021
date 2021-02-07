# COViRondelle2021 scripts

Development scripts for COViRondelle2021

## Installation

With Docker :
```shell
docker build -t scripts .
```

Without Docker : 
```shell
pip install -r requirements.txt
```

## Usage

With Docker :
```shell
docker run scripts python script_to_execute.py
```

Without Docker : 
```shell
python script_to_execute.py
```

## Contributing

Before contributing to the project, please read our [contribution guide](../CONTRIBUTING.md).

To use the following commands with Docker, simply append `docker run scripts` before the command.

Check code style of a single file
```shell
pylint module/script_to_check.py
```

Check code style of all files
```shell
pylint *
```

Run single test file
```shell
pytest tests/test_file.py
```

Run all test files
```shell
pytest tests
```
