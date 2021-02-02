# COViRondelle2021

![Robot CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Robot%20CI/badge.svg)
![Station CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Station%20CI/badge.svg)
![Frontend CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Frontend%20CI/badge.svg)
![Scripts CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Scripts%20CI/badge.svg)

Project of team 4 for course GLO-3013 at Laval University

Our team is called "Robot culinaire"!

We have three main applications and some development scripts, each with their own README : 

- [`robot`](robot) : Python application communicating with `robot` using ROS
- [`station`](station) : Python application communicating with `station` using ROS and `frontend` using websockets
- [`frontend`](frontend) : Vue.js application communicating with `station` using websockets
- [`scripts`](scripts) : Development scripts for testing different functionalities

## Installation

With Docker Compose : 
```shell
docker-compose build
docker-compose build --no-cache # If you have issues with packages not updating or installing
```

Without Docker Compose : refer to each app's README.md file.

## Usage

With Docker Compose :
```shell
docker-compose up
```

Without Docker Compose : refer to each app's README.md file.

Each app will run on : 

- Robot : [localhost:3000](http://localhost:3000) *(CAN CHANGE)*
- Station : [localhost:4000](http://localhost:4000) *(CAN CHANGE)*
- Frontend : [localhost:5000](http://localhost:5000)

## Contributing

Before contributing to the project, please read our [contribution guide](CONTRIBUTING.md). Also, please refer to each app's README.md file.


## License

`MIT` : [Read full license](LICENSE)
