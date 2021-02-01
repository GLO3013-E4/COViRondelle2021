# COViRondelle2021 frontend

Vue.js frontend application for COViRondelle2021

- Communicates with `station` using API calls

## Installation

Run the following commands in this directory.

With Docker :
```shell
docker build -t frontend .
docker build --no-cache -t frontend . # If you have issues with packages not updating or installing
```

Without Docker :
```
yarn install
```

## Usage

### Execute app

Run the following commands in this directory.

With Docker :
```shell
docker run frontend
```

Without Docker :
```
yarn serve
```

The app will be running on [localhost:5000](http://localhost:5000).

## Contributing

The following commands do not concern Docker.

### Run tests

```
yarn test
```

### Check code style

Verify code style :
```
yarn lint
```

Fix code style :
```
yarn lint --fix
```

### Build production app

```
yarn build
```