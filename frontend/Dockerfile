FROM node:15.6.0

WORKDIR /usr/src/app

COPY . .

RUN yarn install

EXPOSE 5000

CMD ["yarn", "serve"]
