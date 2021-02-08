import vuetify from '@/plugins/vuetify';
import {addDecorator} from "@storybook/vue";

export const parameters = {
  layout: 'centered',
}

addDecorator(() => ({
  vuetify,
  template: '<v-app><story/></v-app>'
}))

/*
const req = require.context('../src', true, /\.stories\.ts$/)

function loadStories() {
  req.keys().forEach((filename) => req(filename))
}

configure(loadStories, module);
*/