import vuetify from '@/plugins/vuetify';
import {addDecorator} from "@storybook/vue";

const customViewports = {
  station: {
    name: 'station',
    styles: {
      width: '1680px',
      height: '1050px',
    },
  },
};

export const parameters = {
  viewport: { viewports: customViewports },
  defaultViewport: 'responsive',
}

addDecorator(() => ({
  vuetify,
  template: '<v-app><story/></v-app>'
}))
