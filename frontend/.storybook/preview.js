import Vue from 'vue';
import VueI18n from 'vue-i18n';
import vuetify from '@/plugins/vuetify';
import {addDecorator} from "@storybook/vue";
import { messages, defaultLocale } from '@/i18n';

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
  defaultViewport: customViewports.station,
}

Vue.use(VueI18n);
const i18n = new VueI18n({
   locale: defaultLocale,
   messages,
})

addDecorator(() => ({
  vuetify,
  i18n,
  template: '<v-app><story/></v-app>'
}))
