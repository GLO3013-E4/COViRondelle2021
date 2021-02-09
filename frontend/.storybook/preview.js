import vuetify from '@/plugins/vuetify';
import {addDecorator} from "@storybook/vue";

export const parameters = {
  layout: 'centered',
}

addDecorator(() => ({
  vuetify,
  template: '<v-app><story/></v-app>'
}))
