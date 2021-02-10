import vuetify from '@/plugins/vuetify';
import {addDecorator} from "@storybook/vue";

addDecorator(() => ({
  vuetify,
  template: '<v-app><story/></v-app>'
}))
